/*
 * ASSISTIVE WRISTBAND V3 - LEGACY DISCOVERY + ONE PERIODIC RSSI TARGET
 * ==================================================================
 * Hardware: Seeed Studio XIAO nRF52840, motor driver connected to D0 (P0.02)
 * Platform: Zephyr / nRF Connect SDK (NOT Arduino / Bluefruit)
 *
 * DESIGN: Each wristband advertises in two ways:
 *   1. Legacy advertising (50 ms): PlayerN name, fast discovery, RSSI ranking.
 *   2. Extended + periodic advertising (100 ms / 50 ms): the periodic train.
 *
 * The scanner ALWAYS listens for legacy advertisements from other people.
 * Only the nearest eligible wristband is chosen for periodic synchronization.
 * At most ONE periodic sync object exists on this wristband at any time.
 *
 * The legacy filter ranks targets, while the periodic filter (when fresh)
 * drives the vibration level for the selected target. If periodic reports
 * stop arriving, the selected target's legacy RSSI is used automatically.
 *
 * Important: RSSI is a proximity indicator, NOT a distance in meters.
 * This is experimental firmware; the RF scheduling and motor hardware
 * must be validated before using it with participants.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/net/buf.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

/* ================================================================
 * 1. DEVICE CONFIGURATION
 *    Change the ID for each board. The total must match on all.
 *    Player1 + Player2 = person 1; Player3 + Player4 = person 2.
 * ================================================================ */
#ifndef NUM_WRISTBANDS
#define NUM_WRISTBANDS 4
#endif
#ifndef MY_WRISTBAND_ID
#define MY_WRISTBAND_ID 3
#endif

BUILD_ASSERT(NUM_WRISTBANDS >= 2, "At least two wristbands are required");
BUILD_ASSERT(NUM_WRISTBANDS % 2 == 0, "Exactly two wristbands per person");
BUILD_ASSERT(MY_WRISTBAND_ID >= 1 && MY_WRISTBAND_ID <= NUM_WRISTBANDS,
             "MY_WRISTBAND_ID must be between 1 and NUM_WRISTBANDS");
BUILD_ASSERT(NUM_WRISTBANDS <= 255, "The wire format supports IDs up to 255");

/* ================================================================
 * 2. RADIO, FILTER, AND TARGET-SWITCH PARAMETERS
 * ================================================================ */
#define LEGACY_INTERVAL_UNITS       80   /* 80 * 0.625 ms = 50 ms */
#define EXT_INTERVAL_UNITS         160   /* 160 * 0.625 ms = 100 ms */
#define PERIODIC_INTERVAL_UNITS     40   /* 40 * 1.25 ms = 50 ms */
#define SCAN_INTERVAL_UNITS        160   /* 100 ms */
#define SCAN_WINDOW_UNITS          128   /* 80 ms = 80% scan duty cycle */

#define RSSI_SAMPLES                 5
#define EMA_ALPHA                 0.35f
#define WRISTBAND_TIMEOUT_MS      1500
#define RSSI_UPDATE_INTERVAL_MS    200
#define PERIODIC_FRESH_MS          600  /* Fallback after missing several reports */
#define ADV_METADATA_TIMEOUT_MS    3000

#define TARGET_SWITCH_MARGIN_DB       5 /* New peer must be >= 5 dB stronger */
#define TARGET_SWITCH_CONFIRM_MS   1000 /* ...for at least 1 second */
#define TARGET_MIN_HOLD_MS         1000 /* Prevent immediate re-switch */
#define SYNC_ESTABLISH_TIMEOUT_MS  5000 /* Host API itself has no timeout */
#define SYNC_RETRY_COOLDOWN_MS     3000 /* Avoid rapid failed sync retries */
#define SYNC_TIMEOUT_UNITS          200 /* 200 * 10 ms = 2 seconds */

#define PRINT_DIAGNOSTICS             1 /* Set to 0 for less logging */
#define DIAGNOSTIC_PERIOD_MS       1000

/* ================================================================
 * 3. MOTOR PIN AND PROJECT ADVERTISING FORMAT
 * ================================================================ */
/* Seeed XIAO D0 maps to nRF52840 P0.02. Use an external motor driver. */
#define MOTOR_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTOR_GPIO_PIN  2
static const struct device *const motor_gpio = DEVICE_DT_GET(MOTOR_GPIO_NODE);

/* 0xFFFF is a development-only manufacturer identifier, NOT assigned
 * to this project by the Bluetooth SIG. Change this before production.
 * Payload: [company_low, company_high, marker, wristband_id].
 */
#define PROJECT_COMPANY_LOW  0xFF
#define PROJECT_COMPANY_HIGH 0xFF
#define MARKER_LEGACY       0xA6
#define MARKER_PERIODIC     0xA7

static uint8_t legacy_payload[] = {
    PROJECT_COMPANY_LOW, PROJECT_COMPANY_HIGH,
    MARKER_LEGACY, MY_WRISTBAND_ID
};
static uint8_t periodic_payload[] = {
    PROJECT_COMPANY_LOW, PROJECT_COMPANY_HIGH,
    MARKER_PERIODIC, MY_WRISTBAND_ID
};
static char my_device_name[24];
static struct bt_le_ext_adv *legacy_adv;
static struct bt_le_ext_adv *periodic_adv;

/* ================================================================
 * 4. PER-PEER FILTER STATES
 *    Keep legacy and periodic measurements SEPARATE, because they
 *    arrive on different radio channels and can have different RSSI.
 * ================================================================ */
struct rssi_filter {
    int samples[RSSI_SAMPLES];
    uint8_t count;
    uint8_t next;
    float ema;
    bool initialized;
    int64_t last_sample_ms;
    uint32_t received;
};

struct wristband_state {
    struct rssi_filter legacy;
    struct rssi_filter periodic;
    bool sync_metadata_valid;
    bt_addr_le_t periodic_address;
    uint8_t periodic_sid;
    int64_t metadata_time_ms;
};

static struct wristband_state wristbands[NUM_WRISTBANDS];
K_MUTEX_DEFINE(state_mutex);

/* ================================================================
 * 5. SINGLE PERIODIC SYNC STATE
 *    All Bluetooth API create/delete calls happen in the main loop.
 *    Callbacks only update state; they never create another sync.
 * ================================================================ */
static struct bt_le_per_adv_sync *active_sync;
static int sync_target_id = -1;     /* PlayerN currently being synchronized */
static bool sync_established;
static int64_t sync_started_ms;
static int64_t last_sync_action_ms;

static int selected_target_id = -1; /* Chosen from LEGACY RSSI only */
static int candidate_target_id = -1;
static int64_t candidate_since_ms;
static int64_t selected_since_ms;

/* ================================================================
 * 6. DISTANCE LEVELS AND ORIGINAL VIBRATION STATE
 *    These are the SAME RSSI boundaries and motor timings as in V1.
 * ================================================================ */
enum distance_level {
    VERY_CLOSE, CLOSE, MEDIUM, FAR, VERY_FAR, NO_SIGNAL
};
static enum distance_level current_level = NO_SIGNAL;
static bool motor_state;
static int64_t last_motor_change_ms;
static int64_t last_rssi_evaluation_ms;
static int64_t last_diagnostic_ms;

/* ================================================================
 * 7. PLAYERS AND PAIR EXCLUSION
 * ================================================================ */
static int get_person_id(int wristband_id)
{
    return (wristband_id - 1) / 2 + 1;
}

static bool should_process_wristband(int wristband_id)
{
    if (wristband_id < 1 || wristband_id > NUM_WRISTBANDS) {
        return false;
    }
    /* Ignore the local wristband AND the paired wristband of this person. */
    return get_person_id(wristband_id) != get_person_id(MY_WRISTBAND_ID);
}

/* ================================================================
 * 8. RSSI FILTER: 5-POINT MEDIAN FOLLOWED BY EMA alpha=0.35
 * ================================================================ */
static void filter_reset(struct rssi_filter *filter)
{
    filter->count = 0;
    filter->next = 0;
    filter->ema = 0.0f;
    filter->initialized = false;
    filter->last_sample_ms = 0;
}

static float filter_median(const struct rssi_filter *filter)
{
    int ordered[RSSI_SAMPLES];
    for (int i = 0; i < filter->count; i++) {
        ordered[i] = filter->samples[i];
    }
    /* Five samples maximum: insertion sort is simple and inexpensive. */
    for (int i = 1; i < filter->count; i++) {
        const int value = ordered[i];
        int j = i - 1;
        while (j >= 0 && ordered[j] > value) {
            ordered[j + 1] = ordered[j];
            j--;
        }
        ordered[j + 1] = value;
    }
    if (filter->count & 1) {
        return (float)ordered[filter->count / 2];
    }
    const int right = filter->count / 2;
    return 0.5f * (ordered[right - 1] + ordered[right]);
}

static void filter_add(struct rssi_filter *filter, int rssi, int64_t now_ms)
{
    if (rssi < -110 || rssi > 20) {
        return; /* Reject invalid or unreasonable radio values. */
    }
    if (filter->initialized &&
        now_ms - filter->last_sample_ms > WRISTBAND_TIMEOUT_MS) {
        filter_reset(filter); /* Do not blend new readings with stale ones. */
    }
    filter->samples[filter->next] = rssi;
    filter->next = (filter->next + 1) % RSSI_SAMPLES;
    if (filter->count < RSSI_SAMPLES) {
        filter->count++;
    }
    const float median = filter_median(filter);
    filter->ema = filter->initialized
        ? EMA_ALPHA * median + (1.0f - EMA_ALPHA) * filter->ema
        : median;
    filter->initialized = true;
    filter->last_sample_ms = now_ms;
    filter->received++;
}

static bool filter_is_fresh(const struct rssi_filter *filter,
                            int64_t now_ms, int64_t max_age_ms)
{
    return filter->initialized &&
           now_ms - filter->last_sample_ms <= max_age_ms;
}

/* Round a negative RSSI estimate to the nearest integer. */
static int filter_value(const struct rssi_filter *filter)
{
    return filter->ema < 0.0f
        ? (int)(filter->ema - 0.5f)
        : (int)(filter->ema + 0.5f);
}

static enum distance_level classify_rssi(int rssi)
{
    if (rssi > -50) return VERY_CLOSE;
    if (rssi > -65) return CLOSE;
    if (rssi > -70) return MEDIUM;
    if (rssi > -75) return FAR;
    if (rssi > -90) return VERY_FAR;
    return NO_SIGNAL;
}

static const char *level_name(enum distance_level level)
{
    switch (level) {
    case VERY_CLOSE: return "VERY_CLOSE";
    case CLOSE:      return "CLOSE";
    case MEDIUM:     return "MEDIUM";
    case FAR:        return "FAR";
    case VERY_FAR:   return "VERY_FAR";
    default:         return "NO_SIGNAL";
    }
}

/* ================================================================
 * 9. PARSE THE PLAYER ID FROM BLE ADVERTISING DATA
 *    - New firmware uses marker A6 (legacy) or A7 (extended).
 *    - Original Arduino firmware can still be found by name PlayerN.
 *      It cannot provide periodic data, so it remains legacy-only.
 * ================================================================ */
struct parsed_advert {
    int id;
    uint8_t marker;
    bool matched;
};

static struct parsed_advert parse_advert(struct net_buf_simple *buf)
{
    struct parsed_advert result = { .id = -1, .marker = 0, .matched = false };
    int name_id = -1;
    struct net_buf_simple data = *buf;

    while (data.len >= 2) {
        const uint8_t len = net_buf_simple_pull_u8(&data);
        if (len == 0 || len > data.len) {
            break;
        }
        const uint8_t type = net_buf_simple_pull_u8(&data);
        const uint8_t payload_length = len - 1;
        const uint8_t *payload = data.data;

        if (type == BT_DATA_MANUFACTURER_DATA && payload_length >= 4 &&
            payload[0] == PROJECT_COMPANY_LOW &&
            payload[1] == PROJECT_COMPANY_HIGH &&
            (payload[2] == MARKER_LEGACY || payload[2] == MARKER_PERIODIC)) {
            result.id = (int)payload[3];
            result.marker = payload[2];
            result.matched = true;
        } else if ((type == BT_DATA_NAME_COMPLETE ||
                    type == BT_DATA_NAME_SHORTENED) &&
                   payload_length >= 7 && payload_length <= 12 &&
                   memcmp(payload, "Player", 6) == 0) {
            int parsed_id = 0;
            bool valid_digits = true;
            for (int j = 6; j < payload_length; j++) {
                if (payload[j] < '0' || payload[j] > '9') {
                    valid_digits = false;
                    break;
                }
                parsed_id = parsed_id * 10 + (payload[j] - '0');
            }
            if (valid_digits && parsed_id >= 1 && parsed_id <= NUM_WRISTBANDS) {
                name_id = parsed_id;
            }
        }
        net_buf_simple_pull(&data, payload_length);
    }
    if (!result.matched && name_id > 0) {
        result.id = name_id;
        result.marker = MARKER_LEGACY;
        result.matched = true;
    }
    return result;
}

/* ================================================================
 * 10. SCANNER CALLBACK
 *     Legacy RSSI is recorded for ALL foreign wristbands.
 *     Extended reports are ONLY used to learn the address + SID.
 * ================================================================ */
static void scan_received(const struct bt_le_scan_recv_info *info,
                          struct net_buf_simple *buf)
{
    const struct parsed_advert parsed = parse_advert(buf);
    if (!parsed.matched || !should_process_wristband(parsed.id)) {
        return;
    }
    const int64_t now = k_uptime_get();
    struct wristband_state *peer = &wristbands[parsed.id - 1];

    k_mutex_lock(&state_mutex, K_FOREVER);
    if (parsed.marker == MARKER_LEGACY) {
        /* Only the legacy stream determines who is currently nearest. */
        if (info->rssi != BT_GAP_RSSI_INVALID) {
            filter_add(&peer->legacy, info->rssi, now);
        }
    } else if (parsed.marker == MARKER_PERIODIC && info->interval > 0 &&
               info->sid <= 0x0F) {
        /* Store the latest extended-advertising sync information.
         * A sync cannot be created from a legacy report alone.
         */
        bt_addr_le_copy(&peer->periodic_address, info->addr);
        peer->periodic_sid = info->sid;
        peer->metadata_time_ms = now;
        peer->sync_metadata_valid = true;
    }
    k_mutex_unlock(&state_mutex);
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_received,
};

/* ================================================================
 * 11. PERIODIC SYNC CALLBACKS
 *     Unlike the earlier prototype, the received periodic samples
 *     NEVER influence selection of the nearest wristband.
 * ================================================================ */
static void sync_success(struct bt_le_per_adv_sync *sync,
                         struct bt_le_per_adv_sync_synced_info *info)
{
    ARG_UNUSED(info);
    k_mutex_lock(&state_mutex, K_FOREVER);
    if (sync == active_sync) {
        sync_established = true;
    }
    k_mutex_unlock(&state_mutex);
}

static void sync_lost(struct bt_le_per_adv_sync *sync,
                      const struct bt_le_per_adv_sync_term_info *info)
{
    ARG_UNUSED(info);
    k_mutex_lock(&state_mutex, K_FOREVER);
    if (sync == active_sync) {
        /* Zephyr frees a lost synchronization before invoking .term.
         * Never call bt_le_per_adv_sync_delete() again for this object.
         */
        active_sync = NULL;
        sync_target_id = -1;
        sync_established = false;
        last_sync_action_ms = k_uptime_get();
    }
    k_mutex_unlock(&state_mutex);
}

static void periodic_received(struct bt_le_per_adv_sync *sync,
                              const struct bt_le_per_adv_sync_recv_info *info,
                              struct net_buf_simple *buf)
{
    if (buf == NULL || info->rssi == BT_GAP_RSSI_INVALID) {
        return;
    }
    const struct parsed_advert parsed = parse_advert(buf);
    if (!parsed.matched || parsed.marker != MARKER_PERIODIC ||
        !should_process_wristband(parsed.id)) {
        return;
    }
    const int64_t now = k_uptime_get();
    k_mutex_lock(&state_mutex, K_FOREVER);
    if (sync == active_sync && sync_established &&
        parsed.id == sync_target_id && parsed.id == selected_target_id) {
        filter_add(&wristbands[parsed.id - 1].periodic, info->rssi, now);
    }
    k_mutex_unlock(&state_mutex);
}

static struct bt_le_per_adv_sync_cb periodic_callbacks = {
    .synced = sync_success,
    .term = sync_lost,
    .recv = periodic_received,
};

/* ================================================================
 * 12. SELECT THE NEAREST WRISTBAND USING LEGACY RSSI ONLY
 *     - Instant selection when no target has been selected.
 *     - Switch only when a new candidate is at least 5 dB stronger
 *       for 1 second, avoiding rapid target oscillation.
 *     - Remove a target immediately after the legacy 1500 ms timeout.
 * ================================================================ */
static void select_nearest_legacy_target(int64_t now)
{
    int best_id = -1;
    int best_value = -128;
    int current_value = -128;
    bool current_is_fresh = false;

    /* The caller takes the mutex before examining the per-peer states. */
    for (int i = 0; i < NUM_WRISTBANDS; i++) {
        const int id = i + 1;
        if (!should_process_wristband(id)) {
            continue;
        }
        const struct rssi_filter *f = &wristbands[i].legacy;
        if (!filter_is_fresh(f, now, WRISTBAND_TIMEOUT_MS)) {
            continue;
        }
        const int value = filter_value(f);
        if (value > best_value) {
            best_id = id;
            best_value = value;
        }
        if (id == selected_target_id) {
            current_value = value;
            current_is_fresh = true;
        }
    }

    if (!current_is_fresh) {
        selected_target_id = best_id;
        selected_since_ms = now;
        candidate_target_id = -1;
        candidate_since_ms = 0;
        return;
    }
    if (best_id < 0 || best_id == selected_target_id ||
        best_value < current_value + TARGET_SWITCH_MARGIN_DB ||
        now - selected_since_ms < TARGET_MIN_HOLD_MS) {
        candidate_target_id = -1;
        candidate_since_ms = 0;
        return;
    }
    if (candidate_target_id != best_id) {
        candidate_target_id = best_id;
        candidate_since_ms = now;
        return;
    }
    if (now - candidate_since_ms >= TARGET_SWITCH_CONFIRM_MS) {
        selected_target_id = best_id;
        selected_since_ms = now;
        candidate_target_id = -1;
        candidate_since_ms = 0;
    }
}

/* ================================================================
 * 13. UPDATE PROXIMITY AND VIBRATION LEVEL
 *     Use fresh periodic RSSI for the currently selected target.
 *     Otherwise fall back to the same target's legacy RSSI.
 *     Do NOT allow periodic RSSI to influence which target is chosen.
 * ================================================================ */
static void update_proximity(int64_t now)
{
    int target_id;
    int rssi = -128;
    bool use_periodic = false;
    enum distance_level new_level = NO_SIGNAL;
    uint32_t legacy_count = 0;
    uint32_t periodic_count = 0;
    bool sync_state_for_print = false;

    k_mutex_lock(&state_mutex, K_FOREVER);
    select_nearest_legacy_target(now);
    target_id = selected_target_id;
    if (target_id > 0) {
        const struct wristband_state *peer = &wristbands[target_id - 1];
        const bool periodic_ready = (active_sync != NULL &&
                                     sync_target_id == target_id &&
                                     sync_established);
        if (periodic_ready &&
            filter_is_fresh(&peer->periodic, now, PERIODIC_FRESH_MS)) {
            rssi = filter_value(&peer->periodic);
            use_periodic = true;
        } else if (filter_is_fresh(&peer->legacy, now, WRISTBAND_TIMEOUT_MS)) {
            rssi = filter_value(&peer->legacy);
        }
        legacy_count = peer->legacy.received;
        periodic_count = peer->periodic.received;
    }
    if (rssi != -128) {
        new_level = classify_rssi(rssi);
    }
    current_level = new_level;
    sync_state_for_print = sync_established;
    k_mutex_unlock(&state_mutex);

#if PRINT_DIAGNOSTICS
    if (now - last_diagnostic_ms >= DIAGNOSTIC_PERIOD_MS) {
        last_diagnostic_ms = now;
        if (target_id > 0) {
            printk("TARGET Player%d | %s RSSI=%d dBm | %s | "
                   "legacy=%u periodic=%u | sync=%d\n",
                   target_id, use_periodic ? "PERIODIC" : "LEGACY",
                   rssi, level_name(new_level), legacy_count,
                   periodic_count, sync_state_for_print ? 1 : 0);
        } else {
            printk("NO WRISTBAND DETECTED | NO_SIGNAL\n");
        }
    }
#endif
}

/* ================================================================
 * 14. MANAGE EXACTLY ONE PERIODIC SYNCHRONIZATION
 *     Bluetooth create/delete calls are NEVER made from callbacks.
 *     Do not attempt to sync until matching extended metadata exists.
 *     A sync attempt is cancelled if it cannot establish in 5 seconds.
 * ================================================================ */
static void manage_periodic_sync(int64_t now)
{
    struct bt_le_per_adv_sync *sync_to_delete = NULL;
    bool should_start = false;
    int desired_id = -1;
    bt_addr_le_t address;
    uint8_t sid = 0;

    k_mutex_lock(&state_mutex, K_FOREVER);
    desired_id = selected_target_id;

    /* A different target or a pending sync that took too long requires
     * cleanup. The Bluetooth delete API is called AFTER releasing the lock.
     */
    if (active_sync != NULL &&
        (sync_target_id != desired_id ||
         (!sync_established &&
          now - sync_started_ms >= SYNC_ESTABLISH_TIMEOUT_MS))) {
        sync_to_delete = active_sync;
    }

    if (sync_to_delete == NULL && active_sync == NULL &&
        desired_id > 0 &&
        now - last_sync_action_ms >= SYNC_RETRY_COOLDOWN_MS) {
        const struct wristband_state *peer = &wristbands[desired_id - 1];
        if (peer->sync_metadata_valid &&
            now - peer->metadata_time_ms <= ADV_METADATA_TIMEOUT_MS) {
            bt_addr_le_copy(&address, &peer->periodic_address);
            sid = peer->periodic_sid;
            should_start = true;
        }
    }
    k_mutex_unlock(&state_mutex);

    if (sync_to_delete != NULL) {
        const int err = bt_le_per_adv_sync_delete(sync_to_delete);
        /* On a successful delete, Zephyr calls .term for established
         * syncs (which clears active_sync). Pending cancellation may NOT
         * emit .term: clear the app pointer after a successful request.
         * On an error, keep the pointer and try cleanup again later.
         */
        k_mutex_lock(&state_mutex, K_FOREVER);
        if (err == 0) {
            if (active_sync == sync_to_delete) {
                active_sync = NULL;
                sync_target_id = -1;
                sync_established = false;
            }
            last_sync_action_ms = now;
        }
        k_mutex_unlock(&state_mutex);
        if (err != 0) {
            printk("WARNING: periodic sync delete failed: %d; legacy continues\n", err);
        } else {
            printk("Periodic sync released; legacy scanning continues\n");
        }
        return;
    }

    if (!should_start) {
        return;
    }
    struct bt_le_per_adv_sync_param params = {0};
    bt_addr_le_copy(&params.addr, &address);
    params.sid = sid;
    params.skip = 0;
    params.timeout = SYNC_TIMEOUT_UNITS;

    struct bt_le_per_adv_sync *new_sync = NULL;
    const int err = bt_le_per_adv_sync_create(&params, &new_sync);
    k_mutex_lock(&state_mutex, K_FOREVER);
    last_sync_action_ms = now;
    if (err == 0) {
        active_sync = new_sync;
        sync_target_id = desired_id;
        sync_started_ms = now;
        sync_established = false;
        /* A new synchronization must start with its own filter state. */
        filter_reset(&wristbands[desired_id - 1].periodic);
    }
    k_mutex_unlock(&state_mutex);

    if (err == 0) {
        printk("Attempting periodic sync with Player%d (SID=%u)\n",
               desired_id, sid);
    } else {
        printk("Periodic sync create failed for Player%d: %d; legacy continues\n",
               desired_id, err);
    }
}

/* ================================================================
 * 15. ORIGINAL VIBRATION PATTERNS - UNCHANGED DURATIONS
 *     The motor cycle is NOT restarted when the RSSI level changes.
 * ================================================================ */
static void motor_set(bool on)
{
    gpio_pin_set(motor_gpio, MOTOR_GPIO_PIN, on ? 1 : 0);
    motor_state = on;
}

static void update_vibration(int64_t now)
{
    int off_ms = 0;
    int on_ms = 0;
    switch (current_level) {
    case VERY_CLOSE: off_ms = 100;  on_ms = 250; break;
    case CLOSE:      off_ms = 150;  on_ms = 150; break;
    case MEDIUM:     off_ms = 350;  on_ms = 150; break;
    case FAR:        off_ms = 700;  on_ms = 150; break;
    case VERY_FAR:   off_ms = 1200; on_ms = 100; break;
    case NO_SIGNAL:
        motor_set(false);
        /* As in the original code, last_motor_change_ms is NOT reset. */
        return;
    }
    /* Exactly the original non-blocking ON/OFF state-machine behavior. */
    if (now - last_motor_change_ms >= (motor_state ? on_ms : off_ms)) {
        motor_set(!motor_state);
        last_motor_change_ms = now;
    }
}

/* ================================================================
 * 16. START LEGACY ADVERTISING + EXTENDED/PERIODIC ADVERTISING
 *     Both sets share the same local Bluetooth identity.
 *     If periodic advertising fails, legacy discovery still starts.
 * ================================================================ */
static int start_legacy_advertising(void)
{
    const struct bt_le_adv_param params = {
        .options = 0, /* A legacy nonconnectable/non-scannable PDU. */
        .interval_min = LEGACY_INTERVAL_UNITS,
        .interval_max = LEGACY_INTERVAL_UNITS,
    };
    int err = bt_le_ext_adv_create(&params, NULL, &legacy_adv);
    if (err) return err;

    /* Keep PlayerN visible to the previous project's BLE discovery code.
     * The name and ID fit in 31 bytes of legacy advertising data.
     */
    const struct bt_data data[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA(BT_DATA_NAME_COMPLETE, my_device_name, strlen(my_device_name)),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, legacy_payload, sizeof(legacy_payload)),
    };
    err = bt_le_ext_adv_set_data(legacy_adv, data, ARRAY_SIZE(data), NULL, 0);
    if (err) return err;
    return bt_le_ext_adv_start(legacy_adv, BT_LE_EXT_ADV_START_DEFAULT);
}

static int start_periodic_advertising(void)
{
    const struct bt_le_adv_param params = {
        .options = BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_NO_2M,
        .interval_min = EXT_INTERVAL_UNITS,
        .interval_max = EXT_INTERVAL_UNITS,
        .sid = 1,
    };
    int err = bt_le_ext_adv_create(&params, NULL, &periodic_adv);
    if (err) return err;

    const struct bt_data data[] = {
        BT_DATA(BT_DATA_MANUFACTURER_DATA, periodic_payload, sizeof(periodic_payload)),
    };
    err = bt_le_ext_adv_set_data(periodic_adv, data, ARRAY_SIZE(data), NULL, 0);
    if (err) return err;

    const struct bt_le_per_adv_param periodic_params = {
        .interval_min = PERIODIC_INTERVAL_UNITS,
        .interval_max = PERIODIC_INTERVAL_UNITS,
    };
    err = bt_le_per_adv_set_param(periodic_adv, &periodic_params);
    if (err) return err;
    err = bt_le_per_adv_set_data(periodic_adv, data, ARRAY_SIZE(data));
    if (err) return err;
    /* Zephyr's periodic advertiser example enables the periodic train
     * BEFORE starting the extended advertising event that announces it.
     */
    err = bt_le_per_adv_start(periodic_adv);
    if (err) return err;
    return bt_le_ext_adv_start(periodic_adv, BT_LE_EXT_ADV_START_DEFAULT);
}

/* ================================================================
 * 17. ORIGINAL ERROR VIBRATION PATTERN
 *     A fatal setup error repeats 100 ms ON + 900 ms OFF until reset.
 *     This is the same indication used by the original Bluefruit code.
 * ================================================================ */
static void fatal_error_pattern(void)
{
    while (true) {
        motor_set(true);
        k_sleep(K_MSEC(100));
        motor_set(false);
        k_sleep(K_MSEC(900));
    }
}

/* ================================================================
 * 18. MAIN SETUP AND NON-BLOCKING LOOP
 * ================================================================ */
int main(void)
{
    int err;
    if (!device_is_ready(motor_gpio)) {
        printk("ERROR: motor GPIO controller not ready\n");
        return 0;
    }
    err = gpio_pin_configure(motor_gpio, MOTOR_GPIO_PIN, GPIO_OUTPUT_INACTIVE);
    if (err) {
        printk("ERROR: motor GPIO configure failed: %d\n", err);
        return 0;
    }

    snprintf(my_device_name, sizeof(my_device_name), "Player%d", MY_WRISTBAND_ID);
    printk("Assistive Wristband V3 | %s | Person%d | total=%d\n",
           my_device_name, get_person_id(MY_WRISTBAND_ID), NUM_WRISTBANDS);
    printk("Legacy ranks peers, periodic measures one chosen peer.\n");

    err = bt_enable(NULL);
    if (err) {
        printk("ERROR: Bluetooth initialization failed: %d\n", err);
        fatal_error_pattern();
    }
    bt_le_scan_cb_register(&scan_callbacks);
    bt_le_per_adv_sync_cb_register(&periodic_callbacks);

    err = start_legacy_advertising();
    if (err) {
        printk("ERROR: Legacy advertising unavailable (%d); stopping\n", err);
        fatal_error_pattern();
    }
    printk("Legacy Advertising: 50 ms\n");

    err = start_periodic_advertising();
    if (err) {
        printk("WARNING: Periodic advertising unavailable (%d). "
               "Legacy-only fallback enabled.\n", err);
    } else {
        printk("Extended Advertising: 100 ms, Periodic Advertising: 50 ms\n");
    }

    /* Active scan preserves the ORIGINAL ability to read PlayerN in
     * scan-response packets from older Bluefruit/Arduino wristbands.
     * New V3 firmware also sends an ID inside the legacy advert itself.
     * Extended scanning is enabled by CONFIG_BT_EXT_ADV.
     */
    const struct bt_le_scan_param scan_params = {
        .type = BT_LE_SCAN_TYPE_ACTIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = SCAN_INTERVAL_UNITS,
        .window = SCAN_WINDOW_UNITS,
    };
    err = bt_le_scan_start(&scan_params, NULL);
    if (err) {
        printk("ERROR: Scanning could not start: %d\n", err);
        fatal_error_pattern();
    }
    printk("Scanning: 100 ms interval / 80 ms window (80%%)\n");

    last_sync_action_ms = k_uptime_get() - SYNC_RETRY_COOLDOWN_MS;
    while (true) {
        const int64_t now = k_uptime_get();
        if (now - last_rssi_evaluation_ms >= RSSI_UPDATE_INTERVAL_MS) {
            last_rssi_evaluation_ms = now;
            update_proximity(now);
        }
        manage_periodic_sync(now);
        update_vibration(k_uptime_get());
        k_sleep(K_MSEC(10)); /* Does not block BLE callbacks. */
    }
    return 0;
}
