#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <stdio.h>
#include <string.h>

// =====================================================
// CHANGE LOG
// =====================================================
// 001: Keep motor pin D0 and correct its comment.
// 002: Configure total wristband count and this device ID.
// 003: Ignore wristbands belonging to the same person.
// 004: Generate the name list and size all RSSI arrays
//      automatically from NUM_WRISTBANDS.
// 005: Preserve BLE names Player1, Player2, Player3, etc.
// 006: Constant 50 ms legacy advertising interval.
// 007: Increase BLE scan duty cycle to 80%.
// 008: Replace moving average with per-wristband median(5) + EMA.
// 009: Evaluate proximity every 200 ms and reset stale filters.
// 010: Allow verbose raw RSSI logs to be disabled.

// =====================================================
// DEVICE CONFIGURATION - EDIT THESE TWO VALUES
// =====================================================

// Use the same total on every wristband.
// Example: 6 wristbands = 3 people with 2 wristbands each.
const int NUM_WRISTBANDS = 4;

// Set a unique ID on each device before uploading.
// Valid IDs: 1 to NUM_WRISTBANDS.
const int MY_WRISTBAND_ID = 3;

// Consecutive pairs belong to the same person:
// Person 1: Player1 (left) + Player2 (right)
// Person 2: Player3 (left) + Player4 (right)
// Person 3: Player5 (left) + Player6 (right)

static_assert(NUM_WRISTBANDS >= 2,
              "Configure at least two wristbands.");
static_assert(NUM_WRISTBANDS % 2 == 0,
              "Use an even number: two wristbands per person.");
static_assert(MY_WRISTBAND_ID >= 1 &&
              MY_WRISTBAND_ID <= NUM_WRISTBANDS,
              "MY_WRISTBAND_ID must be between 1 and NUM_WRISTBANDS.");

// =====================================================
// MOTOR
// =====================================================

const int MOTOR_PIN = D0;  // Vibration motor control pin

// =====================================================
// AUTOMATIC WRISTBAND NAMES
// =====================================================

const int NAME_BUFFER_SIZE = 32;
char wristbandNames[NUM_WRISTBANDS][NAME_BUFFER_SIZE];

// =====================================================
// RSSI FILTER AND DETECTION STATE
// =====================================================

const int RSSI_SAMPLES = 5;
const float RSSI_EMA_ALPHA = 0.35f;
// Enable to print every received raw RSSI sample (can affect performance).
const bool DEBUG_RAW_RSSI = false;

// Array index = wristband ID - 1.
int rssiBuffer[NUM_WRISTBANDS][RSSI_SAMPLES];
int rssiIndex[NUM_WRISTBANDS] = {};
bool rssiBufferFull[NUM_WRISTBANDS] = {};
float rssiEMA[NUM_WRISTBANDS] = {};
bool rssiEMAInitialized[NUM_WRISTBANDS] = {};
bool wristbandDetected[NUM_WRISTBANDS] = {};
unsigned long lastWristbandSeen[NUM_WRISTBANDS] = {};

const unsigned long WRISTBAND_TIMEOUT = 1500;
const unsigned long RSSI_UPDATE_INTERVAL = 200;
unsigned long lastRSSIUpdate = 0;

// =====================================================
// VIBRATION STATE
// =====================================================

bool motorState = false;
unsigned long lastMotorChange = 0;

// RSSI-based proximity categories, not measured distances.
enum DistanceLevel {
  VERY_CLOSE,
  CLOSE,
  MEDIUM,
  FAR,
  VERY_FAR,
  NO_SIGNAL
};

DistanceLevel currentLevel = NO_SIGNAL;

// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

int getPersonId(int wristbandId);
bool shouldProcessWristband(int wristbandId);
int getWristbandIndex(const char* deviceName);
void scan_callback(ble_gap_evt_adv_report_t* report);
void addRSSI(int wristband, int rssi);
int getMedianRSSI(int wristband);
int getFilteredRSSI(int wristband);
void resetRSSIFilter(int wristband);
int getStrongestWristband();
DistanceLevel classifyRSSI(int rssi);
void printLevel(DistanceLevel level);
void updateBLE();
void updateVibration();
void startAdvertising();

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("Prototype 2");
  Serial.println("BLE Vibration Wristband");
  Serial.println("==============================");

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // Generate all valid BLE names and initialize RSSI buffers.
  for (int i = 0; i < NUM_WRISTBANDS; i++) {
    snprintf(wristbandNames[i], sizeof(wristbandNames[i]),
             "Player%d", i + 1);

    for (int sample = 0; sample < RSSI_SAMPLES; sample++) {
      rssiBuffer[i][sample] = -100;
    }
  }

  Serial.println("Starting Bluefruit BLE...");

  // Initialize Bluefruit with peripheral and central support.
  if (!Bluefruit.begin(1, 1)) {
    Serial.println("ERROR: Bluefruit initialization failed!");

    // Repeat the error pattern until reset or power-off.
    while (1) {
      digitalWrite(MOTOR_PIN, HIGH);
      delay(100);
      digitalWrite(MOTOR_PIN, LOW);
      delay(900);
    }
  }

  // Automatically derive this device's BLE name from its ID.
  Bluefruit.setName(wristbandNames[MY_WRISTBAND_ID - 1]);
  Bluefruit.setTxPower(4);

  Serial.print("My wristband ID: ");
  Serial.println(MY_WRISTBAND_ID);
  Serial.print("My BLE name: ");
  Serial.println(wristbandNames[MY_WRISTBAND_ID - 1]);
  Serial.print("My person ID: ");
  Serial.println(getPersonId(MY_WRISTBAND_ID));
  Serial.print("Total wristbands: ");
  Serial.println(NUM_WRISTBANDS);

  startAdvertising();

  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);

  // Scan interval = 100 ms; scan window = 80 ms (80% duty cycle).
  Bluefruit.Scanner.setInterval(160, 128);

  // Request scan-response data, which can contain the name.
  Bluefruit.Scanner.useActiveScan(true);

  Serial.println("Starting BLE scan...");
  Bluefruit.Scanner.start(0);

  Serial.println("BLE initialized.");
  Serial.println("Allowed wristbands:");

  for (int i = 0; i < NUM_WRISTBANDS; i++) {
    if (shouldProcessWristband(i + 1)) {
      Serial.print(" - ");
      Serial.println(wristbandNames[i]);
    }
  }

  Serial.println("Ignored wristbands from my person:");

  for (int i = 0; i < NUM_WRISTBANDS; i++) {
    if (getPersonId(i + 1) == getPersonId(MY_WRISTBAND_ID)) {
      Serial.print(" - ");
      Serial.println(wristbandNames[i]);
    }
  }

  Serial.println();
  Serial.println("Waiting for wristbands...");
  Serial.println();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  if (millis() - lastRSSIUpdate >= RSSI_UPDATE_INTERVAL) {
    lastRSSIUpdate = millis();
    updateBLE();
  }

  updateVibration();
}

// =====================================================
// PERSON ASSIGNMENT AND WRISTBAND FILTER
// =====================================================

int getPersonId(int wristbandId) {
  // Integer division maps 1/2 to 1, 3/4 to 2, 5/6 to 3, etc.
  return (wristbandId - 1) / 2 + 1;
}

bool shouldProcessWristband(int wristbandId) {
  if (wristbandId < 1 || wristbandId > NUM_WRISTBANDS) {
    return false;
  }

  // Ignore both this device and the other wristband
  // belonging to the same person.
  return getPersonId(wristbandId) != getPersonId(MY_WRISTBAND_ID);
}

int getWristbandIndex(const char* deviceName) {
  // Only exact names Player1 to PlayerN are accepted.
  for (int i = 0; i < NUM_WRISTBANDS; i++) {
    if (strcmp(deviceName, wristbandNames[i]) == 0) {
      return i;
    }
  }

  return -1;
}

// =====================================================
// BLE ADVERTISING
// =====================================================

void startAdvertising() {
  Bluefruit.Advertising.addFlags(
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE
  );

  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);

  // Both modes use 80 x 0.625 ms = 50 ms base interval.
  // BLE adds a random advertising delay of 0-10 ms per event.
  Bluefruit.Advertising.setInterval(80, 80);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  Serial.println("BLE advertising started.");
}

// =====================================================
// BLE SCAN CALLBACK
// =====================================================

void scan_callback(ble_gap_evt_adv_report_t* report) {
  char deviceName[NAME_BUFFER_SIZE] = {};

  uint8_t nameLength = Bluefruit.Scanner.parseReportByType(
    report,
    BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
    (uint8_t*)deviceName,
    sizeof(deviceName) - 1
  );

  if (nameLength == 0) {
    nameLength = Bluefruit.Scanner.parseReportByType(
      report,
      BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
      (uint8_t*)deviceName,
      sizeof(deviceName) - 1
    );
  }

  if (nameLength == 0) {
    Bluefruit.Scanner.resume();
    return;
  }

  deviceName[nameLength] = '\0';

  int wristbandIndex = getWristbandIndex(deviceName);

  // Reject unknown names and both wristbands of our own person.
  if (wristbandIndex < 0 ||
      !shouldProcessWristband(wristbandIndex + 1)) {
    Bluefruit.Scanner.resume();
    return;
  }

  unsigned long now = millis();
  // Prevent old RSSI history from influencing a newly detected device.
  if (now - lastWristbandSeen[wristbandIndex] > WRISTBAND_TIMEOUT) {
    resetRSSIFilter(wristbandIndex);
  }
  wristbandDetected[wristbandIndex] = true;
  lastWristbandSeen[wristbandIndex] = now;

  int rawRSSI = report->rssi;
  addRSSI(wristbandIndex, rawRSSI);

  if (DEBUG_RAW_RSSI) {
    Serial.print("WRISTBAND: ");
    Serial.print(deviceName);
    Serial.print(" | RSSI: ");
    Serial.print(rawRSSI);
    Serial.println(" dBm");
  }

  Bluefruit.Scanner.resume();
}

// =====================================================
// UPDATE BLE STATE
// =====================================================

void updateBLE() {
  unsigned long currentTime = millis();

  for (int i = 0; i < NUM_WRISTBANDS; i++) {
    if (wristbandDetected[i] &&
        currentTime - lastWristbandSeen[i] > WRISTBAND_TIMEOUT) {
      wristbandDetected[i] = false;
      resetRSSIFilter(i);
    }
  }

  int strongestWristband = getStrongestWristband();

  if (strongestWristband >= 0) {
    int strongestRSSI = getFilteredRSSI(strongestWristband);
    currentLevel = classifyRSSI(strongestRSSI);

    Serial.print("STRONGEST WRISTBAND: ");
    Serial.print(wristbandNames[strongestWristband]);
    Serial.print(" | FILTERED RSSI: ");
    Serial.print(strongestRSSI);
    Serial.print(" dBm | ");
    printLevel(currentLevel);
  } else {
    currentLevel = NO_SIGNAL;
    Serial.println("NO WRISTBAND DETECTED");
  }

  Serial.println("------------------------------");
}

// =====================================================
// RSSI MEDIAN (UP TO 5 SAMPLES) + EXPONENTIAL MOVING AVERAGE
// =====================================================

void resetRSSIFilter(int wristband) {
  rssiIndex[wristband] = 0;
  rssiBufferFull[wristband] = false;
  rssiEMAInitialized[wristband] = false;
  rssiEMA[wristband] = 0.0f;
}

int getMedianRSSI(int wristband) {
  int count = rssiBufferFull[wristband]
                ? RSSI_SAMPLES : rssiIndex[wristband];
  if (count == 0) return -100;

  int sorted[RSSI_SAMPLES];
  for (int i = 0; i < count; i++) sorted[i] = rssiBuffer[wristband][i];
  // Insertion sort; only five values, so no library dependency.
  for (int i = 1; i < count; i++) {
    int value = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = value;
  }
  if (count % 2 != 0) return sorted[count / 2];
  return (sorted[count / 2 - 1] + sorted[count / 2]) / 2;
}

void addRSSI(int wristband, int rssi) {
  rssiBuffer[wristband][rssiIndex[wristband]] = rssi;
  rssiIndex[wristband]++;
  if (rssiIndex[wristband] >= RSSI_SAMPLES) {
    rssiIndex[wristband] = 0;
    rssiBufferFull[wristband] = true;
  }

  // Update EMA exactly once per *new received sample*, not on each
  // 200 ms proximity evaluation. Each wristband has its own state.
  float median = (float)getMedianRSSI(wristband);
  if (!rssiEMAInitialized[wristband]) {
    rssiEMA[wristband] = median;
    rssiEMAInitialized[wristband] = true;
  } else {
    rssiEMA[wristband] = RSSI_EMA_ALPHA * median +
                        (1.0f - RSSI_EMA_ALPHA) * rssiEMA[wristband];
  }
}

int getFilteredRSSI(int wristband) {
  if (!rssiEMAInitialized[wristband]) return -100;
  float value = rssiEMA[wristband];
  return (int)(value >= 0 ? value + 0.5f : value - 0.5f);
}

int getStrongestWristband() {
  int strongestWristband = -1;
  int strongestRSSI = -100;

  for (int i = 0; i < NUM_WRISTBANDS; i++) {
    if (wristbandDetected[i]) {
      int filteredRSSI = getFilteredRSSI(i);

      // Example: -45 dBm is stronger than -70 dBm.
      if (filteredRSSI > strongestRSSI) {
        strongestRSSI = filteredRSSI;
        strongestWristband = i;
      }
    }
  }

  return strongestWristband;
}

// =====================================================
// RSSI CLASSIFICATION
// =====================================================

DistanceLevel classifyRSSI(int rssi) {
  if (rssi > -50) return VERY_CLOSE;
  if (rssi > -65) return CLOSE;
  if (rssi > -70) return MEDIUM;
  if (rssi > -75) return FAR;
  if (rssi > -90) return VERY_FAR;
  return NO_SIGNAL;
}

void printLevel(DistanceLevel level) {
  switch (level) {
    case VERY_CLOSE: Serial.println("VERY CLOSE"); break;
    case CLOSE:      Serial.println("CLOSE");      break;
    case MEDIUM:     Serial.println("MEDIUM");     break;
    case FAR:        Serial.println("FAR");        break;
    case VERY_FAR:   Serial.println("VERY FAR");   break;
    case NO_SIGNAL:  Serial.println("NO SIGNAL");  break;
  }
}

// =====================================================
// VIBRATION
// =====================================================
// Original pulse durations and transition behavior retained.
// The RSSI update does not restart the vibration cycle.

void updateVibration() {
  unsigned long currentTime = millis();

  switch (currentLevel) {
    case VERY_CLOSE:
      if (!motorState) {
        if (currentTime - lastMotorChange >= 100) {
          motorState = true;
          digitalWrite(MOTOR_PIN, HIGH);
          lastMotorChange = currentTime;
        }
      } else {
        if (currentTime - lastMotorChange >= 250) {
          motorState = false;
          digitalWrite(MOTOR_PIN, LOW);
          lastMotorChange = currentTime;
        }
      }
      break;

    case CLOSE:
      if (!motorState) {
        if (currentTime - lastMotorChange >= 150) {
          motorState = true;
          digitalWrite(MOTOR_PIN, HIGH);
          lastMotorChange = currentTime;
        }
      } else {
        if (currentTime - lastMotorChange >= 150) {
          motorState = false;
          digitalWrite(MOTOR_PIN, LOW);
          lastMotorChange = currentTime;
        }
      }
      break;

    case MEDIUM:
      if (!motorState) {
        if (currentTime - lastMotorChange >= 350) {
          motorState = true;
          digitalWrite(MOTOR_PIN, HIGH);
          lastMotorChange = currentTime;
        }
      } else {
        if (currentTime - lastMotorChange >= 150) {
          motorState = false;
          digitalWrite(MOTOR_PIN, LOW);
          lastMotorChange = currentTime;
        }
      }
      break;

    case FAR:
      if (!motorState) {
        if (currentTime - lastMotorChange >= 700) {
          motorState = true;
          digitalWrite(MOTOR_PIN, HIGH);
          lastMotorChange = currentTime;
        }
      } else {
        if (currentTime - lastMotorChange >= 150) {
          motorState = false;
          digitalWrite(MOTOR_PIN, LOW);
          lastMotorChange = currentTime;
        }
      }
      break;

    case VERY_FAR:
      if (!motorState) {
        if (currentTime - lastMotorChange >= 1200) {
          motorState = true;
          digitalWrite(MOTOR_PIN, HIGH);
          lastMotorChange = currentTime;
        }
      } else {
        if (currentTime - lastMotorChange >= 100) {
          motorState = false;
          digitalWrite(MOTOR_PIN, LOW);
          lastMotorChange = currentTime;
        }
      }
      break;

    case NO_SIGNAL:
      motorState = false;
      digitalWrite(MOTOR_PIN, LOW);
      break;
  }
}