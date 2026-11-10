# Assistive Wristband V4 – Hybrid BLE + experimental Nordic Distance Measurement

**Language:** English source comments, German installation guide in `START_HIER_DE.md`.

**Status: source-level prototype; NOT compiled here and NOT tested on hardware.**

This is a real modification of our existing V3 source, not a standalone Nordic sample. The original Arduino code and V3 C file are preserved in `reference/`.

## Purpose

- **Legacy advertising:** find and rank all eligible wristbands with RSSI (50 ms base interval).
- **Periodic advertising:** subscribe to just one selected foreign wristband (50 ms periodic train; extended announcements 100 ms).
- **Nordic DM (NDT):** optional experimental ranging attempts to the selected wristband, synchronized via active-scan request/response.
- **Vibration:** for safety and reproducibility, keep the original RSSI thresholds and motor timing. NDT distance results are **diagnostic only** until calibration succeeds.

There is **one periodic synchronization** per physical wristband. NDT does not use periodic advertising for its timing synchronization. It uses BLE active-scanning `SCAN_REQ` / `SCAN_RSP` as in Nordic's official sample and MPSL radio timeslots for the measurement.

## The three available build modes

| Mode | Legacy RSSI | Periodic RSSI | NDT attempts | Intended use |
|---|---|---|---|---|
| `baseline` | yes | yes | no | compare against existing V3 logic |
| `ndt_focus` | yes | disabled at run time | yes | isolate NDT timing with two boards first |
| `ndt` | yes | yes | yes | intended combined experiment; most demanding on radio scheduler |

NDT experimental integration is intentionally not advertised as a validated ranging solution. A successful firmware build is only the first of several checks; genuine successful DM reports on at least two real boards are required.

## Hardware assumptions

- Seeed Studio XIAO nRF52840 (`xiao_ble/nrf52840`).
- Vibration motor controlled on D0 = P0.02 through an existing suitable motor **driver**. Do not directly drive the motor from the MCU GPIO.
- Player IDs 1 and 2 = same person; 3 and 4 = same person, etc.
- Player IDs must be unique and `NUM_WRISTBANDS` must be an even common number.
- Original ball subsystem is not present.

## Important design choices and limitations

1. The **original RSSI player selection, partner exclusion, classifications, Median(5)+EMA(0.35), 200 ms evaluation, 1500 ms timeout, motor pattern timings** remain unchanged.
2. The selected player is chosen **only from legacy RSSI**. Periodic RSSI feeds only its proximity classification, and DM measurements remain diagnostic. This is intentional; a metre-based vibration mapping needs separate validation.
3. DM uses a **fixed experimental seed** shared by both parties, derived with the initiator's identity address. This is suitable **only for the first 2-board bench experiment**. For 4–6 concurrent wristbands, replace with rotating per-peer synchronization seeds and collision control (see Nordic's `nrf_dm` sample).
4. The reflector accepts scan requests from known foreign wristbands; the initiator sends DM requests only to the selected target. A DM reflector attempt is not a periodic sync slot.
5. The scan-response timing method uses identity addresses in the NDT build. This broadcasts a stable BLE identity and is **not privacy-preserving**; use random/private addressing and robust peer pairing for any deployment.
6. A DM request only **schedules** radio ranging. It does not guarantee success. DM's 0–20 m numeric check is only for diagnostic log plausibility, not a validated operational range.
7. Hardware radio events, competing legacy/extended/periodic advertisements and NDT MPSL timeslots can contend. Test each mode; reduce load before using with people.
8. In the optional USB console variant, new USB stack settings can differ across NCS versions; check `START_HIER_DE.md`.

## Firmware organization

- `src/main.c` – original V3 logic plus `CONFIG_DM_MODULE`-guarded experiment.
- `prj.conf` – legacy + periodic BLE, SoftDevice Controller.
- `dm.conf` – optional Nordic DM + MPSL timeslot overlay.
- `usb.conf` + `boards/usb_cdc.overlay` – optional virtual COM port for logs.
- `boards/xiao_ble_nrf52840.overlay` – required NCS `ncs,dm-timer = &timer2;` devicetree choice.
- `scripts/build_player.ps1` – build per PlayerN in a west-enabled PowerShell.
- `scripts/flash_player.ps1` – show the resulting UF2 path (no risky automatic flashing).
- `reference/` – unmodified V3 C file, unmodified V2 Arduino reference, original source text.
- `tests/test_static.py` – simple invariant regression checks without toolchain.

## Functional test order

1. Build two firmwares for **Player1 and Player3** with `NUM_WRISTBANDS=4`.
2. Flash both in `ndt_focus` mode and monitor USB diagnostic messages.
3. Expect normal player selection and the original motor patterns. Look for **successful** `NDT PlayerN | distance=...` lines, not merely `NDT module initialized`.
4. Test with fixed 0.5, 1, 2, 3 m distances and orientation changes. Reject inconsistent/implausible distance estimates.
5. Rebuild both as `ndt` and compare how often real DM results arrive while periodic RSSI is also active.
6. Only then test Player1–Player4 and later six wristbands; experimental shared seed and RF scheduling require additional development for multiplayer.

## How to build

See **START_HIER_DE.md** for Windows instructions. For SDK-enabled `west` command-line environments:

```powershell
.\scripts\build_player.ps1 -Player 1 -Total 4 -Mode ndt_focus -UsbConsole
.\scripts\build_player.ps1 -Player 3 -Total 4 -Mode ndt_focus -UsbConsole
```

The project contains no built `.uf2` file: it must be compiled using the actual Nordic nRF Connect SDK toolchain.

## Source references

- Nordic experimental DM: https://nrfconnectdocs.nordicsemi.com/ncs/latest/nrf/samples/bluetooth/nrf_dm/README.html
- DM library: https://github.com/nrfconnect/sdk-nrf/blob/main/include/dm.h
- Board/UF2: https://docs.zephyrproject.org/latest/boards/seeed/xiao_ble/doc/index.html
- USB CDC: https://docs.zephyrproject.org/latest/snippets/cdc-acm-console/README.html
