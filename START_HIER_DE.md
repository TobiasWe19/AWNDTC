# Start hier – XIAO nRF52840 mit unserem V4-Code (Windows / VS Code)

## 1. Benötigte Programme

1. Installiere **Visual Studio Code**: https://code.visualstudio.com/
2. In VS Code: **Extensions** öffnen (`Ctrl+Shift+X`) → **nRF Connect for VS Code** (Nordic Semiconductor) installieren.
3. In der nRF-Connect-Erweiterung eine **stabile nRF Connect SDK-Version** samt passender Toolchain installieren. Verwende eine Version, die das Board `xiao_ble/nrf52840` und die Nordic DM Library enthält (zum Beispiel SDK 3.x; genaue Version bitte mit dem ersten Build überprüfen).
4. Öffne das ZIP und darin den Ordner `Assistive_Wristband_NDT_V4_Experimental` als Projektordner.

**Wichtig:** Eine Arduino-Boardinstallation ersetzt das Nordic SDK nicht. Unsere `.ino`-Referenz bleibt unangetastet.

## 2. SDK-Terminal öffnen und erste Tests bauen

In VS Code den **nRF Connect SDK-Terminal** öffnen, damit `west` verfügbar ist.
Im Projektordner in PowerShell:

```powershell
.\scripts\build_player.ps1 -Player 1 -Total 4 -Mode ndt_focus -UsbConsole
.\scripts\build_player.ps1 -Player 3 -Total 4 -Mode ndt_focus -UsbConsole
```

Player1 und Player3 gehören zu **verschiedenen Personen**. Player1/Player2 sind derselbe Spieler und erkennen sich absichtlich nicht gegenseitig.

Wenn die DM-Konfiguration Probleme macht, zunächst die Funk-/Vibrationsbasis prüfen:

```powershell
.\scripts\build_player.ps1 -Player 1 -Total 4 -Mode baseline -UsbConsole
.\scripts\build_player.ps1 -Player 3 -Total 4 -Mode baseline -UsbConsole
```

Wenn beide grundsätzlich funktionieren, die Kombination ausprobieren:

```powershell
.\scripts\build_player.ps1 -Player 1 -Total 4 -Mode ndt -UsbConsole
.\scripts\build_player.ps1 -Player 3 -Total 4 -Mode ndt -UsbConsole
```

Build ohne optionalen USB-Console-Zusatz: einfach `-UsbConsole` weglassen. Dann ist `printk` am USB-Port unter Umständen nicht sichtbar. Der Motor kann trotzdem funktionieren.

## 3. Firmware aufspielen

1. Kompiliertes Abbild liegt z. B. unter `build/player1_ndt_focus/zephyr/zephyr.uf2` (nur wenn der Build erfolgreich war).
2. Den XIAO nRF52840 per USB anschließen.
3. **RESET zweimal kurz drücken**, um den UF2-Bootloader zu starten. Das Laufwerk `XIAO BLE` erscheint (bei intaktem vorhandenen Bootloader).
4. `zephyr.uf2` auf dieses Laufwerk kopieren. Das Gerät startet die Firmware anschließend automatisch.
5. **Nicht** Player1 und Player3 versehentlich auf dasselbe Gerät flashen.
6. Nach dem Neustart den COM-Port erneut prüfen; er ist möglicherweise ein anderer als im Arduino-Bootloader.

Falls kein `zephyr.uf2` erzeugt wird, keine `.bin`/`.hex` blind auf das USB-Laufwerk kopieren. Zuerst die Zephyr-Build- und Runner-Konfiguration prüfen. Bei beschädigtem Bootloader kann ein SWD/J-Link nötig sein.

## 4. Was du auf dem COM-Port sehen solltest

Öffne z. B. nRF Terminal oder einen seriellen Monitor (COM-Port auswählen). Falls `-UsbConsole` im konkreten SDK nicht funktioniert, kann man auf eine passende Zephyr CDC-ACM-Konfiguration oder RTT mit einem SWD-Debugger ausweichen.

Typische **Beispielmeldungen**, keine garantiert beobachteten Messergebnisse:

```text
Assistive Wristband V4 | Player1 | Person1 | total=4
NDT module initialized (diagnostics only)
Legacy Advertising: 50 ms
Periodic Advertising disabled for isolated NDT test
TARGET Player3 | LEGACY RSSI=-67 dBm | MEDIUM | ...
NDT Player3 | distance=1250 mm | quality=0 (experimental)
```

Die NDT-Zeile wird **nur ausgegeben, wenn wirklich ein verwertbares Resultat kommt**. Fehlt diese Zeile, läuft die normale RSSI-Vibration trotzdem weiter.

## 5. Testreihenfolge

- **Zuerst zwei einzelne Bänder:** Player1 ↔ Player3, 1 m Abstand, keine Personen dazwischen.
- `TARGET Player...` beweist, dass Legacy den anderen Player findet.
- `Attempting periodic sync ...` und `PERIODIC RSSI=...` beweisen Periodic-Empfang (nur im Modus `ndt` oder `baseline`).
- `NDT Player... distance=...` beweist nur, dass ein DM-Messergebnis angekommen ist – die Genauigkeit ist damit noch nicht validiert.
- Abstand systematisch auf 0,5 / 1 / 2 / 3 m ändern und Ergebnisse protokollieren.
- Körperabschirmung, Bewegung und weitere Spieler **erst danach** testen.

## 6. Was genau bleibt aus dem Original erhalten?

- Namensschema `PlayerN` und dieselben ID-Paarregeln (1+2, 3+4 usw.).
- Filter der Eigen- und Partnerarmbänder.
- Median aus 5 RSSI-Messwerten + EMA α=0,35.
- RSSI-Auswertung alle 200 ms.
- Ursprüngliche RSSI-Stufen (−50, −65, −70, −75, −90 dBm).
- Ursprüngliche fünf Motor-Pulssequenzen und `NO_SIGNAL` = AUS.
- Ursprünglicher Motorfehlercode: 100 ms ein / 900 ms aus.
- Legacy-Fallback bei fehlerhafter Periodic-Synchronisierung.
- **NDT steuert noch keine Vibration** – hierfür brauchen wir eine empirisch kalibrierte Umrechnung von Distanz zu fünf Stufen.

## 7. Bekannte Entwicklungsrisiken

- Der Nordic-DM-Ansatz ist experimentell und **nicht** auf eurem XIAO geprüft.
- Die aktive Scan-Anfrage und Antwort müssen zeitlich zusammenpassen. Das gelingt nicht allein durch Setzen eines Advertising-Intervalls.
- Der Test nutzt eine konstante Seed-Ableitung und ist zunächst für **zwei Bänder** vorgesehen. Für mehrere gleichzeitig gemessene Fremdgeräte ist zusätzliche Konfliktbehandlung nötig.
- Wenn `DM_MODULE` im SDK Build-Abhängigkeiten meldet, prüfe den ersten Kconfig-Fehler, **nicht** wahllos Optionen aktivieren.
- USB-Konsole und Zephyr-USB-Stack unterscheiden sich mit der SDK-Version. Ein USB-Problem ist nicht zwingend ein BLE-Problem.
- TX-Leistung von +4 dBm und 80 % Scan Duty Cycle erhöhen ggf. die Leistungsaufnahme.
- Kein Hardwareversuch an sehbehinderten Sportlern bevor Zuverlässigkeit und Fail-safe-Verhalten geprüft wurden.

## 8. Offizielle technische Grundlagen

Nordic DM Sample: https://nrfconnectdocs.nordicsemi.com/ncs/latest/nrf/samples/bluetooth/nrf_dm/README.html

Zephyr XIAO Board + UF2: https://docs.zephyrproject.org/latest/boards/seeed/xiao_ble/doc/index.html

Nordic DM Library Integration: https://github.com/nrfconnect/sdk-nrf/blob/main/doc/nrf/libraries/others/dm.rst
