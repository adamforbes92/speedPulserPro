# SpeedPulser Pro

The SpeedPulser Pro converts a vehicle speed signal — from a gearbox hall sensor, a [Can2Cluster](https://forbes-automotive.com) signal, a CAN bus (ECU / ABS / DSG / UDS), or a GPS module — into a 10 kHz PWM signal that drives a BLDC motor and gives life back to an OEM analog speedometer without a mechanical cable. It can also read engine RPM (hall or CAN) and re-output it as a traditional coil-style ignition signal for a tachometer. It is fully open-source, WiFi-configurable, and calibrated per cluster type so the needle reads accurately across the available speed range of the motor.

It is based on an **ESP32 DevKit V1 (WROOM-32)** and uses a **TY3816B** BLDC motor driven via a native LEDC hardware PWM channel. The PCB is an upgrade for the original SpeedPulser board, with the addition of CAN, GPS and a coil-type RPM output.

![SpeedPulser Pro Web UI](/Images/speedPulserProUI.png)

---

## Features at a Glance

| Feature | Detail |
|---|---|
| Speed input | Hall, ECU (CAN), ABS (CAN), DSG (TP2.0), UDS (CAN), GPS, Custom CAN |
| RPM input | Hall (12 V square wave) or CAN |
| Speedometer output | 10 kHz hardware PWM, 10-bit resolution |
| RPM output | Coil-style square wave (hardware-timer driven) |
| CAN | 500 kbit/s TWAI, full RX/TX, broadcast & SavvyCAN forwarding |
| GPS | u-blox NMEA, 1 / 5 / 10 / 16 Hz user-selectable update rate |
| Calibration profiles | 18 built-in (VW, Ford, Fiat, Merc, Smiths, Opel, VW Bay) |
| WiFi UI | Web app type interface |
| Needle sweep | Configurable on power-up (speed + RPM, linear or array-following) |
| Speed offset | Global fixed offset **or** 5-point speed-dependent curve |
| OTA updates | Firmware **and** filesystem upload from the browser |
| CAN analyzer | Forward live frames to SavvyCAN via WiFi (GVRET) or Serial |
| Power management | Auto WiFi-off + CPU scaling after 1 min idle |
| Remembers settings | All settings stored to ESP32 Preferences (NVS/EEPROM) |

---

## Supported Models

| Cluster | Range | Calibration by |
|---|---|---|
| VW MK1 / MK2 Golf | 120 mph | Martin Springell |
| VW MK1 / MK2 Golf | 120 mph | Forbes Automotive |
| VW MK1 / MK2 Golf | 120 mph | Tara |
| VW MK1 / MK2 Golf | 140 mph | Forbes Automotive |
| VW MK1 / MK2 Golf | 160 mph | Forbes Automotive |
| VW MK1 / MK2 Golf | 300 kph (1.8T) | Forbes Automotive |
| VW Bay VDO | 90 mph | Community |
| Ford Escort | 120 mph (var. 1) | Forbes Automotive |
| Ford Escort | 120 mph (var. 2 — Darren) | Community |
| FIAT Uno | 40–160 mph | Forbes Automotive |
| FIAT Uno | 20–110 mph | Forbes Automotive |
| Mercedes W123 | 120 mph | Forbes Automotive |
| Smiths 5/8" | 70 mph | Forbes Automotive |
| Smiths 5/8" | 90 mph | Forbes Automotive |
| Smiths 5/8" | 150 mph | Forbes Automotive |
| Opel Manta A-72 | 200 km/h | Forbes Automotive |

> Users are actively encouraged to submit new models, calibrations, or corrections via GitHub or Discord.

---

## Purchase

Pre-assembled SpeedPulser Pro units are available here: [SpeedPulser Pro — Forbes Automotive](https://forbes-automotive.com/products/speedpulserpro)

---

## More Detailed Instructions

The PDF Installation Guide on GitHub provides full step-by-step hardware fitting instructions.

---

## Hardware Overview

### PCB

The pre-assembled PCB carries three plug-in connectors along its edge plus a 4-pin GPS header:

- **Power / RPM Output** — 3-pin (silkscreen `12v GND MK2RPM`)
- **Motor** — 5-pin (silkscreen `MOTOR`)
- **Inputs / CAN** — 4-pin (silkscreen `RPM Speed CANH CANL`)
- **GPS** — 4-pin pin header (below ESP32)

---

### Power Supply & RPM Output Connector

The three-pin **`12v GND MK2RPM`** connector accepts 12 V battery power and provides the coil-style RPM tach output:

| Pin | Signal | Notes |
|-----|--------|-------|
| 1 | MK2 RPM Out | Closest to edge of PCB — high-voltage RPM output to the tach |
| 2 | GND | Common ground |
| 3 | PWR_IN (12 V) | Closest to the 5-pin motor connector; connect to ignition-switched 12 V |

> **Note:** The RPM output is intended for clusters expecting a coil-style square wave (e.g. VW MK2 tach input). Enable it from the **Configuration** tab via the *Use Coil Output (RPM)* checkbox.

An on-board adjustable **LM2596S** buck converter steps the 12 V supply down to approximately 9 V to power the motor, keeping the motor's operating range and torque within spec across the full speed scale.

---

### Motor Connector

The five-pin **`MOTOR`** connector on the PCB:

| Pin | Signal | Notes |
|-----|--------|-------|
| 1 | Motor Power | Black — 5–9 V (set via on-board trimmer) |
| 2 | Motor Feedback | Not used in current firmware |
| 3 | Motor Direction | Green — pull to GND to reverse needle direction |
| 4 | Motor Ground | White — motor ground return |
| 5 | Motor PWM | 10 kHz PWM from ESP32 (via NPN level-shifter to 5 V) |

---

### Inputs / CAN Connector

The four-pin **`RPM Speed CANH CANL`** connector carries both square-wave inputs and the CAN bus:

| Pin | Signal | Notes |
|-----|--------|-------|
| 1 | RPM In | 12 V square-wave RPM input (closest to the blue trimmer) |
| 2 | Speed In | 5 V or 12 V square-wave speed input (hall sensor or Can2Cluster) |
| 3 | CAN H | CAN bus high |
| 4 | CAN L | CAN bus low (closest to the 5-pin motor connector) |

> **Pull-up / Pull-down jumper:** Different hall sensors require either a pull-up or pull-down resistor. The 2-way jumper marked *SpeedPulser* selects this. If incoming pulses aren't being registered, swap the jumper position. Sensors with an internal resistor can have the jumper removed entirely.

> **CAN termination (`r-term`):** A 120 Ω terminating resistor for the CAN bus is selected by the `r-term` jumper. Leave it fitted if the SpeedPulser Pro is the only device on the bus. Remove it if the bus is already terminated elsewhere.

---

### GPS Connector

The four-pin GPS header accepts any 3.3 V u-blox-compatible NMEA module (e.g. NEO-6M / NEO-M8N):

| Pin | Signal | Notes |
|-----|--------|-------|
| 1 | 3.3 V | Powered from the ESP32 on-board 3.3 V rail |
| 2 | RX | GPS RX — connect to GPS module TX |
| 3 | TX | GPS TX — connect to GPS module RX |
| 4 | GND | Ground |

The GPS is driven via the ESP32 hardware UART for clean reception at high update rates.

---

### Coupler Installation

Motors are supplied with the coupler pre-fitted; the final drive pin and securing pins are included separately.

1. Remove the motor from the coupler housing.
2. Align the drive pin into the coupler recess and press home — a few light taps should seat it fully.
3. A spare coupler is supplied. Couplers are 3D printed; slight alignment variation is normal. A brief gentle heat application (≤ 100 °C) can encourage a true run.
4. If excessive vibration causes the coupler to loosen, a small drop of super-glue will fix it permanently — confirm it runs true before the glue sets.

---

### Trimming the Drive Shaft

Drive shafts are supplied longer than required. Trim to suit your cluster, typically flush with the motor housing. **Confirm the shaft does not bottom out in the cluster before tightening.**

---

### Fine-Tuning the Coupler

Once the motor is assembled, slide it over the OEM cluster shaft. Check:

- The drive pin engages cleanly
- The motor spins freely by hand
- The pin length is sufficient to engage the cluster but does not prevent the housing from seating fully

Take time here — good fitment minimises noise and extends coupler life.

---

## WiFi & Web Interface

Connect to the **`SpeedPulserPro`** WiFi access point and navigate to **`192.168.1.1`** in a browser.

The interface is a single-page app served from the ESP32's LittleFS flash partition. Settings are applied in real time and saved to EEPROM automatically every 5 seconds.

### Dashboard Tab

Live read-outs updated automatically:

| Field | Description |
|---|---|
| RPM (Final) | Engine RPM from the active RPM source |
| Speed (Final) | Vehicle speed from the active speed source |
| Speed Offset Type | Whether a *Global* or *Curve* offset is active |
| Current Speed Offset | The offset value applied at the current speed |
| CAN Bus | Live CAN healthy / not healthy indicator |
| GPS Status | GPS connected / no fix / not connected |

Header status badges also show **CAN**, **Broadcast** and the active **Calibration** profile.

### Configuration Tab

| Setting | Description |
|---|---|
| Enable Needle Sweep | Triggers a full-scale needle sweep on power-up (speed + RPM) |
| Linearise Speed Array | Sweep visits each speed linearly instead of following the calibration array |
| Sweep Speed (ms) | Step delay in milliseconds: lower = faster sweep |
| RPM Step / Speed Step | Sweep step size for RPM and speed |
| Test Needle Sweep | Trigger a sweep immediately from the browser |
| Use Coil Output (RPM) | Enable the coil-style RPM output on the `MK2RPM` pin |
| Cluster in MPH | Convert the km/h source value to mph before looking up the motor duty |
| Motor Calibration | Choose from 18 built-in cluster calibration profiles |
| Maximum Speed (km/h) | Upper end of the cluster's speed scale |
| Maximum Hall Frequency (Hz) | The input frequency that corresponds to Maximum Speed |
| Enable Global Speed Offset | Apply a single fixed offset across the whole range |
| Positive Offset | Direction of the fixed offset (add or subtract) |
| Global Speed Offset | Magnitude of the fixed offset |

### Advanced Tab

Combines live diagnostics, source selection and the CAN analyzer:

| Section | Purpose |
|---|---|
| Live Data – All Inputs | Independent read-out for every speed/RPM source (Hall, ECU, ABS, DSG, TP2.0, UDS, GPS, Custom CAN, plus filtered RPM) |
| Broadcast Speed | Re-emit the final speed onto CAN with user-defined ID, DLC, byte layout, scale & offset, and template bytes |
| GPS Update Rate | Select **1 / 5 / 10 / 16 Hz** (see below) and view the live updates-per-second figure |
| Signal Filters | Hall speed and RPM running-median sample counts (1–10) |
| CAN Analyzer | Forward all received CAN frames to **SavvyCAN** via WiFi (GVRET on `192.168.4.1:23`) or Serial (GVRET) |
| RPM / Speed Output Test | Drive the cluster directly at a chosen RPM or speed |
| Speed Selection | Primary speed source: Hall / ECU / ABS / DSG / TP2.0 / UDS / GPS / Custom CAN |
| Custom CAN Input | When *Custom CAN* is selected, define ID, byte indices, endianness, scale & offset |
| RPM Selection | Primary RPM source (Hall or CAN), cluster frequency limit and RPM ceiling |
| Calibration | Direct duty stepping (±1) for on-the-fly motor calibration |

### Calibration Tab

Contains the **5-Point Speed Offset Curve**.

Allows a different trim offset to be applied per speed band instead of a single global value. Bands are fixed at 0–50 / 51–100 / 101–150 / 151–200 / 201+ km/h, each accepting ±20 km/h. Enable the checkbox to activate the curve in place of the global offset.

### OTA Tab

Two upload slots:

- **Firmware** — upload a new compiled `.bin` to flash the application.
- **Filesystem** — upload a new LittleFS `.bin` to replace the web UI assets.

The device reboots automatically after a successful upload.

---

## GPS & Update Rates

The GPS module is driven via the ESP32 hardware UART for reliable reception. SpeedPulser Pro supports user-selectable update rates of **1 Hz, 5 Hz, 10 Hz and 16 Hz**, sent to the module as u-blox UBX rate commands. The selected rate is stored in EEPROM and survives reboots.

### How It Works

1. On boot, the firmware probes the GPS at 9600 baud and 38400 baud and locks onto whichever yields valid checksum-passing NMEA. The u-blox module always boots at 9600, so a re-flash without power-cycling the GPS is handled cleanly.
2. The GPS is left at its boot baud (9600) until satellite lock is acquired.
3. Once satellites are stable for ~20 seconds, the firmware automatically applies the stored update rate. If a rate of 5 Hz or higher is requested, it first issues a `PUBX,41` command to step the module up to 38400 baud (1 Hz uses 9600).
4. The user can also force the rate at any time from the **GPS Update Rate** card on the Advanced tab; the live *GPS Updates/sec* counter confirms what's actually arriving.

### Choosing a Rate

| Rate | When to use |
|---|---|
| 1 Hz | Default; lowest UART load; suitable for casual / cruise display |
| 5 Hz | Smoother needle response in normal driving |
| 10 Hz | Track / spirited driving; very responsive |
| 16 Hz | Maximum responsiveness; some u-blox modules require 38400 baud |

> The update-rate change is blocking on the UART, so the main loop briefly suspends background tasks while the PUBX command is sent — this prevents the CAN RX, speed ISR and AsyncTCP tasks from corrupting the outgoing serial bytes.

---

## Power Management

The firmware includes a **universal reduced-power codeblock** (`power_manager` — also used in SpeedPulser and Can2Cluster) that activates automatically 1 minute after the last WiFi client disconnects. This cuts current through the on-board linear regulator, directly reducing its heat output — important for long ignition-on times.

**What changes when idle:**

| Action | Saving |
|---|---|
| WiFi radio off | ~80–120 mA average (single biggest saving) |
| CPU: 240 MHz → 80 MHz | Moderate reduction in active current |
| Bluetooth controller released at boot | ~60 KB RAM freed; small idle current saving |
| WiFi modem-sleep while clients are connected | Minor saving without losing connectivity |
| Reduced WiFi TX power | Adequate for in-car range; further small saving |
| Onboard LED off at boot | Tiny but persistent saving |

**Waking back up:**  
As soon as a device reconnects to the WiFi AP, full power is restored automatically — the radio comes back up, the CPU returns to 240 MHz, and the web server resumes. A power-cycle (ignition off/on) will also restore WiFi.

> The ESP32 DevKit V1 (WROOM-32) runs at 240 MHz; the power manager auto-detects this at compile time and adjusts accordingly. On ESP32-C3 / S2 variants the active clock caps at 160 MHz.

---

## Calibration — How It Works

### The Calibration Array

Each calibration profile is a **386-element `uint16_t` array** stored in flash (`PROGMEM`). The **array index** represents speed in km/h (0–385) and the **array value** is the 10-bit PWM duty cycle (0–1023) that drives the motor to produce the corresponding reading on that cluster.

```
index   0  →  duty   0   (motor off / dead band)
index  50  →  duty  ~60  (motor at ~50 km/h cluster reading)
index 120  →  duty ~130  (motor at ~120 km/h cluster reading)
  ...
index 200  →  duty ~195  (motor at full scale for a 200 km/h cluster)
```

Values near index 0 are `0` — the motor's dead band where it will not yet turn. Values plateau near the top because the cluster needle is at full deflection.

### Signal Processing 

```
Source (Hall / ECU / ABS / DSG / TP2.0 / UDS / GPS / Custom CAN)
    │
    ▼  ISR / TWAI RX / GPS parser
    Per-source speed value
    │
    ▼  speedControlTask
    For Hall: map(frequency, 0, maxFreqHall, 0, maxSpeed) → km/h
    For other sources: speed already in km/h
    │
    ▼  RunningMedian filter  (averageFilterHall samples, default 6)
    Smoothed median speed value
    │
    ▼  applyConfiguredSpeedOffset()
    Speed ± global offset  OR  speed ± curve offset for that band
    │
    ▼  Optional: × 0.621371  if "Cluster in MPH" is enabled
    │
    ▼  findClosestMatch()
    Scans motorPerformance[] for the closest matching speed value
    Returns the array index = 10-bit duty cycle
    │
    ▼  setMotorDuty()  — native LEDC IDF driver, 10 kHz
    Motor PWM output on GPIO 21
```

> **Default hall-sensor scaling:** 1 Hz = 1 km/h. This matches 02J / 02M gearbox sensors used in most VW/Audi applications. Adjust `maxFreqHall` and `maxSpeed` together if your sensor has a different ratio (e.g. set both to 160 for a sensor that outputs 160 Hz at 160 km/h).

### How to Calibrate a New Cluster (Step by Step)

1. **Bench setup** — Power the SpeedPulser Pro from 12 V. Fit the motor to the cluster.
2. **Connect to WiFi** — Join the `SpeedPulserPro` AP and open `192.168.1.1`.
3. On the **Advanced** tab, enable **Enable Calibration**.
4. Press **Duty -1%** — this rolls the duty counter to the maximum. Use the on-board potentiometer to set the cluster needle to its maximum speed.
5. Press **Duty +1%** — this rolls the duty counter back to zero.
6. Press **Duty +1%** and increase the duty one step at a time, noting the resulting needle position.
7. Build the 386-element array in `src/SpeedPulserPro_motorCal.cpp`:
   - Each **index** is the km/h speed (0–385).
   - Each **value** is the required duty recorded in steps 5–6.
   - Linearly interpolate between measured points for unmeasured indices.
   - Leave leading entries as `0` until the motor reliably starts turning.
8. Add the array to `calibrationProfiles[]` with a descriptive name and rebuild.
9. Flash and verify the full range on the cluster.
10. **Please share the calibration** — open a pull request or post on Discord so others with the same cluster benefit.

> Couplers are 3D printed and every motor / housing / cluster pairing wears in slightly differently. Expect to revisit the on-board potentiometer once after a few hours of running so the top-end reading still hits full-scale cleanly.

### Reading / Parsing a Calibration Array

To check what duty a profile produces at a given speed, index directly:

```cpp
// Direct lookup — returns the 10-bit duty for 80 km/h on the active profile
uint16_t duty = motorPerformance[80];
```

`findClosestMatch()` does the inverse: given a target speed value, it walks the array to find the entry whose **value** is closest to the target and returns the **index** as the duty cycle. This gracefully handles calibration arrays that are not perfectly monotone.

---

## Speed Offset

Two offset modes are available and are configured from the **Configuration** and **Calibration** tabs respectively.

**Global Offset** (default)  
A single fixed value is added to or subtracted from every speed reading before the calibration lookup. Useful for correcting a systematic bias across the whole scale caused by motor preload or cluster wear.

**5-Point Curve**  
Five independent offsets replace the global offset when enabled. This corrects clusters whose response is non-linear and cannot be fixed by a single constant.

| Point | Speed band |
|---|---|
| 1 | 0 – 50 km/h |
| 2 | 51 – 100 km/h |
| 3 | 101 – 150 km/h |
| 4 | 151 – 200 km/h |
| 5 | 201+ km/h |

Each point accepts ±20 km/h. Enable the *Speed-Dependent Offset Curve* checkbox on the Calibration tab to activate.

---

## CAN Bus

The SpeedPulser Pro runs the ESP32 TWAI controller at **500 kbit/s** with all-pass filtering, so any frame on the bus is available to the firmware.

### Built-in Decoders

| Source | Frames used | Output |
|---|---|---|
| ECU (VAG) | `MOTOR1` engine RPM, `MOTOR2` speed, `MOTOR5/6` EML/EPC/park | `vehicleRPMCAN`, `ecuSpeed` |
| ABS (VAG) | `BRAKES3` wheel-derived speed | `absSpeed` |
| DSG / Gear lever | `mWaehlhebel_1`, `gearLever` | Selected gear, reverse flag |
| DSG (TP2.0) | Channel setup → speed PID | `tp20Speed` |
| Generic | UDS read (0x22) speed PID | `udsSpeed` |
| Custom CAN | User-defined ID + byte layout | `vehicleSpeedCAN` |

Pick the source from the **Speed Selection** drop-down on the Advanced tab.

### Broadcasting Speed

The final processed speed (after offset / MPH conversion) can be re-emitted onto CAN for downstream devices (e.g. cluster mods, head units). Configure the CAN ID, DLC, low/high byte indices, endianness, scale, offset and any template bytes from the **Broadcast Speed** card.

### SavvyCAN Forwarding

Every received frame can be forwarded to **SavvyCAN** for live capture:

- **WiFi mode** — GVRET protocol on TCP port `192.168.4.1:23`.
- **Serial mode** — GVRET protocol on USB-CDC.

The two modes are mutually exclusive and can be toggled from the CAN Analyzer card.

---

## Over-the-Air Updates

New firmware (and the web UI) can be flashed without removing the unit from the vehicle:

1. Build the project in PlatformIO → locate the `.bin` files in `.pio/build/esp32doit-devkit-v1/`.
2. Connect to the `SpeedPulserPro` WiFi AP.
3. Open the **OTA** tab in the browser.
4. To update the application, select `firmware.bin` and click **Upload Firmware**.
5. To update the web UI, build the filesystem image and upload `littlefs.bin` via **Upload Filesystem**.
6. The device flashes and reboots automatically.

---

## Technical Reference

### Pin Assignments (ESP32 DevKit V1)

| GPIO | Function |
|---|---|
| 2  | Onboard LED |
| 13 | GPS TX (ESP → module RX) |
| 14 | GPS RX (ESP ← module TX) |
| 16 | TWAI / CAN TX |
| 17 | TWAI / CAN RX |
| 18 | RPM hall input (falling-edge interrupt) |
| 19 | Motor direction (reserved for future use) |
| 21 | Motor PWM output (LEDC, stepped to 5 V via NPN transistor) |
| 22 | Coil-style RPM output (hardware timer) |
| 26 | Speed hall input (falling-edge interrupt) |

### PWM Parameters

| Parameter | Value |
|---|---|
| Frequency | 10 kHz |
| Resolution | 10-bit (0–1023) |
| Driver | Native ESP-IDF `ledc_set_duty` / `ledc_update_duty` |

### CAN / TWAI

| Parameter | Value |
|---|---|
| Baud | 500 kbit/s |
| Mode | Normal, accept-all filter |
| RX queue | 256 frames |
| TX queue | 16 frames |

### FreeRTOS Tasks

| Task | Purpose |
|---|---|
| `taskCANRx` | TWAI receive + decoder + SavvyCAN forwarder |
| `taskWriteEEP` | Persist Preferences (5 000 ms) |
| `taskUpdateUI` | Push live values to the web UI (200 ms) |
| `taskParseGPS` | Continuous TinyGPS++ parse |
| `taskParseDSG` | DSG / gear ratio decode |
| `taskProcessSpeed` | Hall averaging, source selection, offset, duty lookup |
| `taskProcessRPM` | RPM source selection and output drive |
| `taskBroadcastSpeed` | Periodic CAN broadcast of final speed |
| `taskTP20` / `taskUDS` | Transport-protocol speed pollers |

### PlatformIO Dependencies

| Library | Purpose |
|---|---|
| `mathieucarbou/ESPAsyncWebServer` | Async web server (ESP-IDF 5.x compatible fork) |
| `mathieucarbou/AsyncTCP` | Underlying TCP for AsyncWebServer |
| `bblanchon/ArduinoJson` | JSON serialisation for REST API |
| `RobTillaart/RunningMedian` | Median filter for speed/RPM smoothing |
| `mikalhart/TinyGPSPlus` | NMEA parsing |
| `plerup/espsoftwareserial` | Legacy software-UART fallback |

Platform: `pioarduino/platform-espressif32` 54.03.20 (Arduino-ESP32 3.x / ESP-IDF 5.x)

---

## Building & Flashing

1. Open the project folder in VS Code with the PlatformIO extension installed.
2. Confirm `env:esp32doit-devkit-v1` is selected in `platformio.ini`.
3. **Build Filesystem Image** (PlatformIO sidebar) — packages the web UI files from `data/` into a LittleFS image.
4. **Upload Filesystem Image** — flashes the web UI to the LittleFS partition.
5. **Build & Upload** — flashes the main firmware.
6. Open the Serial Monitor at 115 200 baud to confirm startup messages.

> Serial debug output is controlled by the `serialDebug*` flags in `include/SpeedPulserPro_config.h`. Set all to `0` for production builds.

---

## Disclaimer

The SpeedPulser Pro drives an analog speedometer for display purposes. It should always be assumed that the reading is approximate — Forbes Automotive accepts no responsibility for any speed-related incident arising from its use.

---

## Version History

| Version | Summary |
|---|---|
| V1.01 | Initial release — used SpeedPulser and Can2Cluster as a base |
| V1.02 | Confirmed WiFi and RPM working |
| V1.03 | Board revision — EasyEDA, ground planes, separate Speed / RPM pull-to-ground inputs, ECU RPM repurposed pin |
| V1.04 | "Cannot find" speed → set to zero; low-speed fixes |
| V1.05 | Hall-type RPM in WiFi UI; on-the-fly calibration changes; read EEP at startup |
| V1.06 | New calibrations; removed `setTxPower` (caused WiFi drop on speed change) |
| V1.07 | Speed-input selection in WiFi; UI tidy-up |
| V1.08 | Calibration page |
| V2.00 | PlatformIO port; UDS support for DSG speed reading |
| V2.01 | GPS speed reading; CAN speed broadcasting; configurable & persistent GPS update rate |
| V2.10 | Linearised needle sweep; Tara 120 mph calibration |
| V2.20 | SavvyCAN analyzer; "Cluster in MPH" conversion option |
| V2.30 | RPM / speed task pacing tuned so GPS has room to update |
| V2.40 | GPS moved to hardware `HardwareSerial` UART for faster, cleaner reception |
| V2.50 | Universal power-management module (WiFi-off + CPU scaling after 1 min idle); GPS auto-baud probe on boot; GPS update-rate auto-apply after satellite stability |
