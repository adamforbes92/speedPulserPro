#ifndef VERSION_H
#define VERSION_H

/*
Inputs are a 5v/12v square wave input from Can2Cluster or an OEM Hall Sensor and converts it into a PWM signal for a motor.
To get speeds low enough, the motor voltage needs reduced (hence the adjustable LM2596S on the PCB)
from 12v to ~9v. This allows <10mph readings while still allowing high (160mph) readings.
Clusters supported are 1540 (rotations per mile) =~ (1540*160)/60 = 4100rpm

Default support is for 12v hall sensors from 02J / 02M etc.
According to VW documentation, 1Hz = 1km/h. Other marques may have different calibrations (adjustable in 'config.h')

Motor performance plotted with Duty Cycle & Resulting Speed.
Basic Excel located in GitHub for reference - the motor isn't linear so it cannot be assumed that x*y duty = z speed(!)
LEDc PWM can use various 'bits' for resolution.
8 bit results in a poorer resolution, therefore the speed can be 'jumpy'.
Default is 10 bit which makes it smoother. Both are available.

Uses 'RunningMedian' for capturing multiple input pulses to compare against. Used to ignore 'outliers'

All main adjustable variables are in 'config.h'.

V1.01 - initial release - using SpeedPulser & C2C as a base
V1.02 - full release - confirmed WiFi and RPM working. To test CAN
V1.03 - board revision, moved to EasyEDA and added ground planes. Signal more stable. Changed motor feedback (which isn't being used) to ECU RPM
  > inputs are CAN or Speed & RPM (individual pull-to-ground)
  > outputs are RPM & Speed (via. motor)
  > added WiFi info - has GPS & has CAN confirmation
V1.04 - added 'cannot find' speed to set to zero and low speed issues
V1.05 - added 'hall type' RPM into WiFi and added 'on-the-fly' calibration changes. Also added 'read EEP' at the start, oops!
V1.06 - added new calibrations. Removed 'setTxPower' - seemed to cause WiFi failure on speed change(!)
V1.07 - added selections for speed input and tidied up WiFi
V1.08 - added calibration page

V2.00 - added UDS support for DSG speed reading.  PlatformIO port
V2.01 - added GPS support for speed reading. Added CAN speed broadcasting. Added GPS update rate configuration and persistence.

V2.10 - added Linearise Speed for Needle Sweep & Tara 120mph calibration
V2.20 - added SavvyCAN and Cluster in MPH
V2.30 - changed rpm/speed update rate to ensure GPS has a chance to update
V2.40 - moved GPS to HardwareSerial to improve response

V3.00 - closed-loop motor feedback (PID): measures the motor feedback and trims PWM duty so the needle holds under load.
V3.01 - added a live calibration-curve graph on the Dashboard (duty-vs-speed trace + a marker for the point currently achieved from the hall input, Speed Test or Calibration Mode).
      - graph now zooms to the duty range actually used, labels the axis and marker in raw duty (matching the Motor Duty gauge), and redraws when the calibration is changed.
      - Dashboard layout mirrors the standard SpeedPulser (Motor Duty, Measured Speed and PID Trim gauges) so both share common cards and diagnostics.
      - confirmed Calibration duty rolls over or under
V3.02 - Sweep Speed (ms) is now a live slider (0-50), matching SpeedPulser; dragging it retimes an in-progress sweep smoothly.
      - rewrote the needle sweep: it now drives BOTH needles to full mechanical deflection and back over a bounded duration. Previously the speed needle only reached the calibration ceiling (~37% of the 12-bit range) and a large Max RPM made the sweep run for minutes; it now ramps the full 12-bit PWM range and the full RPM output. The two needles (BLDC motor vs air-core tach coil) ramp at INDEPENDENT rates set by the Speed Ramp Rate / RPM Ramp Rate sliders, so the slower tach needle can be ramped gently enough to physically reach full scale.
      - removed the redundant Linearise Speed Array control (superseded by the new sweep); the per-needle ramp-rate controls are retained.
      - shared, project-agnostic OTA module (ota_manager, used like power_manager): POST /api/ota-update?mode=firmware|filesystem + GET /api/version; OTA tab reworked to match SpeedPulser (Firmware Info + single OTA Update card).
      - GPS status is now three-state: Connected (n sats) / Not Connected / Not Available (no module fitted).
      - moved Speed Test Mode onto the Diagnostics tab.

V3.03 - custom calibrations now remember the cluster unit they were captured in and the device auto-enables "Cluster in MPH" whenever an MPH cal becomes the active calibration (boot, dropdown selection, apply/save/import). /api/cal now returns convertToMPH so the UI mirrors it, and a "Cluster in MPH" toggle was added directly on the Calibration Builder page (kept in lock-step with the Configuration page toggle).
V3.04 - closed-loop feedback is now presence-aware and safer on legacy PCBs without the motor-feedback tacho:
      - the tacho input is monitored so Measured Speed / PID Trim now have three states: "--" (not seen yet, motor idle), a live value (feedback present), or "N/A" (motor running >1.5s with no feedback signal, e.g. legacy PCB). The PID loop only engages once a feedback signal is actually detected, so it stays safely open-loop (feed-forward) otherwise.
      - the gearbox hall input is now ignored while Speed Test or Calibration Mode is active, so an incoming hall signal can no longer disturb a test/cal (gated in the ISR).
      - PID steady-state accuracy improved: the deadband now only silences the P/D terms; the integral keeps trimming inside the band so the needle settles ON target instead of a fixed offset.
      - new user-configurable "PID Deadband (Hz)" slider (0-5 Hz, persisted) exposes that band; 0 = always full PID.

V4.00 - voltage control + VR speed input (ported from SpeedPulser V4):
      - V4 PCB adds an adjustable buck motor supply. A board-version strap (GPIO25, LOW = V4) auto-detects the new board so the same firmware still runs unchanged on legacy PWM-only boards. GPIO32 = buck enable, GPIO33 = V_ADJ PWM "DAC" (inverse-injected into the buck FB node, LEDC ch2/timer2).
      - mid-ranging control law: a fast inner PID drives the throttle PWM to hit the target motor RPM (one-point tacho cal) while a slow integrator trims the buck voltage to keep the PWM near a nominal centre, so the motor rail self-schedules to the motor's non-linear curve with no multi-point calibration. Tunables (nominal/min PWM, min/max volts, Kv, Kp/Ki/Kd) are exposed and persisted.
      - new variable-reluctance (VR) speed input on GPIO27 (VR conditioner C Out, external pull-up), counted on the FALLING edge exactly like the hall input with its own frequency->speed mapping (Max VR Freq), median filter (VR average) and live data. Selectable from the speed-source dropdown ("VR").
      - NOTE: the (currently unused) coolant X9C103 pins were relocated off GPIO25/27 to GPIO4/5 to make room for the board-version strap and VR input.

V4.01 - standardised Forbes Automotive UI theme (shared style.css);
        adopted common wifi_manager (mDNS: can2rpm.local) and ota_manager
        (firmware + filesystem OTA via /api/ota, /api/ota/fs); per-product
        cache-busting on web assets
V4.02 - OTA overhaul (shared ota_manager / wifi_manager v2 + data/ota.js, ported
        from OpenHaldex 9.00): upload callbacks no longer answer mid-body (the
        old per-chunk "200 OK" made the browser drop the connection after the
        first 1.4 kB - a crash in AsyncTCP and a half-written partition, so no
        OTA through the UI had ever completed); filesystem updates unmount
        first, check the announced size, verify the mount and wipe on failure;
        boot only mounts a sane superblock and the web server always starts -
        with no usable UI "/" is a recovery page with the two uploads.
        "Update from GitHub" on the OTA tab (Releases/releases.json via
        tools/make_release.py) plus a Home WiFi (bridge mode) card; power_manager
        holds WiFi up while any browser is active. Assets served no-cache (ETag)
        instead of the hand-bumped ?v=.

V5.00 - ESP32-S3 board (PCBs/rev6-allConfirmed) on the V4.02 base. (Supersedes the
        V4.03 "S3 Board" folder, which branched before the V4.02 DevKit GPTimer RPM
        fix; that fix is kept and the S3 now uses the same GPTimer RPM output.)
      - ONE codebase, two PlatformIO envs: esp32doit-devkit-v1 (-DSPP_BOARD_DEVKIT)
        and esp32-s3 (-DSPP_BOARD_S3, ESP32-S3-WROOM-1-N16R8, 16 MB, native USB).
        Board macros + SPP_HAS_* feature flags in SpeedPulserPro_board.h, per-board
        pin maps in SpeedPulserPro_config.h. The PCB defines what a board has:
        S3 = voltage injection + rail sense + VR + switched speed bias + CAN standby;
        DevKit = PWM motor drive only, and carries NO code/settings/UI for any of
        those (the old DevKit GPIO25 strap / GPIO32-33 buck / GPIO27 VR are gone).
      - S3 pin map confirmed pad-by-pad against the rev6 netlist: motor IO10, coil
        RPM IO9, RPM in IO12, hall in IO16, feedback IO8, dir IO11, CAN RX/TX
        IO13/IO14, CAN STB IO21, GPS IO17/IO18, V_MOT_SENSE IO4, EN_ADJ IO5,
        V_ADJ IO6, BOARD_ID IO7 (sanity check only), VR IO15, speed bias IO47/IO48,
        LED IO38.
      - S3 voltage control, ported from SpeedPulser V5.00 (identical injection /
        sense network): rail commands in volts across 3.0-12.0 V (12 V ceiling;
        voltage control off holds the old fixed 9 V), rail read-back every 100 ms,
        and "Test Board" on the Diagnostics page (steps 5 / 7.5 / 10 V with the
        motor stopped, reads each back, stores pass/fail + this board's own
        duty->volts fit).
      - S3 one-point Top Speed Calibration: Calibration Mode holds the PWM at
        Nominal Throttle PWM (default now 80 %), the user raises the rail
        (-/+ volt buttons) until the needle sits on a top mark, enters the speed
        shown and captures. Tacho Hz averaged over ~1 s and refused while still
        settling; stored as (speed, Hz, rail, PWM) in cluster units, linear to zero,
        and the captured rail seeds the voltage feed-forward. The legacy
        Calibration Builder is only shown when there is no voltage injection.
      - from V4.03: per-board OTA images (releases.json "boards", make_release.py
        --env, ota.js picks this board's images), switched speed-input bias UI,
        CAN standby held LOW, S3 LEDC low-speed mode, power_manager on the S3 LED.

V5.01 - drive calibration: the S3 voltage control + calibration process copied across from
        SpeedPulser V5.01 (same injection / sense network, same wording in the UI).
      - one DRIVE value (V = rail x PWM) commands the motor. Above Minimum Rail x Nominal PWM
        the PWM holds at Nominal and only the rail moves; below it the rail holds at Minimum
        Rail (fixed 3.0 V, the hardware floor) and the PWM falls to zero, so the slowest
        speeds are reachable.
      - multi-mark calibration replaces the single top point: in Calibration Mode set the drive
        until the needle sits on a mark, capture it; repeat for a low mark, the top and a few
        between (up to 10). Each mark stores speed, tacho Hz, drive, rail and PWM. Running
        interpolates the drive between marks (open loop), and Measured Speed maps tacho Hz back
        through the same marks. A V5.00 top point is carried over as the first mark.
      - the mid-ranging law (fast PWM PID + slow voltage loop) is gone. Feedback is now one slow
        trim on the calibrated drive, only with Feedback Enable on, above Min Feedback Speed and
        outside the PID Deadband, capped at ±Feedback Trim Limit (default 20 %).
      - running speed is capped at the top calibration mark (no extrapolated drive past it).
      - capture "Steady" tests DRIFT (newer half-second mean vs older, <= 2 % or 0.75 Hz), not
        spread, so low-speed cogging no longer blocks a capture; spread only has to stay under
        20 % / 4 Hz.
      - tacho and rail are measured every 100 ms in every mode (one tick in the speed task).
      - Voltage Control settings are now Nominal PWM, Maximum Rail (V), Feedback Trim Gain and
        Limit; Min/Max Voltage, Minimum PWM and the inner Kp/Ki/Kd are gone.
      - needle sweep with Voltage Control on drives up to the calibrated Max Speed by drive.
      - voltage calibration export/import ("type": speedpulser-voltage-cal, shareable with
        SpeedPulser); PWM cal files are tagged speedpulser-pwm-cal; one Import button loads
        either. Cal POST bodies are collected across TCP chunks.
      - "Cal:" banner reads "Voltage (n marks, lo-hi)" when Voltage Control drives the motor,
        and the preset selector notes that the PWM preset is idle.
      - own build folder (speedpulserpro-v501-pio) so the V5.00 build stays intact.
      - Reverse Direction inverted to mirror SpeedPulser V5.01: unticked (IO11 LOW) is now the
        correct way. Stored under a new EEP key ("revDir"), so units that had Reverse ticked to
        compensate come up unticked = correct.
      - web UI: /api/status is only requested once the previous answer is back (as SpeedPulser);
        the 200 ms timer used to stack requests until the async server ran out of sockets.
      - Speed Test / RPM Test sliders + checkboxes send one request at a time, latest value wins,
        so a late slider request can no longer switch a test back on after it was unticked.
      - Emerald ECU RPM frame now feeds the CAN RPM source (vehicleRPMCAN) instead of writing
        the output RPM directly past Test RPM and the source selection.

V5.02 - RPM passthrough: the hall RPM input now reaches the tach output 1:1 in Hz.
      - the RPM input was converted Hz -> RPM -> Hz through two truncating map()s (and
        rounded to whole Hz first), so the output ran ~1 Hz low (~30 RPM on a 4-cyl) for
        22 of every 23 frequencies. The filtered input frequency now drives the output
        directly; Cluster Frequency Limit / Cluster RPM Limit only scale the RPM shown and
        CAN / Test RPM into Hz. The output is still capped at Cluster Frequency Limit.
      - RPM output takes fractional Hz (half period rounded to the 1 us tick) instead of
        whole-Hz steps.
      - glitch rejection is now per input: edges closer than half a period at that input's
        full scale (Max Hall Frequency / Max VR Frequency / Cluster Frequency Limit, floor
        1 ms) are dropped. It was a fixed 3 ms (333 Hz) for every input, so a real RPM
        signal above 333 Hz (6-cyl 350 Hz / 8-cyl 466 Hz at 7000 RPM) read HALF its rate.
      - RPM Filter Samples = 1 output 0 RPM (the filter reset ran after the value was set
        and cleared it); fixed.
      - own build folder (speedpulserpro-v502-pio) so the V5.01 build stays intact.

V5.03 - speed source fixes.
      - picking a speed source switches every other source off first. "Custom CAN" used to
        leave TP2.0 on, and the speed task then followed TP2.0 while the page said Custom CAN.
      - the web page's Speed Source and the Live Data highlight now come from one function
        that checks the sources in the same order as the speed task, so they always agree.
      - a source reading above Max Speed for more than 0.5 s now parks the needle at Max Speed.
        It used to hold the last in-range speed for ever, which is 0 straight after boot, so
        an ECU sending more than Max Speed never moved the needle. A short spike is still ignored.
      - the Positive Offset tick (Global Speed Offset sign) now saves. Its settings name
        "speedOffsetPositive" was 19 letters, over the ESP32's 15-letter limit, so it never
        saved and a + offset came back as - after every restart. Now saved as "spdOffsetPos".
      - own build folder (speedpulserpro-v503-pio) so the V5.02 build stays intact.
*/

#endif // VERSION_H
