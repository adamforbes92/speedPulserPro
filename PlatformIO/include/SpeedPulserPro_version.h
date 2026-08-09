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
*/

#endif // VERSION_H
