# SpeedPulser Pro
Based on the original SpeedPulser - the Pro adds on addtional CAN and RPM functionality.  For users wishing to have speed and RPM on one board - the SpeedPulser Pro does this.

It has multiple input types - like Hall, GPS or CAN, it is designed to take these inputs and translate into a PWM signal which drives an OEM speedometer.  It’s perfect for situations where a physical speedometer cable cannot be installed or those wishing for a cleaner installation.  It is budget-friendly and therefore requires some final user setup and calibration.

If provided with a CAN interface, it can capture engine RPM and output this as a traditional coil based ignition output.

![speedPulserProUI](/Images/speedPulserProUI.png)

![Board Overview](/Images/BoardOverview.png)

## Basic Wiring
Only three wires are required to operate the SpeedPulser Pro:

Inputs:
> 12v ignition 

> Ground

Additional Inputs:
> CAN (High & Low) - for RPM/Speed if available

> RPM trigger (12v square wave)

> Speed trigger (12v square wave)

> GPS

Outputs:
> PWM to drive the motor

> Coil type RPM signal

### Sockets
| Pin | Signal | Notes | Location
|-|-|-|-|
| 1 | MK2RPM | High-voltage RPM Output | Closest to edge of PCB |
| 2 | Ground | Chassis Ground | Middle of connector |
| 3 | 12v | 12v Ignition | Closet to 5-pin connector

For reference, the motor pinout is:
| Pin/ | Signal | Notes |
|-----|--------|-------|
| 1 | Motor Power | Black - 5-9v adjustable |
| 2 | Motor Feedback | Not used |
| 3 | Motor Direction | Green - ground to reverse |
| 4 | Motor Ground | White - motor ground |
| 5 | Motor PWM | 10kHz signal from ESP32 |

| Pin | Signal | Notes | Location
|-|-|-|-|
| 1 | RPM | Square Wave RPM Input | Closest to blue potentiometer |
| 2 | Speed | Square Wave Speed Input | Middle of connector |
| 3 | CANH | CANBUS High | Middle of connector |
| 4 | CANL | CANBUS Low | Closest to edge 5 pin connector |

Software Benefits:
> Fully calibrated motor: Duty Percentage vs. Resulting Speed
> Needle Sweep available
> WiFi Enabled

## WiFi Support
WiFi now enabled for easier on-the-fly setup.  Needle sweep & calibration can be done without needing to mess with complicated firmware.
Connect to 'SpeedPulser Pro' on WiFi and use IP Address 192.168.1.1 in your browser to adjust settings!

## Coupler Installation
As the couplers are 3D printed, each coupler, motor housing and speedometer and the alignment of the sun and moon will all be unique and therefore final torque applied to the cluster will be different.  There is a potential for slight variances in final achieved speed.  Therefore, it is the end users requirement to install the coupler and ensure appropriate alignment.  Once installed, the 'highest' speed should be achieved and the potentiometer adjusted to calibrate.  

Press the small button on the right-hand side (closest to the edge of the PCB) to drive the motor at full speed and calibrate to the maximum speed on the speedometer to suit.  This may need a few iterations are parts wear into each other.  

The coupler will be a tight fit on the motor as will the brass driving shaft.  A few light taps will encourge them on.  Remember that this is budget friendly and while a metal alternative would be neat, it's not within budget.

#### Pull-up or Pull-down?
Different hall sensors from different manufacturers may require a pull-up or pull-down resistor.  There is a 2-way header on the board to select this.  If you find the SpeedPulserPro is not detecting incoming pulses, move this jumper.  Some hall sensors may have their own internal resistor, so the jumper can be removed entirely.  It is labelled 'SpeedPulser'

### R-Term
The jumper marked 'r-term' is the terminating resistor for the CANBUS network.  If there are no other CAN devices on the network (this is the only one), the jumper should remain.  If there are other devices on the network, this can be removed.

### Trimming the Drive Shaft
Drive shafts are supplied in excess of required length and should be trimmed to suit each individual cluster.  Typically this is ~level with the motor housing but confirmation that it is not bottomed out is key!

## Calibration
If final speedometer reading is incorrect across the range due to tyre size or gearbox variations, the source code can be editted to suit new readings.
The main array for 'Speed' vs. 'PWM' is within '_defs.h' under 'motorPerformance'.  8-bit resolution is duty between 0-100%, but for some fine granularity, 10 bit resolution is the default 'shipped' with.  Therefore there are ~400 entries of PWM, with each having it's own speed.
> uint8_t motorPerformance[] = {0, 0, 0, 0...};

## Understanding the Excel
There is a plot of motor speed vs. PWM in an Excel.  Should it be required, the module can be put into 'testing mode' and each PWM variance worked through in 5 second intervals.  It takes around 10 minutes to carry out a full sweep of the available speeds, but will give good accuracy.  

## Disclaimer
It should be noted that this will drive a speedometer, but it should always be assumed that it is inaccurate and therefore no responsibilty on speed related incidents are at fault of Forbes Automotive.  
