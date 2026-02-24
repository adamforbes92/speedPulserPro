/*
SpeedPulser Pro - Forbes Automotive '25

A blend of the SpeedPulser and Can2Cluster.  Offering speed and RPM output to MK1 & MK2 Golf clusters.  Supports CAN (for RPM/Speed) and GPS (for speed).
Can capture hall sensor input for traditional speed input.  Can broadcast speed via. CAN or GPS if req.
Note that RPM output is high voltage and MAY cause interference between hall sensors.  Care should be taken to route the cables AWAY from each other.
*/

#include "speedPulserPro_defs.h"

// for motor PWM
ESP32_FAST_PWM* motorPWM;                              // for PWM control.  ESP Boards need to be V2.0.17 - the latest version has known issues with LEDPWM(!)
RunningMedian samples = RunningMedian(averageFilter);  // for calculating median samples - there can be 'hickups' in the incoming signal, this helps remove them(!)

// for CAN
#include <ESP32_CAN.h>
ESP32_CAN<RX_SIZE_256, TX_SIZE_16> chassisCAN;

// for GPS
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
SoftwareSerial ss(pinRX_GPS, pinTX_GPS);
TinyGPSPlus gps;

TickTwo tickEEP(writeEEP, eepRefresh);               // refresh EEP
TickTwo tickWiFiLabels(updateLabels, labelRefresh);  // refresh EEP
Preferences pref;                                    // to record previously saved settings

//create an object from the UpdateServer
ESPAsyncHTTPUpdateServer updateServer;
AsyncWebServer server(80);

hw_timer_t* timer0 = NULL;
bool rpmTrigger = true;
long frequencyRPM = 20;  // 0 to 20000

void IRAM_ATTR onTimer0() {
  rpmTrigger = !rpmTrigger;           // flip/flop RPM trigger - will create a 50% duty at X freq.
  digitalWrite(pinCoil, rpmTrigger);  // drive coil transistor
}

// interrupt routine for the incoming pulse
void incomingHz() {                                               // Interrupt 0 service routine
  static unsigned long previousMicros = micros();                 // remember variable, initialize first time
  unsigned long presentMicros = micros();                         // read microseconds
  unsigned long revolutionTime = presentMicros - previousMicros;  // works fine with wrap-around of micros()
  if (revolutionTime < 1000UL) return;                            // avoid divide by 0, also debounce, speed can't be over 60,000 was 1000UL
  dutyCycleIncoming = (60000000UL / revolutionTime) / 60;         // calculate
  previousMicros = presentMicros;
  lastPulse = millis();

  ledCounter++;  // count LED counter - is used to flash onboard LED to show the presence of incoming pulses
}

void incomingMotorSpeed() {                                       // Interrupt 0 service routine
  static unsigned long previousMicros = micros();                 // remember variable, initialize first time
  unsigned long presentMicros = micros();                         // read microseconds
  unsigned long revolutionTime = presentMicros - previousMicros;  // works fine with wrap-around of micros()
  if (revolutionTime < 1000UL) return;                            // avoid divide by 0, also debounce, speed can't be over 60,000 was 1000UL
  dutyCycleMotor = (60000000UL / revolutionTime) / 60;            // calculate
  previousMicros = presentMicros;

  lastPulseRPM = millis();
}

// setup timers
void setupTimer() {
  timer0 = timerBegin(0, 40, true);  //div 80
  timerAttachInterrupt(timer0, &onTimer0, true);
}

// adjust output frequency
void setFrequencyRPM(long frequencyHz) {
  if (frequencyHz != 0) {
    timerAlarmDisable(timer0);
    timerAlarmWrite(timer0, 1000000l / frequencyHz, true);
    timerAlarmEnable(timer0);
  } else {
    timerAlarmDisable(timer0);
  }
}

void setup() {
#ifdef serialDebug || serialDebugIncoming || serialDebugWifi || serialDebugEEP || serialDebugGPS || ChassisCANDebug
  Serial.begin(115200);
  DEBUG_PRINTLN("Initialising SpeedPulser Pro...");
#endif

  basicInit();   // init PWM, Serial, Pin IO etc.  Kept in '_io.ino' for cleanliness due to the number of Serial outputs
  setupTimer();  // setup the timers (with a base frequency)
  updateMotorArray();

  if (hasNeedleSweep) {
    needleSweep();  // enable needle sweep (in _io.ino).  Get this done immediately after setup so there isn't a visible 'lag'
  }

  tickEEP.start();  // begin ticker for the EEPROM
  tickWiFiLabels.start();

  motorPWM->setPWM(pinMotorOutput, pwmFrequency, dutyCycle);  // set motor to off in first instance (100% duty)

  connectWifi();         // enable / start WiFi
  WiFi.setSleep(false);  // for the ESP32: turn off sleeping to increase UI responsivness (at the cost of power use)
  setupUI();             // setup wifi user interface
  setupOTA();            // setup Over-the-Air updates

  //WiFi.setTxPower(WIFI_POWER_8_5dBm);  // set a lower power mode (some C3 aerials aren't great and leaving it high causes failures)
}

void loop() {
  tickEEP.update();  // refresh the EEP ticker
  tickWiFiLabels.update();

  parseGPS();  // check for GPS updates - not an issue if not connected

  if (tempNeedleSweep) {  // only here if tested in WiFi
    needleSweep();
    tempNeedleSweep = false;  // reset the flag
  }

  if (ledCounter > averageFilter) {
    ledOnboard = !ledOnboard;                 // flip-flop the led trigger
    digitalWrite(pinOnboardLED, ledOnboard);  // flash the LED to show the presence of incoming pulses
    ledCounter = 0;                           // reset the counter
  }

  if (!testSpeedo) {
    if ((millis() + 10 - lastPulse) > durationReset) {  // it's been a while since the last hall input, so update to say 0 speed and reset vars
      dutyCycle = 0;
      dutyCycleIncoming = 0;
      vehicleSpeedHall = 0;
    }

    if ((dutyCycle != dutyCycleIncoming)) {  // only update PWM IF speed has changed (can cause flicker otherwise)
#if serialDebugIncoming
      DEBUG_PRINTF("     DutyIncomingHall: %d", dutyCycleIncoming);  // what is the incoming pulse count?
#endif
      dutyCycleIncoming = map(dutyCycleIncoming, 0, maxFreqHall, 0, maxSpeed);  // map incoming range to this codes range.  Max Hz should match Max Speed - i.e., 200Hz = 200kmh, or 500Hz = 200kmh...
#if serialDebugIncoming
      DEBUG_PRINTF("     DutyPostProc1Hall: %d", dutyCycle);  // what is the new 'pulse count' - mapped to min/max hall and min/max speed
#endif

      if (rawCount < averageFilter) {
        samples.add(dutyCycleIncoming);  // add to a list to create an average
        rawCount++;
      }

      if (rawCount >= averageFilter) {
        hallSpeed = samples.getAverage(averageFilter / 2);  // get the average
#if serialDebugIncoming
        DEBUG_PRINTF("     getAverageHall: %d", dutyCycle);
#endif
        rawCount = 0;
        samples.clear();  // clear the list
      }

      dutyCycle = dutyCycleIncoming;  // re-introduce?  Could do some filter on big changes?  May skip over genuine changes though?
#if serialDebugIncoming
      DEBUG_PRINTLN("");
#endif
    }
  }

  // if testSpeedo, apply offests to to confirm working
  if (testSpeedo) {
    if (speedOffsetPositive) {
      dutyCycle = tempSpeed + speedOffset;
#if serialDebugIncoming
      DEBUG_PRINTLN("+");
      DEBUG_PRINTLN(dutyCycle);
#endif
    } else {
      if (tempSpeed - speedOffset > 0) {
        dutyCycle = tempSpeed - speedOffset;
#if serialDebugIncoming
        DEBUG_PRINTLN("-");
        DEBUG_PRINTLN(dutyCycle);
#endif
      } else {
        dutyCycle = 0;
      }
    }
  }

  //if NOT testSpeedo, apply the caught variables (Hall, GPS or CAN) and transfer across, applying any maps beforehand
  if (!testSpeedo) {
    vehicleSpeed = 0;
    if (useHall) {
      vehicleSpeed = hallSpeed;
    }
    if (useECU) {
      //vehicleSpeedECU = map(vehicleSpeedCAN, 0, maxFreqCAN, 0, maxSpeed);  // map incoming range to this codes range.  Max Hz should match Max Speed - i.e., 200Hz = 200kmh, or 500Hz = 200kmh...
      vehicleSpeed = ecuSpeed;
    }
    if (useABS) {
      //vehicleSpeedECU = map(vehicleSpeedCAN, 0, maxFreqCAN, 0, maxSpeed);  // map incoming range to this codes range.  Max Hz should match Max Speed - i.e., 200Hz = 200kmh, or 500Hz = 200kmh...
      vehicleSpeed = absSpeed;
    }
    if (useDSG) {
      vehicleSpeed = dsgSpeed;
    }
    if (useGPS) {
      vehicleSpeed = gpsSpeed;
    }

    // we now have a final speed - so apply any offsets
    if (speedOffsetPositive) {
      dutyCycle = vehicleSpeed + speedOffset;
#if serialDebugIncoming
      DEBUG_PRINTLN("+");
      DEBUG_PRINTLN(dutyCycle);
#endif
    } else {
      if (vehicleSpeed - speedOffset > 0) {
        dutyCycle = vehicleSpeed - speedOffset;
#if serialDebugIncoming
        DEBUG_PRINTLN("-");
        DEBUG_PRINTLN(dutyCycle);
#endif
      } else {
        dutyCycle = 0;
      }
    }
  }

  if (testRPM) {  // set vehicleRPM is testing or not
    vehicleRPM = tempRPM;
  } else {
    if ((millis() + 10 - lastPulseRPM) > durationReset) {  // it's been a while since the last hall input, so update to say 0 speed and reset vars
      vehicleRPMHall = 0;
    } else {
      vehicleRPMHall = map(dutyCycleMotor, 0, maxRPM, 0, clusterRPMLimit);
      vehicleRPM = vehicleRPMHall;
    }
    if (vehicleRPMCAN > 0) {
      vehicleRPM = vehicleRPMCAN;
    }
  }

  // change the frequency of both RPM & Speed as per receieved information
  if ((millis() - lastMillis2) > rpmPause) {  // check to see if x ms (rpmPause) has elapsed - slow down the frames!
    lastMillis2 = millis();

    // set RPM frequency
    frequencyRPM = map(vehicleRPM, 0, clusterRPMLimit, 0, maxRPM);
    setFrequencyRPM(frequencyRPM);  // minimum speed may command 0 and setFreq. will cause crash, so +1 to error 'catch'

    // set motor duty cycle
    dutyCycle = findClosestMatch(dutyCycle);
    motorPWM->setPWM_manual(pinMotorOutput, dutyCycle);
  }
}

uint16_t findClosestMatch(uint16_t val) {
  // for finding the nearest match of speed from the incoming duty.  There may be instances where the incoming speed does not equal a value in the array, so find the 'nearest' value
  // has to be 16_t due to 10 bit resolution
  // the 'find'/function returns array position, which is equal to the duty cycle
  uint16_t closest = 0;
  uint16_t closest2 = 0;
  uint16_t i = 0;
  bool speedTest = false;

  for (i = 0; i < sizeof motorPerformance / sizeof motorPerformance[0]; i++) {
    if (motorPerformance[i] > 0) {
      if (abs(val) > motorPerformance[i]) {
        speedTest = true;
        i = (sizeof motorPerformance / sizeof motorPerformance[0]);
      }
    }
  }

  if (speedTest) {
    for (i = 0; i < sizeof motorPerformance / sizeof motorPerformance[0]; i++) {
      if (abs(val - closest) >= abs(val - motorPerformance[i])) {
        closest = motorPerformance[i];
      }
    }

    for (i = 0; i < sizeof motorPerformance / sizeof motorPerformance[0]; i++) {
      if (motorPerformance[i] == closest) {
        closest2 = i;
        i = (sizeof motorPerformance / sizeof motorPerformance[0]);
      }
    }

    if (closest2 >= 385) {  // will run into the end of the array IF it's not found a speed, so return 0!
      return 0;
    } else {
      return closest2;  // else return the correct duty cycle from the found speed
    }
  } else {
    return 0;
  }
}

void updateLabels() {
  if ((millis() + 10 - lastCAN) > 500) {
    hasCAN = false;
  } else {
    hasCAN = true;
  }

  if (updateMotorPerformance) {
    updateMotorArray();
  }

  if (hasGPS) {
    char bufhasGPS[50];
    sprintf(bufhasGPS, "Has GPS: Yes (%d satellites)", gps.satellites.value());
    ESPUI.updateLabel(label_hasGPS, String(bufhasGPS));
  } else {
    ESPUI.updateLabel(label_hasGPS, "Has GPS: No");
  }

  if (hasCAN) {
    ESPUI.updateLabel(label_hasCAN, "Has CAN: Yes");
  } else {
    ESPUI.updateLabel(label_hasCAN, "Has CAN: No");
  }

  ESPUI.updateLabel(label_speedHall, String(hallSpeed));  // should be hall
  ESPUI.updateLabel(label_speedGPS, String(gpsSpeed));
  ESPUI.updateLabel(label_speedECU, String(ecuSpeed));
  ESPUI.updateLabel(label_speedABS, String(absSpeed));
  ESPUI.updateLabel(label_speedDSG, String(dsgSpeed));

  ESPUI.updateLabel(label_RPMHall, String(vehicleRPMHall));
  ESPUI.updateLabel(label_RPMCAN, String(vehicleRPMCAN));

  if (useHall) {
    ESPUI.updateSelect(int16_speedType, "Hall");
  }
  if (useABS) {
    ESPUI.updateSelect(int16_speedType, "ABS");
  }
  if (useDSG) {
    ESPUI.updateSelect(int16_speedType, "DSG");
  }
  if (useGPS) {
    ESPUI.updateSelect(int16_speedType, "GPS");
  }
  if (useECU) {
    ESPUI.updateSelect(int16_speedType, "ECU");
  }

  switch (motorPerformanceVal) {
    case 1:
      ESPUI.updateSelect(int16_calNumber, "VW120Martin");
      break;
    case 2:
      ESPUI.updateSelect(int16_calNumber, "VW120Forbes");
      break;
    case 3:
      ESPUI.updateSelect(int16_calNumber, "VW140Forbes");
      break;
    case 4:
      ESPUI.updateSelect(int16_calNumber, "VW160Forbes");
      break;
    case 5:
      ESPUI.updateSelect(int16_calNumber, "VW300Forbes");
      break;
    case 6:
      ESPUI.updateSelect(int16_calNumber, "Ford120Forbes1");
      break;
    case 7:
      ESPUI.updateSelect(int16_calNumber, "Ford120Forbes2");
      break;
    case 8:
      ESPUI.updateSelect(int16_calNumber, "FIAT160Forbes1");
      break;
    case 9:
      ESPUI.updateSelect(int16_calNumber, "FIAT160Forbes2");
      break;
    case 10:
      ESPUI.updateSelect(int16_calNumber, "Merc120Forbes");
      break;
    case 11:
      ESPUI.updateSelect(int16_calNumber, "Smiths70Forbes");
      break;
    case 12:
      ESPUI.updateSelect(int16_calNumber, "Smiths70Forbes");
      break;
    case 13:
      ESPUI.updateSelect(int16_calNumber, "Smiths70Forbes");
      break;
    case 14:
      ESPUI.updateSelect(int16_calNumber, "Smiths70Forbes");
      break;
  }
}