void basicInit() {
  DEBUG_PRINTLN("Initialising SpeedPulser...");

#if serialDebug
  DEBUG_PRINTLN("Reading EEPROM...");
#endif
  readEEP();  // read EEPROM
#if serialDebug
  DEBUG_PRINTLN("Read EEPROM!");
#endif

  DEBUG_PRINTLN("Setting up LED Output...");
  pinMode(pinOnboardLED, OUTPUT);
  digitalWrite(pinOnboardLED, ledOnboard);
  DEBUG_PRINTLN("Set up LED Output!");

  DEBUG_PRINTLN("Setting up Coil Output...");
  pinMode(pinCoil, OUTPUT);
  DEBUG_PRINTLN("Set up Coil Output!");

  DEBUG_PRINTLN("Setting up PWM...");
  motorPWM = new ESP32_FAST_PWM(pinMotorOutput, pwmFrequency, dutyCycle, 0, pwmResolution);  // begin PWM with a base freq. of 10kHz @ 100% duty (mot. off)
  DEBUG_PRINTLN("Set up PWM!");

  DEBUG_PRINTLN("Setting up Speed Interrupt...");
  attachInterrupt(digitalPinToInterrupt(pinSpeedInput), incomingHz, FALLING);          //setup interrupt to toggle pin on change
  attachInterrupt(digitalPinToInterrupt(pinMotorInput), incomingMotorSpeed, FALLING);  //setup interrupt to toggle pin on change
  DEBUG_PRINTLN("Set up Speed Interrupt!");

  DEBUG_PRINTLN("Setting up Speed Interrupt...");
  canInit();
  DEBUG_PRINTLN("Set up Speed Interrupt!");

  DEBUG_PRINTLN("Setting up GPS Module...");
  ss.begin(baudGPS);
  DEBUG_PRINTLN("Set up GPS Module!");

  DEBUG_PRINTLN(TinyGPSPlus::libraryVersion());
  DEBUG_PRINTLN(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  DEBUG_PRINTLN(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  DEBUG_PRINTLN(F("----------------------------------------------------------------------------------------------------------------------------------------"));

  DEBUG_PRINTLN("Initialised SpeedPulser!");
}

void testSpeed() {
  // check to see if tempSpeed has a value.  IF it does (>0), set the speed using the 'find closest match' as a duty cycle
  if (tempSpeed == 0) {
    for (uint16_t i = 15; i < 385; i++) {  // run through all available speeds and drive the motor
      DEBUG_PRINTF("Duty: %d", i);
      DEBUG_PRINTLN("");

      motorPWM->setPWM_manual(pinMotorOutput, i);  // set the duty of the motor from the calculations
      delay(sweepSpeed * 300);
    }
  }
}

void needleSweep() {
  // ramp up
  for (int i = 0; i < maxSpeed; i++) {
    DEBUG_PRINTF("Speed: %d", i);
    DEBUG_PRINTLN("");

    dutyCycle = findClosestMatch(i * (stepSpeed / 10));
    motorPWM->setPWM_manual(pinMotorOutput, dutyCycle);
    setFrequencyRPM(i * (stepRPM / 10));
    delay(sweepSpeed);
  }

  // pause
  delay(sweepSpeed);

  // ramp down
  for (int i = maxSpeed; i > 0; i--) {  // set at >0 to stop the needle 'bouncing' when it returns to zero
    DEBUG_PRINTF("Speed: %d", i);
    DEBUG_PRINTLN("");

    dutyCycle = findClosestMatch(i * (stepSpeed / 10));
    motorPWM->setPWM_manual(pinMotorOutput, dutyCycle);
    setFrequencyRPM(i * (stepRPM / 10));
    delay(sweepSpeed);
  }
  delay(sweepSpeed);
  motorPWM->setPWM_manual(pinMotorOutput, 0);
  setFrequencyRPM(0);
}