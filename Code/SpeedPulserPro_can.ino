void canInit() {
  chassisCAN.setRX(pinRX_CAN);
  chassisCAN.setTX(pinTX_CAN);
  chassisCAN.begin();
  chassisCAN.setBaudRate(500000);  // CAN Speed in Hz
  chassisCAN.onReceive(onBodyRX);
  // set filters up for focusing on only MOT1 / MOT 2?
}

void onBodyRX(const CAN_message_t& frame) {
#if ChassisCANDebug  // print incoming CAN messages
  Serial.print("Length Recv: ");
  Serial.print(frame.len);
  Serial.print(" CAN ID: ");
  Serial.print(frame.id, HEX);
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < frame.len; i++) {
    Serial.print(frame.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

  lastCAN = millis();

  switch (frame.id) {
    case MOTOR1_ID:
      // frame[2] (byte 3) > motor speed low byte
      // frame[3] (byte 4) > motor speed high byte
      // frame[4] (byte 3) > khm speed?
      vehicleRPMCAN = ((frame.buf[3] << 8) | frame.buf[2]) * 0.25;  // conversion: 0.25*HEX
      break;

    case MOTOR2_ID:
      ecuSpeed = (frame.buf[3] * 100 * 128) / 10000;
      break;

    case MOTOR5_ID:
      // set EML & EPC based on the bit read (LSB, so backwards)
      vehicleEML = bitRead(frame.buf[1], 5);
      vehicleEPC = bitRead(frame.buf[1], 6);
      break;

    case MOTOR6_ID:
      if (frame.buf[0] == 0x73 || frame.buf[0] == 0x72) {
        vehicleReverse = true;
      } else {
        vehicleReverse = false;
      }
      if (frame.buf[0] == 0x83 || frame.buf[0] == 0x82) {
        vehiclePark = true;  // unused bool, but a good to have...
      } else {
        vehiclePark = false;  // unused bool, but a good to have...
      }
      break;

    case BRAKES3_ID:
      absSpeed = ((frame.buf[3] << 8) | frame.buf[2]) * 1.28;  // conversion: 0.25*HEX
      break;

    case mWaehlhebel_1_ID:
      gear_raw = ((frame.buf[7] & 0b01110000) >> 4) - 1;
      lever_raw = (frame.buf[7] & 0b00000001);

      if (lever_raw) {
        gear = gear_raw;

        switch (gear) {
          case 3:  // reverse
            vehicleReverse = true;
            break;
          default:
            vehicleReverse = false;
            break;
        }

        if (gear == 0xFF) {
          gear = 1;
        }
      }
      break;

    case gearLever_ID:
      lever = (frame.buf[0] & 0b11110000) >> 4;
      break;

    case emeraldECU1_ID:
      vehicleRPM = ((frame.buf[0] << 8) | frame.buf[1]);  // conversion: 0.25*HEX // this is RPM
      break;

    case emeraldECU2_ID:
      ecuSpeed = ((frame.buf[2] << 8) | frame.buf[3]) * (2.25 / 256);  // conversion: 0.25*HEX // this is RPM
      break;

    default:
      // do nothing...
      break;
  }
    // do the calc


#if stateDebug
  Serial.println();
  Serial.print("vehicleRPM: ");
  Serial.println(vehicleRPM);

  Serial.print("vehicleSpeed: ");
  Serial.println(vehicleSpeed);

  Serial.print("vehicleEML: ");
  Serial.println(vehicleEML);

  Serial.print("vehicleEPC: ");
  Serial.print(vehicleEPC);
#endif
}

void sendPaddleUpFrame() {
  CAN_message_t paddlesUp;  //0x7C0
  paddlesUp.id = GRA_ID;
  paddlesUp.len = 8;
  paddlesUp.buf[0] = 0x0E;             // was 0xB7
  paddlesUp.buf[2] = 0x0C;             // was 0x34
  paddlesUp.buf[3] = 0x02;             //
  bitSet(paddlesUp.buf[3], 1);         // set high (trigger)
  if (!chassisCAN.write(paddlesUp)) {  // write CAN frame from the body to the Haldex
  }
}

void sendPaddleDownFrame() {
  CAN_message_t paddlesDown;  //0x7C0
  paddlesDown.id = GRA_ID;
  paddlesDown.len = 8;
  paddlesDown.buf[0] = 0x0D;  //
  paddlesDown.buf[2] = 0x0C;
  paddlesDown.buf[3] = 0x01;
  bitSet(paddlesDown.buf[3], 1);         // set high (trigger)
  if (!chassisCAN.write(paddlesDown)) {  // write CAN frame from the body to the Haldex
  }
}