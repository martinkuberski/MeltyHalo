void pollSerial() {
  while(RC.available() == 8) {
    if(serialState == SERIAL_WAIT) {
        serialState = SERIAL_PACKETSTART;
        bytesRead = 0;
        continue;
    }
    packet[bytesRead] = RC.read(bytesRead + 1);
    bytesRead++;

    if(bytesRead == 6) {
      receivePacket();
      bytesRead = 0;
      serialState = SERIAL_WAIT;
      break;
    }
    
  }
}

void receivePacket() {
  lastReceived = micros();
  
  //status byte
  //int stat = packet[0];//nothing is currently done here. Can be used to transmit switch states, etc.
  //if(senseMode != BEACON_SENSING) {//If we are in beacon-only mode due to a fault or deliberate code change, the sense select switch does nothing
  //  if(stat & 0x01) {//bit one is the sense select switch for choosing whether beacon is used
  //    senseMode = HYBRID_SENSING;
  //  } else {
  //    senseMode = ACCEL_SENSING;
  //  }
  // }

  //flip = (stat & 0x02) > 0;//indicates whether the bot is inverted. 1 is inverted, 0 is normal

  bool tankOverride = map((uint16_t) packet[4], 1000, 2000, false, true); //this set the throttle to 0, forcing the bot into tank mode. Faster than adjusting the throttle pot.

  //thumbstick X
  thumbX = map((uint16_t) packet[0], 1000, 2000, -100, 100);
  //thumbstick Y
  thumbY = map((uint16_t) packet[1], 1000, 2000, -100, 100);
  //throttle
  if(tankOverride) {
    throt = 0;      
  } else {
    throt = map((uint16_t) packet[2], 1000, 2000, 0, 100);
  }
  //heading - 
  head = map((uint16_t) packet[5], 1000, 2000, -180, 180);
  //enable
  //en = packet[9];
  en = 1;
  if(state == STATE_SPIN) {
    //calculate the commanded direction and speed
    meltyThrottle = sqrt(thumbX*thumbX + thumbY*thumbY)/2;
   int16_t calcAngle = (int16_t) (atan2((double) thumbY, (double) thumbX*(flip*2-1))*180.0/PI);
   if(calcAngle < 0) calcAngle += 360;
   meltyAngle = (uint16_t) calcAngle;
  }

  //now we build the return packet (unused)
  //packet[0] = 0x7E;//start of packet byte
  
  //standard return packet
  //uint16_t batteryVoltage = getBatteryVoltage();
  //packet[1] = (byte) ((batteryVoltage & 0xFF00) >> 8);
  //packet[2] = (byte) (batteryVoltage & 0x00FF);
  //packet[3] = 0x00;

  //Serial1.write(packet, 4);
  //*/
  
  /*/this code is used in calibration testing
  packet[1] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0xFF000000) >> 24);
  packet[2] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x00FF0000) >> 16);
  packet[3] = (byte) (((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x0000FF00) >> 8);
  packet[4] = (byte) ((beaconEdgeTime[0] - beaconEdgeTime[1]) & 0x000000FF);
  packet[5] = (byte) ((zAccel & 0xFF00) >> 8);
  packet[6] = (byte) (zAccel & 0x00FF);

  Serial1.write(packet, 7);
  //*/
}
