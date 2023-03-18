void LORA_Telem() {
  if (LORA_TelemetBool == true) {
    logikWing();
    countLoraSend++;
    LoRa.beginPacket(); // Створення пакету
    LoRa.write((uint8_t*)&dataTelem, sizeof(dataTelem));
    LoRa.endPacket(true);
    LoRa.receive();
    LORA_TelemetBool = false;
  }
}
void LORA_SEND() {
  if (millis() - timeoutBeginPacket >= 100) {
    timeoutBeginPacket = millis();
    if (flag == false) {
      unsigned int dt = millis() - lastTime;
      if (dt >= timeout) {
        motor.write(1800);
        servo1.write(map(80, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
        servo2.write(map(180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
        servo3.write(map(0, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      }
      if (dt >= timeoutBack) {
        DISTANCE_LAT = eeprom_read_float(0);
        DISTANCE_LNG = eeprom_read_float(4);
        flag = true;
        gpsav();
      }
    }
  }
}

void onReceive(int packetSize) {
  LoRa.readBytes((uint8_t *)&dataControl, packetSize);
  byte crcc = crc16_asm((byte*)&dataControl, sizeof(dataControl)); // Считуємо crc посилки повністю
  if (crcc == 0) {
    if (dataControl.ch[9] != 205) { // якщо одержувач не це пристрій або трансляція,
      //Serial.println("Надійшло повідомлення, але не для мене.");
      return; //пропустити решту функції
    }
    FOR_i(0, 10) {
      ControlCH[i] = dataControl.ch[i];
    }
    lastTime = millis();
    if (flag == false ) {
      motor.write(map(ControlCH[0], 0, 255, 800, 2550));
      servo1.write(map(ControlCH[1], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    }
    if (ControlCH[5] != 1) {
      servo2.write(map(ControlCH[2], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      servo3.write(map(ControlCH[3], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    }
    LORA_TelemetBool = true;
    countLoraRead++;
  }
}
//ФУНКЦІЯ CRC шифрування ________________________________
byte crc16_asm(byte *buffer, byte size) {
  byte crc = 0;
  for (byte i = 0; i < size; i++) {
    byte data = buffer[i];
    uint8_t counter;
    uint8_t buffer;
    asm volatile (
      "EOR %[crc_out], %[data_in] \n\t"
      "LDI %[counter], 16          \n\t"
      "LDI %[buffer], 0x8C        \n\t"
      "_loop_start_%=:            \n\t"
      "LSR %[crc_out]             \n\t"
      "BRCC _loop_end_%=          \n\t"
      "EOR %[crc_out], %[buffer]  \n\t"
      "_loop_end_%=:              \n\t"
      "DEC %[counter]             \n\t"
      "BRNE _loop_start_%="
      : [crc_out]"=r" (crc), [counter]"=d" (counter), [buffer]"=d" (buffer)
      : [crc_in]"0" (crc), [data_in]"r" (data)
    );
  }
  return crc;
}
