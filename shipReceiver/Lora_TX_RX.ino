void LORA_Telem() {
  if (loraTelemetryBoolean == true) {
    logikWing();
    countLoraSend++;
    LoRa.beginPacket(); // Створення пакету
    LoRa.write((uint8_t*)&dataTelem, sizeof(dataTelem));
    LoRa.endPacket(true);
    LoRa.receive();
    loraTelemetryBoolean = false;
  }
}
void RTH() {
  if (millis() - timeoutBeginPacket >= 100) {
    timeoutBeginPacket = millis();
    unsigned int dt = millis() - lastCommunicationTime;
    if (dt >= communicationTimeout) {
      motor.write(1800);
      servo1.write(map(80, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      servo2.write(map(180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      servo3.write(map(0, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    }
    if (dt >= returnTimeout) {
      DISTANCE_LAT = eeprom_read_float(0);
      DISTANCE_LNG = eeprom_read_float(4);
      gpsav();
    }
  }
}

void onReceive(int packetSize) {
  LoRa.readBytes((uint8_t *)&dataControl, packetSize);
  byte crcc = crc16_asm((byte*)&dataControl, sizeof(dataControl)); // Считуємо crc посилки повністю
  if (crcc == 0 && dataControl.ch[9] == 205) {// якщо CRC вірне та наша трансляція
    memcpy(controlChannel, dataControl.ch, sizeof(controlChannel)); // Копіювання даних з dataControl.ch в controlChannel
    lastCommunicationTime = millis();// перевірка на втрату звязку
    if (!whileLoop) { // якщо прапор false, дозволяєм керувати катером
      motor.write(map(controlChannel[0], 0, 255, 800, 2550));
      servo1.write(map(controlChannel[1], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    }
    if (controlChannel[5] != 1) { // якщо controlChannel[5] = 0, дозволяєм перекидати контейнери
      servo2.write(map(controlChannel[2], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      servo3.write(map(controlChannel[3], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    }

    loraTelemetryBoolean = true;// дозволяєм відправити телеметрію
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
