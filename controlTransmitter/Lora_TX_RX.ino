void logikWing(void) {
  dataControl.ch[0] = constrain(map(analogRead(MOTOR_POT_PIN), 0, 1023, minDrive, maxDrive), 130, motorLimit);
  dataControl.ch[1] = constrain(map(analogRead(ROLE_POT_PIN), leftRole, rightRole, 60, 100), 60, 100);
  dataControl.ch[2] = map(analogRead(SERVO1_POT_PIN), 0, 1023, 30, 180);
  dataControl.ch[3] = map(analogRead(SERVO2_POT_PIN), 0, 1023, 0, 150);
  dataControl.ch[9] = 205;
  dataControl.CRC = crc16_asm((byte*)&dataControl, sizeof(dataControl) - 1);
}
void LORA_SEND(void) {
  if (millis() - timeoutBeginPacket >= 100) {
    timeoutBeginPacket = millis();
    logikWing();
    countLORA++;
    LoRa.beginPacket(); // Створення пакету
    LoRa.write((uint8_t*)&dataControl, sizeof(dataControl));
    LoRa.endPacket(true);
    LoRa.receive();
  }
  debag();
}
void onReceive(int packetSize) {
  LoRa.readBytes((uint8_t *)&dataTelem, packetSize);
  byte crc = crc16_asm((byte*)&dataTelem, sizeof(dataTelem));
  if (crc == 0 && dataTelem.ch[9] == 210) {
    FOR_i(0, 10) {
      Telemetry[i] = dataTelem.ch[i];
    }
    countLoraRead++;
  }
}
//ФУНКЦІЯ CRC шифрування ________________________________
byte crc16_asm(byte *buffer, byte size) {
  byte crc = 0;
  FOR_i(0, size) {
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
