unsigned int communicationTimeout = 500; // інтервал розриву зв'язку
unsigned int returnTimeout = 60000; // інтервал повернення після розриву зв'язку
unsigned int lastCommunicationTime;

void logikWing() {
  dataTelem.ch[9] = 210;
  dataTelem.CRC = crc16_asm((byte*)&dataTelem, sizeof(dataTelem) - 1);
}
void LoraTelemetrySend() {
  logikWing();//готуєм для верифікацій
  LoRa.beginPacket(); // Створення пакету
  LoRa.write((uint8_t*)&dataTelem, sizeof(dataTelem));
  LoRa.endPacket(true);
  LoRa.receive();
  countLoraSend++;
}
void RTH() {
  if (millis() - timeoutBeginPacket >= 100) {
    timeoutBeginPacket = millis();
    unsigned int dt = millis() - lastCommunicationTime;
    if (dt >= communicationTimeout) {
      if (!whileLoop) { // Якщо whileLoop == false, дозволяємо RTH керувати катером
        motor.write(MIDDLE_PULSE_WIDTH);
        servo1.write(map(80, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
        servo2.write(map(180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
        servo3.write(map(0, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      }
    }
    if (dt >= returnTimeout) {
      DISTANCE_LAT = eeprom_read_float(0);
      DISTANCE_LNG = eeprom_read_float(4);
      gpsav();
    }
  }
}
void onReceive(int packetSize) {// Функція onReceive приймає параметр packetSize, який вказує на розмір пакету.
  LoRa.readBytes((uint8_t *)&dataControl, packetSize); //читає цілі байти з LoRa-модуля та записує їх у змінну dataControl
  byte crcc = crc16_asm((byte*)&dataControl, sizeof(dataControl)); // Обчислюємо контрольну суму (CRC) для отриманих даних
  if (crcc == 0 && dataControl.ch[9] == 205) { // Перевіряємо правильність CRC та перевіряємо, чи це наша трансляція
    LoraTelemetrySend();// Відправляємо телеметрію
    memcpy(controlChannel, dataControl.ch, sizeof(controlChannel)); // Копіюємо дані з dataControl.ch у controlChannel
    lastCommunicationTime = millis(); // Зберігаємо час останньої комунікації
    if (!whileLoop) { // Якщо whileLoop == false, дозволяємо керувати катером
      motor.write(map(controlChannel[0], 0, 255, 800, 2550));
      servo1.write(map(controlChannel[1], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    }
    if (controlChannel[5] != 1) { // Якщо controlChannel[5] = 0, дозволяємо перекидати контейнери
      servo2.write(map(controlChannel[2], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      servo3.write(map(controlChannel[3], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    }
    turnLights();// перевірка чи потрібно вимкнення світла
    countLoraRead++; // Збільшуємо лічильник отриманих пакетів
  }
}
void turnLights(){
  digitalWrite(A1, controlChannel[7] > 0 ? LOW : HIGH);
  digitalWrite(A2, controlChannel[8] > 0 ? LOW : HIGH);
}
//ФУНКЦІЯ CRC шифрування ________________________________
byte crc16_asm(byte * buffer, byte size) {
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
