#include <ServoTimer2.h> // Підключаємо бібліотеку для роботи з сервоприводами

ServoTimer2 servo1;// Створюємо об'єкти для керування сервомоторами
ServoTimer2 servo2;
ServoTimer2 servo3;
ServoTimer2 motor;

void initCompass(DFRobot_QMC5883& compass) {
  compass.begin();
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
}
void initLora() {
  LoRa.begin(437E6);//запустим на цій частоті
  LoRa.setSignalBandwidth(125E3); //defaults to 125E3. 7.8E3, 10.4E3, 15.6E3, 20.8E3 (250), 31.25E3(200), 41.7E3(150), 62.5E3(80-100), 125E3, 250E3, 500E3
  LoRa.setTxPower(20);// потужність передавання
  //LoRa.enableInvertIQ();//використовується для налаштування інверсії IQ в LoRa-модуляції.
  //LoRa.setCodingRate4(8);//коефіцієнт кодування CR4 (Coding Rate 4) використовується для збільшення стійкості передачі даних за рахунок додаткових бітів, що додаються до кожного блоку даних, що передається.
  //LoRa.setSpreadingFactor(6);//коефіцієнт розгортання (spreading factor) в LoRa-модуляторі. Коефіцієнт розгортання визначає ширину сигналу, який передається через передавальний канал, і впливає на дальність та швидкість передачі даних.
  LoRa.setSyncWord(0x44);
  LoRa.receive();// переведим радіо в режим прийому
}
void initPinOutput() {
  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  servo1.write(map(80, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  servo2.write(map(180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  servo3.write(map(0, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
}
void initESC() {
  motor.attach(6);
  motor.write(2250);
  delay(7000);
  motor.write(MIDDLE_PULSE_WIDTH);
  delay(3000);
}
