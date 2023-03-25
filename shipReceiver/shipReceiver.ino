/*
Скетч використовує 20762 байтів (67%) місця зберігання для програм. Межа 30720 байтів.
Глобальні змінні використовують 994 байтів (48%) динамічної пам’яті,  залишаючи 1054 байтів для локальних змінних. Межа 2048 байтів. 24.02.2023
Скетч використовує 21084 байтів (68%) місця зберігання для програм. Межа 30720 байтів.
Глобальні змінні використовують 995 байтів (48%) динамічної пам’яті,  залишаючи 1053 байтів для локальних змінних. Межа 2048 байтів. 24.02.2023

*/

#define FOR_i(from, to) for (int i = (from); i < (to); i++)

#define GPS_SERIAL_PORT_NAME "AltSoftSerial"
#define MIDDLE_PULSE_WIDTH 1800

#include <SPI.h> // Підключаємо бібліотеку для роботи з SPI-інтерфейсом
#include <LoRa.h> // Підключаємо бібліотеку для роботи з модулем LoRa
#include <avr/eeprom.h> // Підключаємо бібліотеку для роботи з енергонезалежною пам'яттю
#include <NMEAGPS.h> // Підключаємо бібліотеку для роботи з GPS-модулем (виклик функції GPS NEO-6m) dataTelem.ch[1] - GPS курс, dataTelem.ch[2] - дистанція, dataTelem.ch[3] - заданий курс, dataTelem.ch[5] - КМ/год (GPS може видавати дані 10 раз на секунду)
#include <GPSport.h>// Підключаємо бібліотеку для роботи з GPS-модулем (містить класи та функції для роботи з різними типами портів, такими як послідовний порт (Serial), SoftwareSerial, AltSoftSerial та HardwareSerial)
#include <AltSoftSerial.h> // Підключаємо бібліотеку для роботи з AltSoftSerial (порт для GPS)
#include <Wire.h> // Підключаємо бібліотеку для роботи з I2C-протоколом
#include <DFRobot_QMC5883.h> // Підключаємо бібліотеку для роботи з магнітним датчиком
#include "Initialization.h" //Підключаємо файл ініціалізацій

NMEAGPS gps; // Створюємо об'єкт для роботи з GPS-модулем
DFRobot_QMC5883 compass; // Створюємо об'єкт для роботи з магнітним датчиком

AltSoftSerial SerialGPS(8, 9);

float voltage; //Вольт метр
const float r1 = 101500.0; //опір резистора r1
const float r2 = 20000.0; // опір резистора r2

unsigned long autopilotTimeout; //общий таймер

float DISTANCE_LAT_BUFER = 0, DISTANCE_LNG_BUFER = 0;
float DISTANCE_LAT = eeprom_read_float(0), DISTANCE_LNG = eeprom_read_float(4);

unsigned long timeoutStat, timeoutBeginPacket, countLoop; // Створюєм змінні для debagStat()
byte countLoraRead, countLoraSend;

struct TX_DATA { // Гарячі пиріжки
  byte ch[10];
  byte CRC;
} dataControl;
struct RX_DATA {
  int ch[10];
  byte CRC;
} dataTelem;
byte controlChannel[16];// Канали керування 

void setup() {
  Serial.begin(115200); //відкриваємо порт для зв'язку з ПК
  SerialGPS.begin(38400); //відкриваємо порт для зв'язку з GPS
  initializingPinOutput();
  initializingESC();
  initializingCompass(compass);
  initializingLora();
  LoRa.onReceive(onReceive); //зареєструвати прийом зворотного дзвінка
}
void debagStat() {
  if (millis() - timeoutStat >= 1000) {
    timeoutStat = millis();
    Serial.print("byte: ");
    Serial.print(controlChannel[0]);
    Serial.print(" loop/с: ");
    Serial.print(countLoop);
    Serial.print(" LoraSend/с: ");
    Serial.print(countLoraSend);
    Serial.print(" LoraRead/с: ");
    Serial.println(countLoraRead);
    countLoop = 0;
    countLoraSend = 0;
    countLoraRead = 0;
  }
  countLoop++;
}
void loop() {
  GPSStatys();
  if (millis() - autopilotTimeout >= 200) { // затримка в 200ms (надто велика швидкість компаса, ардуїнці (NANO) складно ловити пакети з GPS)
    autopilotTimeout = millis();
    voltmeter();
    StatCompass();
  }
  //debagStat();
  RTH();
  switch (controlChannel[4]) {
    case 1:
      DISTANCE_LAT = eeprom_read_float(0);
      DISTANCE_LNG = eeprom_read_float(4);
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG); // Оновлюєм відстань
      gpsav();
      break;
    case 2:
      eeprom_write_float(0, DISTANCE_LAT_BUFER);
      eeprom_write_float(4, DISTANCE_LNG_BUFER);
      break;
    case 3:
      DISTANCE_LAT = eeprom_read_float(8);
      DISTANCE_LNG = eeprom_read_float(12);
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG); // Оновлюєм відстань
      gpsav();
      break;
    case 4:
      eeprom_write_float(8, DISTANCE_LAT_BUFER);
      eeprom_write_float(12, DISTANCE_LNG_BUFER);
      break;
    case 5:
      DISTANCE_LAT = eeprom_read_float(16);
      DISTANCE_LNG = eeprom_read_float(20);
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG); // Оновлюєм відстань
      gpsav();
      break;
    case 6:
      eeprom_write_float(16, DISTANCE_LAT_BUFER);
      eeprom_write_float(20, DISTANCE_LNG_BUFER);
      break;
    case 7:
      DISTANCE_LAT = eeprom_read_float(24);
      DISTANCE_LNG = eeprom_read_float(28);
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG); // Оновлюєм відстань
      gpsav();
      break;
    case 8:
      eeprom_write_float(24, DISTANCE_LAT_BUFER);
      eeprom_write_float(28, DISTANCE_LNG_BUFER);
      break;
    case 9:
      DISTANCE_LAT = eeprom_read_float(32);
      DISTANCE_LNG = eeprom_read_float(36);
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG); // Оновлюєм відстань
      gpsav();
      break;
    case 10:
      eeprom_write_float(32, DISTANCE_LAT_BUFER);
      eeprom_write_float(36, DISTANCE_LNG_BUFER);
      break;
    case 11:
      DISTANCE_LAT = eeprom_read_float(40);
      DISTANCE_LNG = eeprom_read_float(44);
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG); // Оновлюєм відстань
      gpsav();
      break;
    case 12:
      eeprom_write_float(40, DISTANCE_LAT_BUFER);
      eeprom_write_float(44, DISTANCE_LNG_BUFER);
      break;
    case 13:
      DISTANCE_LAT = eeprom_read_float(48);
      DISTANCE_LNG = eeprom_read_float(52);
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG); // Оновлюєм відстань
      gpsav();
      break;
    case 14:
      eeprom_write_float(48, DISTANCE_LAT_BUFER);
      eeprom_write_float(52, DISTANCE_LNG_BUFER);
      break;
  }
  digitalWrite(A1, controlChannel[7] > 0 ? LOW : HIGH);
  digitalWrite(A2, controlChannel[8] > 0 ? LOW : HIGH);
}

void voltmeter() {
  float voltage = (analogRead(A0) * 5.0) / 1024.0 / (r2 / (r1 + r2));
  dataTelem.ch[0] = voltage;
  double integerPart;
  double decimalPart = modf(voltage, &integerPart);
  int decimalValue = decimalPart * 100;
  dataTelem.ch[8] = decimalValue;
}
