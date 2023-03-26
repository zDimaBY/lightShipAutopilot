/*
Скетч використовує 20714 байтів (67%) місця зберігання для програм. Межа 30720 байтів.
Глобальні змінні використовують 994 байтів (48%) динамічної пам’яті,  залишаючи 1054 байтів для локальних змінних. Межа 2048 байтів.
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
  initPinOutput();
  initESC();
  initCompass(compass);
  initLora();
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
  for (int i = 0; i < 14; i++) {
    byte lat_eeprom_address = i * 4;
    byte lng_eeprom_address = lat_eeprom_address + 4;

    if (controlChannel[4] == i + 1) {
      DISTANCE_LAT = eeprom_read_float(lat_eeprom_address);
      DISTANCE_LNG = eeprom_read_float(lng_eeprom_address);
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG);
      gpsav();
    } else if (controlChannel[4] == i + 2) {
      eeprom_write_float(lat_eeprom_address, DISTANCE_LAT_BUFER);
      eeprom_write_float(lng_eeprom_address, DISTANCE_LNG_BUFER);
    }
  }
}
void voltmeter() {
  float voltage = (analogRead(A0) * 5.0) / 1024.0 / (r2 / (r1 + r2));
  dataTelem.ch[0] = voltage;
  double integerPart;
  double decimalPart = modf(voltage, &integerPart);
  int decimalValue = decimalPart * 100;
  dataTelem.ch[8] = decimalValue;
}
