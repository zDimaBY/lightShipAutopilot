#define FOR_i(from, to) for (int i = (from); i < (to); i++)

#define GPS_PORT_NAME "AltSoftSerial"

#include <SPI.h>  // Підключаємо бібліотеку для роботи з SPI-інтерфейсом
#include <LoRa.h>
#include <avr/eeprom.h> // Енергонезалежна пам'ять
#include <NMEAGPS.h> // (виклик функції GPS NEO-6m) dataTelem.ch[1] - GPS курс, dataTelem.ch[2] - дистанція, dataTelem.ch[3] - азимут, dataTelem.ch[5] - КМ/год (GPS може видавати дані 1 раз на секунду)
#include <GPSport.h>//Бібліотека GPSport.h є частиною бібліотеки NeoGPS і містить класи та функції для роботи з різними типами портів, такими як послідовний порт (Serial), SoftwareSerial, AltSoftSerial та HardwareSerial.
#include <ServoTimer2.h>
#include <AltSoftSerial.h> //Підключений GPS
#include <Wire.h> //для QMC5883
#include <DFRobot_QMC5883.h>

ServoTimer2 motor;
ServoTimer2 servo1;
ServoTimer2 servo2;
ServoTimer2 servo3;
NMEAGPS gps;
DFRobot_QMC5883 compass;

AltSoftSerial SerialGPS(8, 9);
bool flag = false;

unsigned int timeout = 500; // інтервал розриву зв'язку
unsigned int timeoutBack = 60000; // інтервал повернення після розриву зв'язку
unsigned int lastTime = 0;

float volt; //Вольт метр
float temp = 0.0;
const float r1 = 101500.0; //опір резистора r1
const float r2 = 20000.0; // опір резистора r2

byte motorspeed, limitspeed; // Для автопілота
int delta, conteinerMillis;
unsigned long timeoutAV, timeoutAV1;
bool GPS_ON = false, target_on = false, is_one1 = false, LORA_TelemetBool = false;

float DISTANCE_LAT_BUFER = 0, DISTANCE_LNG_BUFER = 0;
float DISTANCE_LAT = eeprom_read_float(0), DISTANCE_LNG = eeprom_read_float(4);

unsigned long timeoutStat, timeoutBeginPacket, countLoop;
byte countLoraRead, countLoraSend;

struct TX_DATA {
  byte ch[10];
  byte CRC;
} dataControl;

struct RX_DATA {
  int ch[10];
  byte CRC;
} dataTelem;

byte ControlCH[16];

float xv, yv, zv; //-------------------- КОМПАС!!!
//calibrated_values[3] це глобальний масив, куди будуть розміщені калібровані дані
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
float calibrated_values[3];
//transformation(float uncalibrated_values[3]) це функція корекції даних магнітометра
//uncalibrated_values[3] це масив даних некаліброваного магнітометра
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc
//vector_length_stabilasation() - – функція стабілізації довжини вектора магнітометра (стабілізації радіуса сфери)
float scaler;
boolean scaler_flag = false;
float normal_vector_length;

void setup() {
  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  motor.attach(6);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  servo1.write(map(80, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  servo2.write(map(180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  servo3.write(map(0, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  Serial.begin(9600); //відкриваємо порт для зв'язку з ПК
  SerialGPS.begin(38400); //відкриваємо порт для зв'язку з GPS
  compass.begin();
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  LoRa.begin(437E6);
  LoRa.onReceive(onReceive); //зареєструвати прийом зворотного дзвінка
  LoRa.receive();// переведим радіо в режим прийому
  LoRa.setSignalBandwidth(250E3); //defaults to 125E3. 7.8E3, 10.4E3, 15.6E3, 20.8E3 (250), 31.25E3(200), 41.7E3(150), 62.5E3(80-100), 125E3, 250E3, 500E3
  LoRa.setTxPower(20);
  //LoRa.enableInvertIQ(); // Інверсія
  //LoRa.setCodingRate4(8);
  //LoRa.setSpreadingFactor(6);
  LoRa.setSyncWord(0x44);
  motor.write(2250);
  delay(7000);
  motor.write(1800);
  delay(3000);

}
void debagStat() {
  if (millis() - timeoutStat >= 1000) {
    timeoutStat = millis();
    Serial.print(" byte: ");
    Serial.print(ControlCH[0]);
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
  if (millis() - timeoutAV >= 200) { // затримка в 50ms (бо велика швидкість компаса з цього ардуїнці (NANO) складно ловити пакети з GPS)
    timeoutAV = millis();
    StatCompass();
    voltmeter();
  }
  //debagStat();
  LORA_SEND();
  LORA_Telem();
  if (ControlCH[4] == 1) {
    DISTANCE_LAT = eeprom_read_float(0);
    DISTANCE_LNG = eeprom_read_float(4);
    flag = true;
    gpsav();
  }
  if (ControlCH[4] == 2) {
    eeprom_write_float(0, DISTANCE_LAT_BUFER);
    eeprom_write_float(4, DISTANCE_LNG_BUFER);
  }
  if (ControlCH[4] == 3) {
    DISTANCE_LAT = eeprom_read_float(8);
    DISTANCE_LNG = eeprom_read_float(12);
    flag = true;
    gpsav();
  }
  if (ControlCH[4] == 4) {
    eeprom_write_float(8, DISTANCE_LAT_BUFER);
    eeprom_write_float(12, DISTANCE_LNG_BUFER);
  }
  if (ControlCH[4] == 5) {
    DISTANCE_LAT = eeprom_read_float(16);
    DISTANCE_LNG = eeprom_read_float(20);
    flag = true;
    gpsav();
  }
  if (ControlCH[4] == 6) {
    eeprom_write_float(16, DISTANCE_LAT_BUFER);
    eeprom_write_float(20, DISTANCE_LNG_BUFER);
  }
  if (ControlCH[4] == 7) {
    DISTANCE_LAT = eeprom_read_float(24);
    DISTANCE_LNG = eeprom_read_float(28);
    flag = true;
    gpsav();
  }
  if (ControlCH[4] == 8) {
    eeprom_write_float(24, DISTANCE_LAT_BUFER);
    eeprom_write_float(28, DISTANCE_LNG_BUFER);
  }
  if (ControlCH[4] == 9) {
    DISTANCE_LAT = eeprom_read_float(32);
    DISTANCE_LNG = eeprom_read_float(36);
    flag = true;
    gpsav();
  }
  if (ControlCH[4] == 10) {
    eeprom_write_float(32, DISTANCE_LAT_BUFER);
    eeprom_write_float(36, DISTANCE_LNG_BUFER);
  }
  if (ControlCH[4] == 11) {
    DISTANCE_LAT = eeprom_read_float(40);
    DISTANCE_LNG = eeprom_read_float(44);
    flag = true;
    gpsav();
  }
  if (ControlCH[4] == 12) {
    eeprom_write_float(40, DISTANCE_LAT_BUFER);
    eeprom_write_float(44, DISTANCE_LNG_BUFER);
  }
  if (ControlCH[4] == 13) {
    DISTANCE_LAT = eeprom_read_float(48);
    DISTANCE_LNG = eeprom_read_float(52);
    flag = true;
    gpsav();
  }
  if (ControlCH[4] == 14) {
    eeprom_write_float(48, DISTANCE_LAT_BUFER);
    eeprom_write_float(52, DISTANCE_LNG_BUFER);
  }
  if (ControlCH[7] > 0) {
    digitalWrite(A1, LOW); // розмикаємо реле
  } else {
    digitalWrite(A1, HIGH); // замикаємо реле
  }
  if (ControlCH[8] > 0) {
    digitalWrite(A2, LOW); // розмикаємо реле
  } else {
    digitalWrite(A2, HIGH); // замикаємо реле
  }
}
void logikWing() {
  dataTelem.ch[9] = 210;
  dataTelem.CRC = crc16_asm((byte*)&dataTelem, sizeof(dataTelem) - 1);
}

void voltmeter() {
  temp = (analogRead(A0) * 5.0) / 1024.0; // формула для конвертування значення напруги
  volt = temp / (r2 / (r1 + r2));
  dataTelem.ch[0] = volt;
  String VoltString = "";
  byte iptr;
  volt = modf(volt, iptr);
  VoltString.concat(volt);
  VoltString.remove(0, 2);
  dataTelem.ch[8] = VoltString.toInt();
}

void SpeedMotor() {
  if (millis() - timeoutAV1 >= 300) {
    timeoutAV1 = millis();
    if (dataTelem.ch[2] > 15) {
      motorspeed++;
      if (motorspeed > limitspeed + int(ControlCH[6])) {
        motorspeed--;
        motorspeed--;
      }
      motor.write(map(motorspeed, 0, 255, 1800, 2550));
    } else {
      motorspeed++;
      if (motorspeed > 19 + int(ControlCH[6])) {
        motorspeed--;
        motorspeed--;
      }
      motor.write(map(motorspeed, 0, 255, 1800, 2550));
    }
  }
}
void turnservo() {
  int directionShip = getDirectionShip(dataTelem.ch[4], dataTelem.ch[3]); //dataTelem.ch[4] - курс з цифрового компасу, dataTelem.ch[3] - курс який потрібно тримати, dataTelem.ch[1] - GPS курс
  if (directionShip == 0) {
    //Serial.println("Ми йдемо прямо за курсом! Поворот не потрібний!");
  } else if (directionShip == -1) {
    servo1.write(map(constrain(80 + delta, 60, 100), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  } else if (directionShip == 1) {
    servo1.write(map(constrain(80 - delta, 60, 100), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  }
}
void gpsav() {
  GPS_ON = false, target_on = false, is_one1 = false;
  while (flag) {
    GPSStatys(); // (виклик функції GPS NEO-6m) dataTelem.ch[1] - GPS курс, dataTelem.ch[2] - дистанція, dataTelem.ch[3] - азимут, dataTelem.ch[5] - КМ/год (GPS може видавати дані 1 раз на секунду)
    if (millis() - timeoutAV >= 50) { // затримка в 50ms (бо велика швидкість компаса з цього ардуїнці (NANO) складно ловити пакети з GPS)
      timeoutAV = millis();
      StatCompass();
      turnservo();
    }
    LORA_SEND(); // передача даних із пультом + Цифровий компас отримуємо dataTelem.ch[4] - курс
    LORA_Telem();
    if (ControlCH[3] > 10) { // Вимкнути функцію з пульта
      flag = false; // закриваємо цикл while (flag).
      motorspeed = 0; // Обнулюємо змінну двигуна
      return;
    }
    if (dataTelem.ch[2] < 2) { // Якщо дистанція менше 2м відключаємо цикл
      if (is_one1) { // якщо прапор істина
        flag = false; // закриємо цикл функції
        motorspeed = 0; // обнулюємо зміну оборотів
        motor.write(map(0, 0, 255, 1800, 2550));
        return;
      }
      if (ControlCH[5] == 1) { // якщо на пульті увімкнено режим 1 (Заплив на точку, розвантаження та повернення на домашню точку.)
        DISTANCE_LAT = eeprom_read_float(0); // читаємо по байтах координати з енергонезалежної пам'яті
        DISTANCE_LNG = eeprom_read_float(4);
        GPSStatys();
        int flipContainersL = 180, flipContainersR = 0;
        conteinerMillis = 600;
        if (DISTANCE_LAT != eeprom_read_float(0)) {
          while (conteinerMillis > 1) {
            if (millis() - timeoutAV >= 4) {
              timeoutAV = millis();
              conteinerMillis--;
              flipContainersL--;
              flipContainersR++;
              servo2.write(map(constrain(flipContainersL, 30, 180), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
              servo3.write(map(constrain(flipContainersR, 0, 150), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
            }
            LORA_SEND();
            if (ControlCH[3] > 10) { // Вимкнути функцію з пульта
              flag = false; // закриваємо цикл while (flag).
              motorspeed = 0; // Обнулюємо змінну двигуна
              return;
            }
          }
        }
        is_one1 = true;
      } else { // Якщо ControlCH[5] != 1
        is_one1 = true; // відключаємо цикл while (flag)
      }
    } else {
      servo2.write(map(180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      servo3.write(map(0, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      limitspeed = 20; // змінити ліміт обертів SpeedMotor();
      SpeedMotor(); // виклик фун. оборотів двигуна.
    }
  }
}
int getDirectionShip(int shipAzimuth, int targetAzimuth) { // функція повороту
  delta = abs(shipAzimuth - targetAzimuth);
  if (delta == 0 | delta == 360)
    return 0;
  if (shipAzimuth > targetAzimuth)
    return delta >= 180 ? 1 : -1;
  else
    return delta >= 180 ? -1 : 1;
}
