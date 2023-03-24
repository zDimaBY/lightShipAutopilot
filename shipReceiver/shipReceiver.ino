#define FOR_i(from, to) for (int i = (from); i < (to); i++)

#define GPS_SERIAL_PORT_NAME "AltSoftSerial"

#include <SPI.h>  // Підключаємо бібліотеку для роботи з SPI-інтерфейсом
#include <LoRa.h>
#include <avr/eeprom.h> // Енергонезалежна пам'ять
#include <NMEAGPS.h> // (виклик функції GPS NEO-6m) dataTelem.ch[1] - GPS курс, dataTelem.ch[2] - дистанція, dataTelem.ch[3] - азимут, dataTelem.ch[5] - КМ/год (GPS може видавати дані 10 раз на секунду)
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

unsigned int communicationTimeout = 500; // інтервал розриву зв'язку
unsigned int returnTimeout = 60000; // інтервал повернення після розриву зв'язку
unsigned int lastCommunicationTime = 0;

float voltage; //Вольт метр
const float r1 = 101500.0; //опір резистора r1
const float r2 = 20000.0; // опір резистора r2

byte motorSpeed, limitspeed; // Для автопілота
unsigned long autopilotTimeout;
bool loraTelemetryBoolean = false;
bool whileLoop = false;

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

byte controlChannel[16];

float xValue, yValue, zValue; //-------------------- КОМПАС!!!
//calibratedValues[3] це глобальний масив, куди будуть розміщені калібровані дані
//calibratedValues[3]: [0]=Xc, [1]=Yc, [2]=Zc
float calibratedValues[3];
//transformation(float uncalibratedValues[3]) це функція корекції даних магнітометра
//uncalibratedValues[3] це масив даних некаліброваного магнітометра
//uncalibratedValues[3]: [0]=Xnc, [1]=Ync, [2]=Znc
//vector_length_stabilasation() - – функція стабілізації довжини вектора магнітометра (стабілізації радіуса сфери)
float scalerValue;
boolean scalerFlag = false;
float normal_vector_length;

// PID constants
const int Kp = 100;
const float Ki = 0.2;
const int Kd = 400;

// variables
int error, previous_error = 0;
int integral = 0, derivative;

void debugPIDOutput(int output) {
  Serial.print("error: ");
  Serial.print(error);
  Serial.print(" integral: ");
  Serial.print(integral);
  Serial.print(" derivative: ");
  Serial.print(derivative);
  Serial.print(" output: ");
  Serial.println(output);
}
void turnServo() {
  error = dataTelem.ch[3] - dataTelem.ch[4];//розрахування помилки

  // вираховуєм PID output
  integral += error * 10;
  integral = constrain(integral, -600, 600);
  derivative = error - previous_error;
  int output = (Kp * error + Ki * integral + Kd * derivative) / 100;

  servo1.write(map(constrain(80 - output, 60, 100), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));// update servo position
  //debugPIDOutput(output);
  previous_error = error;// update previous error
}

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
  Serial.begin(115200); //відкриваємо порт для зв'язку з ПК
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
  if (millis() - autopilotTimeout >= 200) { // затримка в 200ms (бо велика швидкість компаса з цього ардуїнці (NANO) складно ловити пакети з GPS)
    autopilotTimeout = millis();
    StatCompass();
    voltmeter();
  }
  //debagStat();
  RTH();
  LORA_Telem();
  switch (controlChannel[4]) {
    case 1:
      DISTANCE_LAT = eeprom_read_float(0);
      DISTANCE_LNG = eeprom_read_float(4);
      gpsav();
      break;
    case 2:
      eeprom_write_float(0, DISTANCE_LAT_BUFER);
      eeprom_write_float(4, DISTANCE_LNG_BUFER);
      break;
    case 3:
      DISTANCE_LAT = eeprom_read_float(8);
      DISTANCE_LNG = eeprom_read_float(12);
      gpsav();
      break;
    case 4:
      eeprom_write_float(8, DISTANCE_LAT_BUFER);
      eeprom_write_float(12, DISTANCE_LNG_BUFER);
      break;
    case 5:
      DISTANCE_LAT = eeprom_read_float(16);
      DISTANCE_LNG = eeprom_read_float(20);
      gpsav();
      break;
    case 6:
      eeprom_write_float(16, DISTANCE_LAT_BUFER);
      eeprom_write_float(20, DISTANCE_LNG_BUFER);
      break;
    case 7:
      DISTANCE_LAT = eeprom_read_float(24);
      DISTANCE_LNG = eeprom_read_float(28);
      gpsav();
      break;
    case 8:
      eeprom_write_float(24, DISTANCE_LAT_BUFER);
      eeprom_write_float(28, DISTANCE_LNG_BUFER);
      break;
    case 9:
      DISTANCE_LAT = eeprom_read_float(32);
      DISTANCE_LNG = eeprom_read_float(36);
      gpsav();
      break;
    case 10:
      eeprom_write_float(32, DISTANCE_LAT_BUFER);
      eeprom_write_float(36, DISTANCE_LNG_BUFER);
      break;
    case 11:
      DISTANCE_LAT = eeprom_read_float(40);
      DISTANCE_LNG = eeprom_read_float(44);
      gpsav();
      break;
    case 12:
      eeprom_write_float(40, DISTANCE_LAT_BUFER);
      eeprom_write_float(44, DISTANCE_LNG_BUFER);
      break;
    case 13:
      DISTANCE_LAT = eeprom_read_float(48);
      DISTANCE_LNG = eeprom_read_float(52);
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
void logikWing() {
  dataTelem.ch[9] = 210;
  dataTelem.CRC = crc16_asm((byte*)&dataTelem, sizeof(dataTelem) - 1);
}

void voltmeter() {
  float voltage = (analogRead(A0) * 5.0) / 1024.0 / (r2 / (r1 + r2));
  dataTelem.ch[0] = voltage;
  double integerPart;
  double decimalPart = modf(voltage, &integerPart);
  int decimalValue = decimalPart * 100;
  dataTelem.ch[8] = decimalValue;
}

void SpeedMotor() {
  const uint32_t interval = 300; // інтервал в мілісекундах
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    int targetSpeed = 0;
    if (dataTelem.ch[2] > 15) {
      targetSpeed = limitspeed + int(controlChannel[6]);
    } else {
      targetSpeed = 19 + int(controlChannel[6]);
    }
    if (motorSpeed < targetSpeed) {
      motorSpeed++;
    } else if (motorSpeed > targetSpeed) {
      motorSpeed--;
    }
    motor.write(map(motorSpeed, 0, 255, 1800, 2550));
  }
}

void gpsav() {
  float homeCoordinatesLat = eeprom_read_float(0), homeCoordinatesLng = eeprom_read_float(4);// читаємо по байтах координати з енергонезалежної пам'яті
  int containerMillis, flipContainersL = 180, flipContainersR = 0;
  whileLoop = true;
  while (1) {
    //debagStat();
    GPSStatys(); // Якщо є пакети від GPS, то оновлюєм данні dataTelem.ch[1] - GPS курс, dataTelem.ch[2] - дистанція, dataTelem.ch[3] - заданий курс, dataTelem.ch[5] - КМ/год (GPS може видавати дані 10 раз на секунду)
    if (millis() - autopilotTimeout >= 100) { // затримка в 100ms (надто велика швидкість компаса, ардуїнці (NANO) складно ловити пакети з GPS)
      autopilotTimeout = millis();
      StatCompass(); // оновити дані компаса
      turnServo(); // повернути серво по оновленим данним
    }
    RTH(); // Повернення на домашню точку
    LORA_Telem();//Якщо дані прийшли, то відправимо телеметрію
    if (controlChannel[3] > 10) { // Вимкнути функцію з пульта
      whileLoop = false;
      motor.write(map(motorSpeed = 0, 0, 255, 1800, 2550));// обнулюємо зміну оборотів
      break;// закриємо цикл функції, припинемо виконувати код далі
    }
    if (dataTelem.ch[2] < -2) { // Якщо дистанція менше 2м вимикаємо функцію
      if (!whileLoop) { // якщо прапор false
        whileLoop = false;
        motor.write(map(motorSpeed = 0, 0, 255, 1800, 2550));// обнулюємо зміну оборотів
        break;// закриємо цикл функції, припинемо виконувати код далі
      }
      if (controlChannel[5] == 1) { // якщо на пульті увімкнено режим 1 (Заплив на точку, розвантаження та повернення на домашню точку.)
        DISTANCE_LAT = homeCoordinatesLat; // читаємо координати з енергонезалежної пам'яті
        DISTANCE_LNG = homeCoordinatesLng;
        GPSStatys(); //Оновити геопозицію
        containerMillis = 600;
        if (DISTANCE_LAT != eeprom_read_float(0)) {
          while (containerMillis > 1) {
            if (millis() - autopilotTimeout >= 4) {
              autopilotTimeout = millis();
              containerMillis--;
              flipContainersL--;
              flipContainersR++;
              servo2.write(map(constrain(flipContainersL, 30, 180), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
              servo3.write(map(constrain(flipContainersR, 0, 150), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
            }
            RTH();
            if (controlChannel[3] > 10) { // Вимкнути функцію з пульта
              whileLoop = false;
              motor.write(map(motorSpeed = 0, 0, 255, 1800, 2550));// обнулюємо зміну оборотів
              break;// закриємо цикл функції, припинемо виконувати код далі
            }
          }
        }
      } else { // Якщо controlChannel[5] != 1
        whileLoop = false; // відключаємо цикл while
      }
    } else {
      servo2.write(map(180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      servo3.write(map(0, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      limitspeed = 20; // змінити ліміт обертів SpeedMotor();
      //SpeedMotor(); // виклик функції оборотів двигуна
    }
  }
}
