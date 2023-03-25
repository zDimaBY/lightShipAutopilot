/*
Скетч використовує 25218 байтів (82%) місця зберігання для програм. Межа 30720 байтів.
Глобальні змінні використовують 1440 байтів (70%) динамічної пам’яті,  залишаючи 608 байтів для локальних змінних. Межа 2048 байтів.
*/
// Макроси для циклів
#define FOR_i(from, to) for (int i = (from); i < (to); i++)
#define FOR_iTen(from, to) for (int i = (from), a = (from); i < (to); i=i+10, a++)

#include <SPI.h> // Підключаємо бібліотеку для роботи з SPI-інтерфейсом
#include <LoRa.h> // Підключаємо бібліотеку для роботи з модулем LoRa
#include <avr/eeprom.h> // Підключаємо бібліотеку для роботи з енергонезалежною пам'яттю
#include <Adafruit_GFX.h>// Основна бібліотека для роботи з графікою
#include <Adafruit_ST7735.h>// Бібліотека для роботи з конкретним апаратним забезпеченням
#include "Initialization.h" // Файл ініціалізації

byte autopilotData[5], shipData[5], remoteControlData[5];// Масиви даних для збереження даних налаштувань


byte motorLimit = 155, minDrive = 144, maxDrive = 255;// Обмеження обертів двигуна
int leftRole = 0, rightRole = 1023;// Для обмеження ROLL, корекції керування

bool pointFlag = true, is_one0 = true; // up OLED
byte point = 0; // Змінна покажчика у пікселях
byte pointer = 0; // Змінна покажчика у масивах
byte countLoraRead, countLORA; // лічильники пакетів даних

// Опір резисторів та таймери
const float r1 = 101000.0;
const float r2 = 10000.0;
unsigned long timeoutERROR, TimerOLED, joustik, timerPointer, timeoutBeginPacket, countLoop;

struct TX_DATA {
  byte ch[10];
  byte CRC;
} dataControl;

struct RX_DATA {
  int ch[10];
  byte CRC;
} dataTelem;

int Telemetry[16];

void setup() {
  //Serial.begin(115200);
  initST7735S(tft);
  initHardwareSettings(shipData, remoteControlData, autopilotData, dataControl.ch, leftRole, rightRole);
  initializingPinOutput();
  initializingLora();
  LoRa.onReceive(onReceive); //Коли модуль LoRa отримує дані, він викликає функцію зворотнього виклику onReceive, яка передає отримані дані у вигляді масиву байтів.
  transmitter();
}
void loop() {//Головне меню
  if (is_one0) {
    tft.fillScreen(ST77XX_BLACK);
    is_one0 = false;
    point = 0; // Змінна покажчика у пікселях
    TextMenu("  РЕЖИМ КЕРУВАННЯ", 0);
    TextMenu("  GPS Автопiлот", 10);
    TextMenu("  налаштування", 20);
    TextMenu("  ___________________", 30);
    TextMenu("  Катер Олександра", 40);
    TextMenu("           Мельничука", 50);
    TextMenu("  ___________________", 60);
  }
  okButton.tick(); //Опитування кнопок
  printPointer_L();  // Показати стрілку
  if (okButton.isClick()) {   // Натискання на ОК - перехід до пункту меню
    switch (point) {  // За номером покажчиків маємо вкладені функції (можна вкладені меню)
      case 0: is_one0 = true; transmitter(); break;  // Після натискання на ОК при наведенні на 0й пункт викликати функцію
      case 10: is_one0 = true; GPSautopilot(); break;
      case 20: is_one0 = true; Settings(); break;
    }
  }
  debag();
}

float voltage(void) {
  return (analogRead(A7) * 5.0) / 1024 / (r2 / (r1 + r2));// формула для конвертування значення напруги
}
void transmitter(void) {
  tft.fillScreen(ST77XX_BLACK);
  is_one0 = true;
  TextMenu("Дросель:", 0);
  tft.setCursor(70, 0);
  tft.print("%>");
  tft.setCursor(103, 0);
  tft.print(utf8rus2("КМ/ч"));
  TextMenu("GPS точ.", 10);
  TextMenu("АКБ пульта:", 20);
  TextMenu("АКБ катера:", 30);
  TextMenu("RSSI:", 40);
  TextMenu("компас:", 50);
  while (1) {
    LORA_SEND();
    OLEDtextTransmitter();
    okButton.tick();
    reverseButton.tick();
    if (reverseButton.isClick()) {
      minDrive = 144;
      maxDrive = 255;
    }
    if (reverseButton.isHolded()) {
      minDrive = 144;
      maxDrive = 0;
    }
    if (okButton.isClick()) return;
  }
}

void GPSautopilot(void) {
  while (1) {
    if (is_one0) {
      tft.fillScreen(ST77XX_BLACK);
      is_one0 = false;
      point = 0; // Змінна покажчика у пікселях
      TextMenu("  Домашня точка", 0);
      TextMenu("  точка 1", 10);
      TextMenu("  точка 2", 20);
      TextMenu("  точка 3", 30);
      TextMenu("  точка 4", 40);
      TextMenu("  точка 5", 50);
      TextMenu("  точка 6", 60);
      TextMenu("  назад у меню <---", 70);
    }
    okButton.tick(); //Опитування кнопок
    printPointer_L();  // Показати стрілку
    if (okButton.isClick()) {   // Натискання на OK - Перехід до пункту меню
      switch (point) {  // За кількістю знаків ми вклали функції (ви можете вкладені меню)
        case 0: loopAutopilot(1, 2); is_one0 = true; break;  // Натиснувши на OK, коли керується 0 точкою, зателефонуйте на функцію (в даному випадку домашня точка)
        case 10: loopAutopilot(3, 4); is_one0 = true; break;
        case 20: loopAutopilot(5, 6); is_one0 = true; break;
        case 30: loopAutopilot(7, 8); is_one0 = true; break;
        case 40: loopAutopilot(9, 10); is_one0 = true; break;
        case 50: loopAutopilot(11, 12); is_one0 = true; break;
        case 60: loopAutopilot(13, 14); is_one0 = true; break;
        case 70: is_one0 = true; return; break;
      }
    }
  }
}
void loopAutopilot(byte oneAV, byte twoAV) {
  OLED_textTarger();
  while (1) {
    LORA_SEND();
    OLEDtextAV();
    debag();
    workAVcontrol(oneAV, twoAV);
    if (okButton.isClick()) return;
  }
}
void Settings(void) {
  while (1) {
    if (is_one0) {
      is_one0 = false;
      point = 0; // Змінна покажчика у пікселях
      tft.fillScreen(ST77XX_BLACK);
      TextMenu("  автопiлот", 0);
      TextMenu("  катер", 10);
      TextMenu("  пульт", 20);
      TextMenu("  (0 - OFF, 1 - ON)", 60);
      TextMenu("  назад у меню <---", 70);
    }
    okButton.tick(); //Опитування кнопок
    printPointer_L();  // Показати стрілку
    if (okButton.isClick()) {   // Нажатие на ОК - переход в пункт меню
      switch (point) {  // По номеру указателей располагаем вложенные функции (можно вложенные меню)
        case 0: is_one0 = true; SettingsAvtopilot(); break;
        case 10: is_one0 = true; SettingsShip(); break;
        case 20: is_one0 = true; SettingsRemotecontrol(); break;
        case 30: is_one0 = true; return; break;
        case 40: is_one0 = true; return; break;
        case 50: is_one0 = true; return; break;
        case 70: is_one0 = true; return; break;
      }
    }
  }
}
void SettingsAvtopilot(void) {
  while (1) {
    if (is_one0) {
      is_one0 = false;
      point = 0; // Змінна покажчика у пікселях
      pointer = 0; // Змінна покажчика у масивах
      tft.fillScreen(ST77XX_BLACK);
      TextMenu("  режим автопiло:", 0);
      TextMenu("  обр. автопiлот:", 10);
      TextMenu("  (0 - OFF, 1 - ON)", 60);
      TextMenu("  назад у меню <---", 70);
    }
    printPointer_R(autopilotData, 20); //Вказуєм вказівник на масив налаштувань, та вивід кількість налаштувань у пікселях.
    if (okButton.isClick()) {   // Нажатие на ОК - переход в пункт меню
      switch (pointer) {  // По номеру указателей располагаем вложенные функции (можно вложенные меню)
        case 0: pointFlag = !pointFlag; break;
        case 1: pointFlag = !pointFlag; break;
        case 7: is_one0 = true;
          eeprom_write_byte(3, autopilotData[0]);
          return;
          break;
      }
    }
    autopilotData[0] = constrain(autopilotData[0], 0, 1);//обмежуєм налаштування
    dataControl.ch[5] = autopilotData[0]; //збережемо для відправки на керуючу модель
    dataControl.ch[6] = autopilotData[1]; //збережемо для відправки на керуючу модель
    okButton.tick();
  }
}
void SettingsShip(void) {
  while (1) {
    if (is_one0) {
      is_one0 = false;
      point = 0; // Змінна покажчика у пікселях
      pointer = 0; // Змінна покажчика у масивах
      tft.fillScreen(ST77XX_BLACK);
      TextMenu("    вкл.габарити:", 0);
      TextMenu("   влючення фари:", 10);
      TextMenu("  обмеж. швидкос:", 20);
      TextMenu("  (0 - OFF, 1 - ON)", 60);
      TextMenu("  назад у меню <---", 70);
    }
    printPointer_R(shipData, 30); //Вказуєм вказівник на масив налаштувань, та вивід кількість налаштувань у пікселях.
    if (okButton.isClick()) {   // Натискання на ОК - перехід до пункту меню
      switch (pointer) {  // За номером покажчиків маємо вкладені функції (можна вкладені меню)
        case 0: pointFlag = !pointFlag; break;
        case 1: pointFlag = !pointFlag; break;
        case 2: pointFlag = !pointFlag; break;
        case 7: is_one0 = true;
          eeprom_write_byte(4, shipData[0]);
          eeprom_write_byte(5, shipData[1]);
          return;
          break;
      }
    }
    shipData[0] = constrain(shipData[0], 0, 1);//обмежуєм налаштування
    dataControl.ch[7] = shipData[0];//збережемо для відправки на керуючу модель

    shipData[1] = constrain(shipData[1], 0, 1);//обмежуєм налаштування
    dataControl.ch[8] = shipData[1];//збережемо для відправки на керуючу модель
    
    shipData[2] = constrain(shipData[2], 0, 1);//обмежуєм налаштування
    motorLimit = (shipData[2] == 0) ? 255 : 155;//якщо значення shipData[2] дорівнює 1, то motorLimit отримує значення 155, а якщо ні, то отримує значення 255
    okButton.tick();
  }
}
void SettingsRemotecontrol(void) {
  while (1) {
    if (is_one0) {
      is_one0 = false;
      point = 0; // Змінна покажчика у пікселях
      pointer = 0; // Змінна покажчика у масивах
      tft.fillScreen(ST77XX_BLACK);
      TextMenu("  +до лiвого рол:", 0);
      TextMenu("  -вiд прав. рол:", 10);
      TextMenu("  яркiсть екрана:", 20);
      TextMenu("  (0 - OFF, 1 - ON)", 60);
      TextMenu("  назад у меню <---", 70);
    }
    printPointer_R(remoteControlData, 30); //Вказуєм вказівник на масив налаштувань, та вивід кількість налаштувань у пікселях.
    if (okButton.isClick()) { // Нажатие на ОК - переход в пункт меню
      switch (pointer) { // По номеру указателей располагаем вложенные функции (можно вложенные меню)
        case 0: pointFlag = !pointFlag; break;
        case 1: pointFlag = !pointFlag; break;
        case 2: pointFlag = !pointFlag; break;
        case 7: is_one0 = true;
          leftRole = 0, rightRole = 1023;
          eeprom_write_byte(0, remoteControlData[0]);
          eeprom_write_byte(1, remoteControlData[1]);
          eeprom_write_byte(2, remoteControlData[2]);// яскравість дисплею
          analogWrite(TFT_BL_BACKLIGHT, remoteControlData[2]);
          leftRole = leftRole + eeprom_read_byte(0);
          rightRole = rightRole - eeprom_read_byte(1);
          return;
          break;
      }
    }
    remoteControlData[2] = constrain(remoteControlData[2], 0, 190);//обмежуєм налаштування
    okButton.tick();
  }
}