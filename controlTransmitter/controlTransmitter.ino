/*
Скетч використовує 24960 байтів (81%) місця зберігання для програм. Межа 30720 байтів.
Глобальні змінні використовують 1410 байтів (68%) динамічної пам’яті,  залишаючи 638 байтів для локальних змінних. Межа 2048 байтів.
*/

#define FOR_i(from, to) for (int i = (from); i < (to); i++)
#define FOR_iTen(from, to) for (int i = (from), a = (from); i < (to); i=i+10, a++)

#include <SPI.h>
#include <LoRa.h>
#include <avr/eeprom.h>// Енергонезависимая память
#include <Adafruit_GFX.h>// Core graphics library
#include <Adafruit_ST7735.h>// Hardware-specific library
#include "Initialization.h" //Підключаємо файл ініціалізацій

byte dataAvtopilot[5], dataShip[5], dataRemotecontrol[5]; // Общее кол во пунктов
byte limitmotor = 155, DriveMin = 144, DriveMax = 255; // Ограничение оборотов двигателя
int RoleLeft = 0, RoleRigch = 1023;
bool flagPoint = true, is_one0 = true;// up OLED
byte ITEMS = 70; //Вивід пунктів
byte pointITEMS = 7; //Вибір пунктів
byte point = 0; // Змінна покажчика у пікселях
byte pointer = 0; // Змінна покажчика у масивах

float r1 = 101000.0; //сопротивление резистора r1
float r2 = 10000.0; //сопротивление резистора r2

unsigned long timeoutERROR, TimerOLED, joustik, timerPointer; //таймер

unsigned long timeoutBeginPacket, countLoop;
byte countLoraRead, countLORA;

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
  initializationST7735S(tft);
  initializationHardwareSettings(dataShip, dataRemotecontrol, dataAvtopilot, dataControl.ch, RoleLeft, RoleRigch);
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
  ok.tick(); //Опитування кнопок
  printPointer_L();  // Показати стрілку
  if (ok.isClick()) {   // Натискання на ОК - перехід до пункту меню
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
    ok.tick();
    reverse.tick();
    if (reverse.isClick()) {
      DriveMin = 144;
      DriveMax = 255;
    }
    if (reverse.isHolded()) {
      DriveMin = 144;
      DriveMax = 0;
    }
    if (ok.isClick()) return;
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
    ok.tick(); //Опитування кнопок
    printPointer_L();  // Показати стрілку
    if (ok.isClick()) {   // Натискання на OK - Перехід до пункту меню
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
    if (ok.isClick()) return;
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
    ok.tick(); //Опитування кнопок
    printPointer_L();  // Показати стрілку
    if (ok.isClick()) {   // Нажатие на ОК - переход в пункт меню
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
    printPointer_R(dataAvtopilot, 20); //Вказуєм вказівник на масив налаштувань, та вивід кількість налаштувань у пікселях.
    if (ok.isClick()) {   // Нажатие на ОК - переход в пункт меню
      switch (pointer) {  // По номеру указателей располагаем вложенные функции (можно вложенные меню)
        case 0: flagPoint = !flagPoint; break;
        case 1: flagPoint = !flagPoint; break;
        case 7: is_one0 = true;
          eeprom_write_byte(2, dataAvtopilot[0]);
          return;
          break;
      }
    }
    if (dataAvtopilot[0] == 2) {
      dataAvtopilot[0] = 1;
    } else if (dataAvtopilot[0] > 250) {
      dataAvtopilot[0] = 0;
    }
    dataControl.ch[5] = dataAvtopilot[0];
    dataControl.ch[6] = dataAvtopilot[1];
    ok.tick();
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
    printPointer_R(dataShip, 30); //Вказуєм вказівник на масив налаштувань, та вивід кількість налаштувань у пікселях.
    if (ok.isClick()) {   // Натискання на ОК - перехід до пункту меню
      switch (pointer) {  // За номером покажчиків маємо вкладені функції (можна вкладені меню)
        case 0: flagPoint = !flagPoint; break;
        case 1: flagPoint = !flagPoint; break;
        case 2: flagPoint = !flagPoint; break;
        case 7: is_one0 = true;
          eeprom_write_byte(3, dataShip[0]);
          eeprom_write_byte(4, dataShip[1]);
          return;
          break;
      }
    }
    if (dataShip[0] == 2) {
      dataShip[0] = 1;
    } else if (dataShip[0] > 200) {
      dataShip[0] = 0;
    }
    dataControl.ch[7] = dataShip[0];
    if (dataShip[1] == 2) {
      dataShip[1] = 1;
    } else if (dataShip[1] > 200) {
      dataShip[1] = 0;
    }
    dataControl.ch[8] = dataShip[1];
    if (dataShip[2] == 0) {
      limitmotor = 255;
    } else if (dataShip[2] == 1) {
      limitmotor = 155;
    }
    if (dataShip[2] == 2) {
      dataShip[2] = 1;
    } else if (dataShip[2] > 250) {
      dataShip[2] = 0;
    }
    ok.tick();
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
      TextMenu("  (0 - OFF, 1 - ON)", 60);
      TextMenu("  назад у меню <---", 70);
    }
    printPointer_R(dataRemotecontrol, 20); //Вказуєм вказівник на масив налаштувань, та вивід кількість налаштувань у пікселях.
    if (ok.isClick()) { // Нажатие на ОК - переход в пункт меню
      switch (pointer) { // По номеру указателей располагаем вложенные функции (можно вложенные меню)
        case 0: flagPoint = !flagPoint; break;
        case 1: flagPoint = !flagPoint; break;
        case 7: is_one0 = true;
          RoleLeft = 0, RoleRigch = 1023;
          eeprom_write_byte(0, dataRemotecontrol[0]);
          eeprom_write_byte(1, dataRemotecontrol[1]);
          RoleLeft = RoleLeft + eeprom_read_byte(0);
          RoleRigch = RoleRigch - eeprom_read_byte(1);
          return;
          break;
      }
    }
    ok.tick();
  }
}
