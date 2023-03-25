#include "GyverButton.h"// Ліба кнопок

#define MOTOR_POT_PIN A0
#define ROLE_POT_PIN A4
#define UP_DOWN_PIN A3
#define SERVO1_POT_PIN A2
#define SERVO2_POT_PIN A1

GButton reverseButton(7);
GButton okButton(A5);

//для ініцілізації ТФТ
//#define TFT_MOSI 11  // Data out
//#define TFT_SCLK 13  // Clock out
#define TFT_CS 5//це Chip Select, він використовується для вибору дисплея при передачі даних від мікроконтролера. Кожен дисплей має свій власний Chip Select пін.
#define TFT_DC 6//це Data/Command Select, він використовується для вказання, чи дані є даними для відображення на екрані, чи командою для керування дисплеєм.
#define TFT_RST 4//це Reset пін, він використовується для скидання дисплея у випадку, якщо він заблокувався або не відповідає на команди.
#define TFT_BL_BACKLIGHT 3 //Регуліровка яркості

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void initializingLora() {
  LoRa.begin(437E6);//запустим на цій частоті
  LoRa.setSignalBandwidth(125E3); //defaults to 125E3. 7.8E3, 10.4E3, 15.6E3, 20.8E3 (250), 31.25E3(200), 41.7E3(150), 62.5E3(80-100), 125E3, 250E3, 500E3
  LoRa.setTxPower(20);// потужність передавання
  //LoRa.enableInvertIQ();//використовується для налаштування інверсії IQ в LoRa-модуляції.
  //LoRa.setCodingRate4(8);//коефіцієнт кодування CR4 (Coding Rate 4) використовується для збільшення стійкості передачі даних за рахунок додаткових бітів, що додаються до кожного блоку даних, що передається.
  //LoRa.setSpreadingFactor(6);//коефіцієнт розгортання (spreading factor) в LoRa-модуляторі. Коефіцієнт розгортання визначає ширину сигналу, який передається через передавальний канал, і впливає на дальність та швидкість передачі даних.
  LoRa.setSyncWord(0x44);
}
void initST7735S(Adafruit_ST7735& tft) {
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setTextWrap(false);//якщо текст виводиться з обгортанням(true) і не вміщується на одному рядку, він буде перенесений на наступний рядок.
  tft.cp437(true);//вказує дисплею на використання таблиці кодування CP437, що містить символи ASCII з додатковими символами, такими як лінії, криві, різні графічні символи тощо. Для Укранїнських символів.
  //tft.setFont(&FreeSans6pt8b); // вибір шрифта
}
void initializingPinOutput() {
  pinMode(MOTOR_POT_PIN, INPUT);
  pinMode(ROLE_POT_PIN, INPUT);
  pinMode(UP_DOWN_PIN, INPUT);
  pinMode(SERVO1_POT_PIN, INPUT);
  pinMode(SERVO2_POT_PIN, INPUT);
  pinMode(TFT_BL_BACKLIGHT, OUTPUT);
}
void initHardwareSettings(byte *shipData, byte *remoteControlData, byte *autopilotData, byte *dataControl_ch, byte *leftRole, byte *rightRole) {
  shipData[2] = 1; // Налаштування апаратури
  remoteControlData[0] = eeprom_read_byte(0);
  remoteControlData[1] = eeprom_read_byte(1);
  analogWrite(TFT_BL_BACKLIGHT, remoteControlData[2] = eeprom_read_byte(2));
  leftRole = leftRole + eeprom_read_byte(0);
  rightRole = rightRole - eeprom_read_byte(1);
  autopilotData[0] = dataControl_ch[5] = eeprom_read_byte(3);
  shipData[0] = dataControl_ch[7] = eeprom_read_byte(4);
  shipData[1] = dataControl_ch[8] = eeprom_read_byte(5);
}