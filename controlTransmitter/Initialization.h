#include "GyverButton.h"// Ліба кнопок

#define potPinMotor A0
#define potPinRole A4
#define UpDown A3
#define potContainerServo1 A2
#define potContainerServo2 A1
GButton reverse(3); // Кнопки
GButton ok(A5); // Кнопки

//для ініцілізації ТФТ
//#define TFT_MOSI 11  // Data out
//#define TFT_SCLK 13  // Clock out
#define TFT_CS 5//це Chip Select, він використовується для вибору дисплея при передачі даних від мікроконтролера. Кожен дисплей має свій власний Chip Select пін.
#define TFT_DC 6//це Data/Command Select, він використовується для вказання, чи дані є даними для відображення на екрані, чи командою для керування дисплеєм.
#define TFT_RST 4//це Reset пін, він використовується для скидання дисплея у випадку, якщо він заблокувався або не відповідає на команди.
//#define TFT_BL_BACKLIGHT 3 //Регуліровка якркості

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void initializingLora() {
  LoRa.begin(437E6);
  LoRa.setSignalBandwidth(250E3); //defaults to 125E3. 7.8E3, 10.4E3, 15.6E3, 20.8E3 (250), 31.25E3(200), 41.7E3(150), 62.5E3(80-100), 125E3, 250E3, 500E3
  LoRa.setTxPower(20);
  //LoRa.enableInvertIQ();//використовується для налаштування інверсії IQ в LoRa-модуляції.
  //LoRa.setCodingRate4(8);//коефіцієнт кодування CR4 (Coding Rate 4) використовується для збільшення стійкості передачі даних за рахунок додаткових бітів, що додаються до кожного блоку даних, що передається.
  //LoRa.setSpreadingFactor(6);//коефіцієнт розгортання (spreading factor) в LoRa-модуляторі. Коефіцієнт розгортання визначає ширину сигналу, який передається через передавальний канал, і впливає на дальність та швидкість передачі даних.
  LoRa.setSyncWord(0x44);
}
void initializationST7735S(Adafruit_ST7735& tft){
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setTextWrap(false);//якщо текст виводиться з обгортанням(true) і не вміщується на одному рядку, він буде перенесений на наступний рядок.
  tft.cp437(true);//вказує дисплею на використання таблиці кодування CP437, що містить символи ASCII з додатковими символами, такими як лінії, криві, різні графічні символи тощо. Для Укранїнських символів.
  //tft.setFont(&FreeSans6pt8b); // вибір шрифта
}
void initializingPinOutput() {
  pinMode(potPinMotor, INPUT);
  pinMode(potPinRole, INPUT);
  pinMode(UpDown, INPUT);
  pinMode(potContainerServo1, INPUT);
  pinMode(potContainerServo2, INPUT);
}
void initializationHardwareSettings(byte *dataShip, byte *dataRemotecontrol, byte *dataAvtopilot, byte *dataControl_ch, byte *RoleLeft, byte *RoleRigch) {
  dataShip[2] = 1; // Налаштування апаратури
  dataRemotecontrol[0] = eeprom_read_byte(0);
  dataRemotecontrol[1] = eeprom_read_byte(1);
  RoleLeft = RoleLeft + eeprom_read_byte(0);
  RoleRigch = RoleRigch - eeprom_read_byte(1);
  dataAvtopilot[0] = dataControl_ch[5] = eeprom_read_byte(2);
  dataShip[0] = dataControl_ch[7] = eeprom_read_byte(3);
  dataShip[1] = dataControl_ch[8] = eeprom_read_byte(4);
}
