void OLED_textTarger(void) {
  tft.fillScreen(ST77XX_BLACK);
  TextMenu("вiдстань:", 0);
  TextMenu("GPS", 10);
  TextMenu("компас:", 20);
  TextMenu("азимут:", 30);
  TextMenu("АКБ пульта:", 40);
  TextMenu("АКБ катера:", 50);
}
void OLEDtextTransmitter(void) {
  if (millis() - TimerOLED >= 1150) {
    TimerOLED = millis();
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(48, 0);
    tft.print(map(dataControl.ch[0], 144, 255, 0, 99));
    tft.print(" ");
    tft.setCursor(87, 0);
    tft.print(Telemetry[5]);
    tft.print(" ");
    tft.setCursor(50, 10);
    tft.print(Telemetry[6]);
    tft.print(utf8rus2("cм. спут."));
    tft.print(Telemetry[7]);
    tft.print("  ");
    tft.setCursor(66, 20);
    tft.print(voltage());
    tft.setCursor(66, 30);
    tft.print(Telemetry[0]);
    tft.print(".");
    tft.print(Telemetry[8]);
    tft.setCursor(30, 40);
    tft.print(LoRa.packetRssi());
    tft.print(" dBm ");
    tft.setCursor(45, 50);
    tft.print("K ");
    tft.print(Telemetry[4]);
    tft.print(" G ");
    tft.print(Telemetry[1]);
    tft.print("    ");
  }
}
void OLEDtextAV(void) {
  if (millis() - TimerOLED >= 1000) {
    TimerOLED = millis();
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(55, 0);
    tft.print(Telemetry[2]);
    tft.print("m. ");
    tft.print(byte(Telemetry[5]));
    tft.print(utf8rus2(" КМ/ч "));
    tft.setCursor(22, 10);
    tft.print(utf8rus2("точ."));
    tft.print(Telemetry[6]);
    tft.print(utf8rus2("cм.|SAT."));
    tft.print(Telemetry[7]);
    tft.print(" ");
    tft.setCursor(41, 20);
    tft.print(" K ");
    tft.print(Telemetry[4]);
    tft.print(" G ");
    tft.print(Telemetry[1]);
    tft.print("    ");
    tft.setCursor(42, 30);
    tft.print(Telemetry[3]);
    tft.print(utf8rus2(" град.  "));
    tft.setCursor(66, 40);
    tft.print(voltage());
    tft.setCursor(66, 50);
    tft.print(Telemetry[0]);
    tft.print(".");
    tft.print(Telemetry[8]);
  }
}
void TextMenu(String text, byte numberStr) {
  tft.setCursor(0, numberStr);
  tft.print(utf8rus2(text));
}
void debag(void) {
  countLoop++;
  if (millis() - timeoutERROR >= 1000) {
    timeoutERROR = millis();
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    TextMenu("  TX ", 150);
    tft.print (countLORA);
    tft.print (" RX ");
    tft.print (countLoraRead);
    tft.print (" ");
    tft.print (countLoop);
    tft.print ("   ");
    /* Діагностична статистика */
    /*Serial.print(" byte: ");
      Serial.print(Telemetry[2]);
      Serial.print(" loop/с: ");
      Serial.print(countLoop);
      Serial.print(" LoraSend/с: ");
      Serial.print(countLORA);
      Serial.print(" LoraRead/с: ");
      Serial.println(countLoraRead);
      FOR_i(0, 17) {
      Serial.print(" ");
      Serial.print(dataControl.ch[i]);
      }*/
    countLORA = 0;
    countLoop = 0;
    countLoraRead = 0;
  }
}
String utf8rus2(String source)
{
  int i, k;
  String target;
  unsigned char n;
  char m[2] = { '0', '\0' };

  k = source.length(); i = 0;

  while (i < k) {
    n = source[i]; i++;

    if (n >= 0xC0) {
      switch (n) {
        case 0xD0: {
            n = source[i]; i++;
            if (n == 0x81) {
              n = 0xA8;
              break;
            }
            if (n >= 0x90 && n <= 0xBF) n = n + 0x30;
            break;
          }
        case 0xD1: {
            n = source[i]; i++;
            if (n == 0x91) {
              n = 0xB8;
              break;
            }
            if (n >= 0x80 && n <= 0x8F) n = n + 0x70;
            break;
          }
        case 0xC2: {
            n = source[i]; i++;
            if (n == 0xB0) break;
            n = 0xC2; i--;
            break;
          }
      }
    }
    m[0] = n; target = target + String(m);
  }
  return target;
}
