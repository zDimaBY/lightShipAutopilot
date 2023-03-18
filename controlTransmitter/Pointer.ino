void workAVcontrol(uint8_t UpAV, uint8_t DownAV) {
  ok.tick();
  if (millis() - timerPointer >= 100) {
    timerPointer = millis();
    int upDownVal = analogRead(UpDown);
    if (upDownVal < 200) {
      dataControl.ch[4] = UpAV;
      TextMenu("Старт автопiлота ! ", 70);
    }
    if (200 < upDownVal && 800 > upDownVal) {
      dataControl.ch[4] = 0;
      if (millis() - joustik >= 5000) {
        joustik = millis();
        TextMenu("                   ", 70);
      }
    }
    if (upDownVal > 800) {
      dataControl.ch[4] = DownAV;
      TextMenu("Точка збережена !!", 70);
    }
  }
}
void printPointer_L(void) {
  if (millis() - timerPointer >= 100) {
    timerPointer = millis();
    int upDownVal = analogRead(UpDown);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (upDownVal < 300) {// Якщо кнопку натиснули або утримують
      TextMenu("  ", point);
      point = constrain(point - 10, 0, ITEMS); // Рухаємо покажчик у межах дисплея
    }
    if (upDownVal > 600) {
      TextMenu("  ", point);
      point = constrain(point + 10, 0, ITEMS);
    }
    TextMenu(">-", point);
  }
}
void printPointer_R(byte *arr, byte pixelCountMenu) {
  if (millis() - timerPointer >= 100) {
    timerPointer = millis();
    int upDownVal = analogRead(UpDown);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (upDownVal < 300) { // Якщо кнопку натиснули або утримують
      if (flagPoint) {
        TextMenu("  ", point);
        point = constrain(point - 10, 0, ITEMS);// Рухаємо покажчик у межах дисплея
        pointer = constrain(pointer - 1, 0, pointITEMS); 
      } else {
        arr[pointer]++;
      }
    }
    if (upDownVal > 600) {
      if (flagPoint) {
        TextMenu("  ", point);
        point = constrain(point + 10, 0, ITEMS);// Рухаємо покажчик у межах дисплея
        pointer = constrain(pointer + 1, 0, pointITEMS); 
      } else {
        arr[pointer]--;
      }
    }
    if (flagPoint) {
      TextMenu(">-", point);
    } else {
      tft.setCursor(120, point);
      tft.print("<");
    }
    FOR_iTen(0, pixelCountMenu) {
      tft.setCursor(103, i);
      tft.print (arr[a]);
      tft.print ("   ");
    }
  }
}
void printPointer(uint8_t pointer) {
  if (flagPoint) {
    TextMenu(">-", pointer);
  } else {
    tft.setCursor(120, pointer);
    tft.print("<");
  }
}
