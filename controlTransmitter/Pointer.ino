const byte pixelItems = 70; //Вивід пунктів
const byte selectedItems = 7; //Вибір пунктів

void workAVcontrol(uint8_t UpAV, uint8_t DownAV) {
  okButton.tick();
  if (millis() - timerPointer >= 100) {
    timerPointer = millis();
    int UP_DOWN_PINVal = analogRead(UP_DOWN_PIN);
    if (UP_DOWN_PINVal < 200) {
      dataControl.ch[4] = UpAV;
      TextMenu("Старт автопiлота ! ", 70);
    }
    if (200 < UP_DOWN_PINVal && 800 > UP_DOWN_PINVal) {
      dataControl.ch[4] = 0;
      if (millis() - joustik >= 5000) {
        joustik = millis();
        TextMenu("                   ", 70);
      }
    }
    if (UP_DOWN_PINVal > 800) {
      dataControl.ch[4] = DownAV;
      TextMenu("Точка збережена !!", 70);
    }
  }
}
void printPointer_L(void) {
  if (millis() - timerPointer >= 100) {
    timerPointer = millis();
    int UP_DOWN_PINVal = analogRead(UP_DOWN_PIN);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (UP_DOWN_PINVal < 300) {// Якщо кнопку натиснули або утримують
      TextMenu("  ", point);
      point = constrain(point - 10, 0, pixelItems); // Рухаємо покажчик у межах дисплея
    }
    if (UP_DOWN_PINVal > 600) {
      TextMenu("  ", point);
      point = constrain(point + 10, 0, pixelItems);
    }
    TextMenu(">-", point);
  }
}
void printPointer_R(byte *arr, byte pixelCountMenu) {
  if (millis() - timerPointer >= 100) {
    timerPointer = millis();
    int UP_DOWN_PINVal = analogRead(UP_DOWN_PIN);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    if (UP_DOWN_PINVal < 300) { // Якщо кнопку натиснули або утримують
      if (pointFlag) {
        TextMenu("  ", point);
        point = constrain(point - 10, 0, pixelItems);// Рухаємо покажчик у межах дисплея
        pointer = constrain(pointer - 1, 0, selectedItems); 
      } else {
        arr[pointer]++;
      }
    }
    if (UP_DOWN_PINVal > 600) {
      if (pointFlag) {
        TextMenu("  ", point);
        point = constrain(point + 10, 0, pixelItems);// Рухаємо покажчик у межах дисплея
        pointer = constrain(pointer + 1, 0, selectedItems); 
      } else {
        arr[pointer]--;
      }
    }
    if (pointFlag) {
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
  if (pointFlag) {
    TextMenu(">-", pointer);
  } else {
    tft.setCursor(120, pointer);
    tft.print("<");
  }
}
