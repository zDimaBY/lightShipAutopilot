byte motorSpeed, limitSpeed; // Для автопілота
bool GPSPacketReceived;
bool whileLoop = false;

void gpsav() {
  limitSpeed = 20; // змінити ліміт обертів SpeedMotor();
  float homeCoordinatesLat = eeprom_read_float(0), homeCoordinatesLng = eeprom_read_float(4);// читаємо по байтах координати з енергонезалежної пам'яті
  whileLoop = true;
  while (1) {
    //debagStat();
    GPSStatys(); // Якщо є пакети від GPS, то оновлюєм данні dataTelem.ch[1] - GPS курс, dataTelem.ch[2] - дистанція, dataTelem.ch[3] - заданий курс, dataTelem.ch[5] - КМ/год (GPS може видавати дані 10 раз на секунду)
    if (GPSPacketReceived) { // затримка в 100ms (надто велика швидкість компаса, ардуїнці (NANO) складно ловити пакети з GPS)
      StatCompass(); // оновити дані компаса
      turnServo(); // повернути серво по оновленим даним
    }
    RTH(); // Повернення на домашню точку
    if (controlChannel[3] > 10) { // Вимкнути функцію з пульта
      stopFunction();
      break;// закриємо цикл функції, припинемо виконувати код далі
    }
    if (dataTelem.ch[2] < 2) { // Якщо дистанція менше 2м вимикаємо функцію
      if (!whileLoop) { // якщо прапор false
        stopFunction();
        break;// закриємо цикл функції, припинемо виконувати код далі
      }

      // якщо на пульті увімкнено режим 1 (Заплив на точку, розвантаження та повернення на домашню точку) і якщо відстать більша від домашньой точки чим 6 метрів
      if (controlChannel[5] == 1 && distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, homeCoordinatesLat, homeCoordinatesLng) > 6) { //тоді..
        unloadContainers();
        returnHome();
      } else { // Якщо controlChannel[5] != 1
        whileLoop = false; // відключаємо цикл while
      }
    } else {
      SpeedMotor(); // виклик функції оборотів двигуна
    }
  }
}
void stopFunction() {
  whileLoop = false;
  motor.write(map(motorSpeed = 0, 0, 255, MIDDLE_PULSE_WIDTH, 2550));// обнулюємо зміну оборотів
}
void unloadContainers() {
  int containerMillis = 600, flipContainersL = 180, flipContainersR = 0;
  while (containerMillis > 1) {
    if (millis() - autopilotTimeout >= 4) {
      autopilotTimeout = millis();
      containerMillis--;
      servo2.write(map(constrain(flipContainersL--, 30, 180), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      servo3.write(map(constrain(flipContainersR++, 0, 150), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
    }
  }
  servo2.write(map(180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
  servo3.write(map(0, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
}

void returnHome() {
  DISTANCE_LAT = eeprom_read_float(0); // читаємо координати з енергонезалежної пам'яті
  DISTANCE_LNG = eeprom_read_float(4);
  dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG); //Оновити відстаннь
}

void SpeedMotor() {
  const int interval = 100; // інтервал в мілісекундах
  static uint32_t previousMillis;
  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    int targetSpeed = 0;
    if (dataTelem.ch[2] > 15) {
      targetSpeed = limitSpeed + controlChannel[6];
    } else {
      targetSpeed = limitSpeed;
    }
    if (motorSpeed < targetSpeed) {
      motorSpeed++;
    } else if (motorSpeed > targetSpeed) {
      motorSpeed--;
    }
    motor.write(map(motorSpeed, 0, 255, MIDDLE_PULSE_WIDTH, 2550));
    voltmeter();
  }
}
