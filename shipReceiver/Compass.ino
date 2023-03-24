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

void getHeading() {
  Vector mag = compass.readRaw();
  xValue = mag.XAxis;
  yValue = mag.YAxis;
  zValue = mag.ZAxis;
}
void StatCompass() {
  float values_from_magnetometer[3];
  getHeading(); //Читаєм компас
  values_from_magnetometer[0] = xValue;
  values_from_magnetometer[1] = yValue;
  values_from_magnetometer[2] = zValue;
  transformation(values_from_magnetometer); // корекції даних магнітометра
  vector_length_stabilasation(); // стабілізації довжини вектора магнітометра (стабілізації радіуса сфери)

  /*Serial.flush();
    Serial.print(calibratedValues[0]);
    Serial.print(",");
    Serial.print(calibratedValues[1]);
    Serial.print(",");
    Serial.print(calibratedValues[2]);
    Serial.println();*/

  float heading = atan2(calibratedValues[1], calibratedValues[0]);// Розрахувати заголовок
  // Встановити кут відхилення для вашого місця розташування та зафіксувати курс
  // Ви можете знайти своє відхилення на: http://magnetic-declination.com/
  // (+) Позитивний або (-) для негативного
  // Для Битома / Польща кут схилення 4'26E (позитивний)
  // Формула: (град + (мін / 60,0)) / (180 / PI);
  float declinationAngle = (7.0 + (20.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  // Convert to degrees
  dataTelem.ch[4] = heading * 180 / PI;

  // Output
  /*Serial.print(" Heading = ");
    Serial.print(heading);
    Serial.print(" Degress = ");
    Serial.print(dataTelem.ch[4]);
    Serial.println();*/
}
void transformation(float uncalibratedValues[3]) {
  //calibration_matrix[3][3] це є матрицею перетворення
  //replace M11, M12,..,M33 з вашими даними матриці перетворення
  double calibration_matrix[3][3] = {
    {1.087, -0.013, 0.01},
    {0.022, 1.09, 0.046},
    {0.022, -0.026, 1.247}
  };
  //bias[3] це упередженість
  //replace Bx, By, Bz з вашими даними про упередження
  double bias[3] = {
    -669.03,
    804.228,
    526.645
  };
  //calculation
  for (int i = 0; i < 3; ++i) uncalibratedValues[i] = uncalibratedValues[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibratedValues[j];
  for (int i = 0; i < 3; ++i) calibratedValues[i] = result[i];
}
void vector_length_stabilasation() { //обчислити нормальну довжину вектора
  if (scalerFlag == false) {
    getHeading();
    normal_vector_length = sqrt(calibratedValues[0] * calibratedValues[0] + calibratedValues[1] * calibratedValues[1] + calibratedValues[2] * calibratedValues[2]);
    scalerFlag = true;
  } //обчислити поточний масштабувальник
  scalerValue = normal_vector_length / sqrt(calibratedValues[0] * calibratedValues[0] + calibratedValues[1] * calibratedValues[1] + calibratedValues[2] * calibratedValues[2]);
  //застосувати поточний масштабувальник до каліброваних координат (глобальний масив calibratedValues)
  calibratedValues[0] = calibratedValues[0] * scalerValue;
  calibratedValues[1] = calibratedValues[1] * scalerValue;
  calibratedValues[2] = calibratedValues[2] * scalerValue;
}
