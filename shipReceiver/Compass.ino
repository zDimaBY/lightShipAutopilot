void getHeading() {
  Vector mag = compass.readRaw();
  xv = mag.XAxis;
  yv = mag.YAxis;
  zv = mag.ZAxis;
}
void StatCompass() {
  float values_from_magnetometer[3];
  getHeading(); //Читаєм компас
  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer); // корекції даних магнітометра
  vector_length_stabilasation(); // стабілізації довжини вектора магнітометра (стабілізації радіуса сфери)

  /*Serial.flush();
    Serial.print(calibrated_values[0]);
    Serial.print(",");
    Serial.print(calibrated_values[1]);
    Serial.print(",");
    Serial.print(calibrated_values[2]);
    Serial.println();*/

  float heading = atan2(calibrated_values[1], calibrated_values[0]);// Розрахувати заголовок
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
void transformation(float uncalibrated_values[3]) {
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
  for (int i = 0; i < 3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i = 0; i < 3; ++i) calibrated_values[i] = result[i];
}
void vector_length_stabilasation() { //обчислити нормальну довжину вектора
  if (scaler_flag == false) {
    getHeading();
    normal_vector_length = sqrt(calibrated_values[0] * calibrated_values[0] + calibrated_values[1] * calibrated_values[1] + calibrated_values[2] * calibrated_values[2]);
    scaler_flag = true;
  } //обчислити поточний масштабувальник
  scaler = normal_vector_length / sqrt(calibrated_values[0] * calibrated_values[0] + calibrated_values[1] * calibrated_values[1] + calibrated_values[2] * calibrated_values[2]);
  //застосувати поточний масштабувальник до каліброваних координат (глобальний масив calibrated_values)
  calibrated_values[0] = calibrated_values[0] * scaler;
  calibrated_values[1] = calibrated_values[1] * scaler;
  calibrated_values[2] = calibrated_values[2] * scaler;
}
