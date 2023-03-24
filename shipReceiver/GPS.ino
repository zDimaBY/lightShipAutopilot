void GPSStatys() {
  if (gps.available(gpsPort)) {
    gps_fix fix = gps.read(); //Робота з GPS
    if (fix.valid.location ) {// Коли у нас є місце розташування, обчислюємо.
      DISTANCE_LAT_BUFER = fix.latitude();
      DISTANCE_LNG_BUFER = fix.longitude();
      dataTelem.ch[2] = distanceBetween(DISTANCE_LAT_BUFER, DISTANCE_LNG_BUFER, DISTANCE_LAT, DISTANCE_LNG);
      dataTelem.ch[3] = courseTo(fix.latitude(), fix.longitude(), DISTANCE_LAT, DISTANCE_LNG);
    }
    if (fix.valid.heading ) {
      dataTelem.ch[1] = fix.heading(); // 0 (курс)
    }
    if (fix.valid.speed ) {
      dataTelem.ch[5] = fix.speed_kph();
    }
    if (fix.valid.hdop) {
      dataTelem.ch[6] = min(fix.hdop / 10, 999); // Використовуємо функцію min() для обмеження значення до 999
    }
    if (fix.valid.satellites ) {
      dataTelem.ch[7] = fix.satellites;
    }
    GPSPacketReceived = true;
  }
}
int distanceBetween(double lat1, double long1, double lat2, double long2) {
  // повертає відстань у метрах між двома вказаними позиціями
  // у вигляді десяткових градусів широти та довготи зі знаком. Використовує велике коло
  // Розрахунок відстані для гіпотетичної сфери радіусом 6372795 метрів.
  // Оскільки Земля не є точною кулею, похибки округлення можуть становити до 0,5%.
  // Надано Мартеном Ламерсом
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}
int courseTo(double lat1, double long1, double lat2, double long2) {
  // повертає курс у градусах (північ=0, захід=270) з позиції 1 до позиції 2,
  // обидва вказані як десяткові градуси зі знаком, широта та довгота.
  // Оскільки Земля не є точною кулею, розрахований курс може відхилятися на крихітну частку.
  // Надано Мартеном Ламерсом
  double dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) {
    a2 += TWO_PI;
  }
  return degrees(a2);
}
