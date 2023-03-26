// PID constants
const int Kp = 100;
const float Ki = 0.2;
const int Kd = 800;

// variables
int error, previous_error = 0;
int integral = 0, derivative;

void debugPIDOutput(int output) {
  Serial.print("error: ");
  Serial.print(error);
  Serial.print(" integral: ");
  Serial.print(integral);
  Serial.print(" derivative: ");
  Serial.print(derivative);
  Serial.print(" output: ");
  Serial.println(output);
}
void turnServo() {//Функція turnServo() виконує поворот сервоприводу за допомогою розрахунку помилки та PID-контролера.
  error = dataTelem.ch[3] - dataTelem.ch[4];//розрахування помилки dataTelem.ch[1] - GPS курс, dataTelem.ch[3] - заданий курс, dataTelem.ch[4] - компас курс
  integral += error * 20;// Обчислення інтегральної складової з контролем переповнення
  integral = constrain(integral, -600, 600);// Обмеження інтегральної складової

  derivative = error - previous_error;// Обчислення похідної складової
  int output = (Kp * error + Ki * integral + Kd * derivative) / 100;// Обчислення вихідного сигналу з ПІД-регулятора
  servo1.write(map(constrain(80 - output, 60, 100), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));// Обмеження вихідного сигналу і запис значення на сервопривід
  previous_error = error;// Запис поточної помилки для використання як попередньої при наступному виклику функції
}
