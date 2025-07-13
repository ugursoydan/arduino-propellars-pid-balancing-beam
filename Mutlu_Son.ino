#include <Wire.h>
#include <MPU6050.h>
#include "Kalman.h"
#include <Servo.h>

MPU6050 mpu;
Kalman kalman;
Servo motor1;
Servo motor2;

const int startButtonPin = 6;
const int stopButtonPin = 7;
bool systemRunning = false;

float setpoint = 0.0;  // Hedef: yere paralel 0 derece
float kalAngle;

float error, previousError = 0, integral = 0, derivative;
float pidOutput;

float Kp = 1.3;
float Ki = 0.00014;
float Kd = 0.9;

unsigned long lastTime = 0;

void resetMPU6050() {
  Serial.println("MPU6050 resetleniyor...");
  TWCR = 0;
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  delay(100);

  mpu.initialize();
  delay(100);

  if (!mpu.testConnection()) {
    Serial.println("MPU bağlantısı başarısız. Sistem beklemeye alındı.");
    while (1);
  } else {
    Serial.println("MPU yeniden bağlandı.");
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  mpu.initialize();

  if (!mpu.testConnection()) {
    resetMPU6050();
  }

  motor1.attach(9);
  motor2.attach(10);

  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  delay(2000);

  // Kalman filtresini başlat: başlangıç açısı ne olursa olsun referans alınsın
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float accAngle = atan2(ay, az) * 180 / PI;
  //kalman.setAngle(accAngle);  // Başlangıç açısını Kalman'a ver

  lastTime = millis();

  pinMode(stopButtonPin, INPUT_PULLUP);
  pinMode(startButtonPin, INPUT_PULLUP);

  Serial.println("Start için butona bas...");
}

void loop() {
  // Başlatma
  if (!systemRunning && digitalRead(startButtonPin) == LOW) {
    delay(50);
    if (digitalRead(startButtonPin) == LOW) {
      systemRunning = true;
      Serial.println("Sistem çalışıyor...");
      delay(500);
    }
  }

  // Durdurma
  if (systemRunning && digitalRead(stopButtonPin) == LOW) {
    delay(50);
    if (digitalRead(stopButtonPin) == LOW) {
      systemRunning = false;
      motor1.writeMicroseconds(1000);
      motor2.writeMicroseconds(1000);
      Serial.println("Sistem durduruldu.");
      delay(500);
    }
  }

  if (!systemRunning) {
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    return;
  }

  // === MPU6050 + Kalman + PID Hesaplama ===
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0 || dt > 0.5) return;

  float accAngle = atan2(ay, az) * 180 / PI;
  float gyroRate = gx / 131.0;

  // Kalman açısını hesapla (offset YOK)
  kalAngle = kalman.getAngle(accAngle, gyroRate, dt);

  // PID Hesaplama
  error = setpoint - kalAngle;
  integral += error * dt;
  derivative = (error - previousError) / dt;
  pidOutput = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // Motor PWM sinyali
  int basePower = 1200;
  int pwm1 = constrain(basePower - pidOutput, 1000, 1400);
  int pwm2 = constrain(basePower + pidOutput, 1000, 1400);

  motor1.writeMicroseconds(pwm1);
  motor2.writeMicroseconds(pwm2);

  // Seri monitör
  Serial.print("acc: "); Serial.print(accAngle);
  Serial.print(" | kalAngle: "); Serial.print(kalAngle);
  Serial.print(" | error: "); Serial.print(error);
  Serial.print(" | pidOut: "); Serial.print(pidOutput);
  Serial.print(" | pwm1: "); Serial.print(pwm1);
  Serial.print(" | pwm2: "); Serial.println(pwm2);

  delay(10);
}
