# arduino-propellars-pid-balancing-beam
Self-balancing propeller beam using Arduino, MPU6050, and PID control.

This project is a self-balancing beam system controlled by Arduino using a PID algorithm and an MPU6050 IMU sensor. It maintains the beam at a level position by adjusting the **PWM signals sent to ESCs**, which control the speed of brushless DC motors. As the motors spin faster or slower, the attached propellers generate varying amounts of thrust to balance the beam.

Hardware Components:
Arduino Uno (or compatible)
2x ESC (Electronic Speed Controller)
2x Brushless DC Motors (e.g. RS2205 2300KV)
MPU6050 (Accelerometer + Gyroscope)
Power supply (LiPo battery or DC source)
Propeller, beam (330 mm 6-channel aluminum profile, propellers 5.1x3.1x3)
Jumper wires, breadboard or PCB

System Overview:
MPU6050 reads the beam’s error angle in real-time.
PID control algorithm computes the required correction.
ESCs receive PWM signals to increase/decrease motor speeds.
Motors generate thrust to balance the beam horizontally.

PID Tuning
The PID values are manually tuned. You can modify the constants:
float Kp = 1.3;
float Ki = 0.0005;
float Kd = 0.9;
