#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

Servo myServo;
const int servoPin = 9;
const int minPulse = 500;
const int maxPulse = 2500;

MPU6050 MPU;

float theta_est_prev = 0.0;
float theta_est = 0.0;
float Ts = 3;           // 10 ms sample time
float tau = 0.5;
float alpha = tau / (Ts + tau);
unsigned long last_update_time = 0;

float input_angle = 90;

// PID gains
float Kp = 7.2624;
float Ki = 1.3763;
float Kd = 0.64986;

// PID memory
float integral = 0;
float prev_error = 0;

// Target angle
float target_angle = 90.0;  // Start centered

void setup() {
  myServo.attach(servoPin, minPulse, maxPulse);
  Wire.begin();
  Serial.begin(115200);
  MPU.initialize();

  if (!MPU.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  MPU.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  MPU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  last_update_time = millis();
  moveconverter(input_angle);
  Serial.println("Setup complete");
}

void loop() {
  unsigned long now = millis();

  if (now - last_update_time >= Ts * 1000) {
    last_update_time += Ts * 1000;

    int16_t ax, ay, az, gx, gy, gz;
    MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accY = (float)ay / 16384.0;
    float accZ = (float)az / 16384.0;
    float accX = (float)ax / 16384.0;
    float theta_gyro = (float)gx / 131.0;

    float theta_accel = atan(accY / sqrt(accX * accX + accZ * accZ)) * 180.0 / PI + 90;

    theta_est = (1 - alpha) * theta_accel + alpha * (theta_est_prev + Ts * theta_gyro);
    theta_est_prev = theta_est;

    // PID controller
    float error = target_angle - theta_est;
    integral += error * Ts;
    float derivative = (error - prev_error) / Ts;
    float control_signal = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;

    // Calculate servo angle and constrain it
    input_angle = 90 + control_signal;

    moveconverter(input_angle);

    // Serial output
    Serial.print("Theta Est: "); Serial.print(theta_est, 3);
    Serial.print(" | Target: "); Serial.print(target_angle, 3);
    Serial.print(" | Error: "); Serial.print(error, 3);
    Serial.print(" | Control: "); Serial.print(control_signal, 3);
    Serial.print(" | Servo Angle: "); Serial.println(input_angle, 3);
  }

  // No delay() here!
}

void moveconverter(int angle) {
  angle = constrain(angle, 64, 116);
  int pulse = map(angle, 0, 270, minPulse, maxPulse);
  myServo.writeMicroseconds(pulse);
}
