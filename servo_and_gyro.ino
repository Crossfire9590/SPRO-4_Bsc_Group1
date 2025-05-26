#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

Servo myServo;
const int servoPin = 9;
const int minPulse = 500;   // in microseconds
const int maxPulse = 2500;  // in microseconds

MPU6050 MPU;

float theta_est_prev = 0.0;      // Estimated angle at time [k-1]
float theta_est = 0.0;           // Estimated angle at time [k]
float Ts = 0.01;                 // Sampling time (in seconds)
float tau = 0.5;                 // Time constant for filter
float alpha = tau / (Ts + tau);  // Complementary filter parameter
unsigned long last_update_time = 0;




void setup() {
  myServo.attach(servoPin, minPulse, maxPulse); 
  Wire.begin();
  Serial.begin(115200);
  MPU.initialize();

  //Check for mpu connection
  if (!MPU.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  MPU.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250°/s
  MPU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g

  last_update_time = millis();
}

void loop() {
  for (int i = 0; i <= 100; i++) {
    unsigned long now = millis();
    if ((now - last_update_time) >= Ts * 1000) {
      last_update_time += Ts * 1000;

      //Read raw sensor data. Accelerometer reads acceleration and gyroscope reads angular velocity.
      int16_t ax, ay, az, gx, gy, gz;
      MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      //Convert to physical units
      float accY = (float)ay / 16384.0;  // ±2g = 16384 LSB/g
      float accZ = (float)az / 16384.0;
      float accX = (float)ax / 16384.0;
      float theta_gyro = (float)gx / 131.0;  // ±250°/s = 131 LSB/(°/s)

      //Compute theta_accel from accelerometer
      //float theta_accel = atan2(accY, accZ) * 180.0 / PI;
      float theta_accel = atan(accY / sqrt (accX * accX + accZ * accZ)) * 180.0 / PI;

      //Apply complementary filter
      theta_est = (1 - alpha) * theta_accel + alpha * (theta_est_prev + Ts * theta_gyro);
      //Store for next loop
      theta_est_prev = theta_est;

      //Output
      Serial.println(theta_est);
    }
  }
  int Rollconverter = int(theta_est) + 90;
  moveconverter(Rollconverter);
}

void moveconverter(int angle) {
  // Constrain angle to 0–270
  angle = constrain(angle, 0, 270);
  // Map 0–270° to minPulse–maxPulse
  int pulse = map(angle, 0, 270, minPulse, maxPulse);
  //set motor
  myServo.writeMicroseconds(pulse);
}
