#include <Arduino_LSM9DS1.h>
float roll, pitch, yaw;
void readGyro() {
  if (!IMU.accelerationAvailable()) {
    Serial.println("Could not read from accel");
  }
  IMU.readAcceleration(roll, pitch, yaw);
}

void setup() {
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    exit(0);
  }
}

void loop() {
  readGyro();
  Serial.print("X: ");
  Serial.println(roll);
  Serial.print("Y: ");
  Serial.println(pitch);
  Serial.print("Z: ");
  Serial.println(yaw);
  Serial.println();
  delay(100);
}