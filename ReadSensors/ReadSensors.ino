#include <Arduino_LSM9DS1.h>


#define PITCH_BIAS 2.4
#define ROLL_BIAS 1.23

float roll, pitch, yaw;
float roll_off = 0.0, pitch_off = 0.0, yaw_off = 0.0;

float x, y, z;
float mag_readX, mag_readY, mag_readZ;

void readSensors() {
  if (!IMU.accelerationAvailable()) {
    Serial.println("Could not read from accel");
  }
  IMU.readAcceleration(x, y, z);
  IMU.readMagneticField(mag_readX, mag_readY, mag_readZ);

  pitch = PITCH_BIAS + 180 * atan(x/sqrt(y*y + z*z))/M_PI;
  roll = ROLL_BIAS + 180 * atan(y/sqrt(x*x + z*z))/M_PI;

  // float Yh = (mag_readY * cos(roll)) - (mag_readZ * sin(roll));
  // float Xh = (mag_readX * cos(pitch))+(mag_readY * sin(roll)*sin(pitch)) + (mag_readZ * cos(roll) * sin(pitch));

  //  yaw =  atan2(Yh, Xh) * 57.3;

  // float mag_x = mag_readX * cos(pitch) + mag_readY * sin(roll) * sin(pitch) + mag_readZ * cos(roll) * sin(pitch);
  // float mag_y = mag_readY * cos(roll) - mag_readZ * sin(roll);
  // yaw = 180 * atan2(-mag_y,mag_x)/M_PI;
}

void setup() {
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    exit(0);
  }
}

void loop() {
  readSensors();
  Serial.print("Pitch: "); Serial.println(pitch);
  
  Serial.print("Roll: "); Serial.println(roll);
  
  Serial.print("Yaw: "); Serial.println(yaw);
  
  Serial.println();
  delay(100);
}
