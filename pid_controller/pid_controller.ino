// ---------------------------------------------------------------------------
#include <Servo.h>
#include <PID_v1.h>
#include "FastPID.h"
#include <Arduino_LSM9DS1.h>
// ---------------------------------------------------------------------------

#define MIN_PULSE_LENGTH 1100 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 1300 // Maximum pulse length in µs (2000)

#define PITCH_BIAS 2.05
#define ROLL_BIAS 1.45

#define ESC_PIN1 8
#define ESC_PIN2 9
#define ESC_PIN3 10
#define ESC_PIN4 11

#define OUTPUT_BITS 16
#define OUTPUT_SIGNED 1
#define MOTORS 1

#define BIAS 15

float pitch = 0, roll = 0;
float pitch_sp = 0, roll_sp = 0;
float kp_p = 0.2, ki_p = 0; // kp may be too high
float kp_r = 0.04, ki_r = 0;
float kd = 0.1, hz = 10;
int pitch_out, roll_out;

int esc1_val = 1200;
int esc2_val = 1200;
int esc3_val = 1200;
int esc4_val = 1200;

FastPID pitchPID(kp_p, ki_p, kd, hz, OUTPUT_BITS, OUTPUT_SIGNED);
FastPID rollPID(kp_r, ki_r, kd, hz, OUTPUT_BITS, OUTPUT_SIGNED);
Servo esc1, esc2, esc3, esc4;

void setup() {
  // Set up serial monitor
  Serial.begin(9600);
  Serial.setTimeout(500);
  while (!Serial);

  // Wait for IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    exit(0);
  }

  // Init escs
  initEscs();

  // PID Set Up
  if (pitchPID.err()) {
    Serial.println("PID CONFIG ERROR");
    exit(0);
  }
  pitchPID.setOutputRange(-100, 100);
  rollPID.setOutputRange(-100, 100);

  while (Serial.available() == 0);

  if (MOTORS) {
    for (int i = 1150; i <= 1200; i += 5) {
      Serial.print("Pulse length = ");
      Serial.println(i);
      writeTo4Escs(i);
      delay(200);
    }
  }
}

void loop() {
  // Read Sensors values and update input values for PID
  readSensors();
  Serial.print("pitch angle: "); Serial.println(pitch);
  Serial.print("roll angle: "); Serial.println(roll);

  // Compute PID values
  pitch_out = pitchPID.step(pitch_sp, pitch);
  roll_out = rollPID.step(roll_sp, roll);

  Serial.print("pitch_out: "); Serial.println(pitch_out);
  Serial.print("roll_out: "); Serial.println(roll_out);


  if (MOTORS) {
    esc1_val = esc1.readMicroseconds();
    esc2_val = esc2.readMicroseconds();
    esc3_val = esc3.readMicroseconds();
    esc4_val = esc4.readMicroseconds();
  }

  // Superimpose pitch, roll, yaw pid values
  Serial.print("esc1 val: "); Serial.println(esc1_val);
  Serial.print("esc2 val: "); Serial.println(esc2_val);
  Serial.print("esc3 val: "); Serial.println(esc3_val);
  Serial.print("esc4 val: "); Serial.println(esc4_val);
  Serial.println();

  // Pitch (-1 -2 +3 +4)
  esc1_val -= pitch_out;
  esc2_val -= pitch_out;
  esc3_val += pitch_out;
  esc4_val += pitch_out;

  // Roll (+1 -2 -3 +4)
  esc1_val += roll_out;
  esc2_val -= roll_out;
  esc3_val -= roll_out;
  esc4_val += roll_out;

  if (MOTORS) {
    writeTo4Escs(esc1_val, esc2_val, esc3_val, esc4_val, BIAS);
  }

  delay(10);
}

void readSensors() {
  if (!IMU.accelerationAvailable()) {
    Serial.println("Could not read from accel");
    return;
  }
  float x, y, z;
  IMU.readAcceleration(x, y, z);

  pitch = PITCH_BIAS + 180 * atan(x / sqrt(y * y + z * z)) / M_PI;
  roll = ROLL_BIAS + 180 * atan(y / sqrt(x * x + z * z)) / M_PI;
}

void writeTo4Escs(int throttle1, int throttle2, int throttle3, int throttle4, int bias) {
  // add bias back from read
  throttle1 += bias;
  throttle2 += bias;
  throttle3 += bias;
  throttle4 += bias;
  
  if (throttle1 < MIN_PULSE_LENGTH) {
    throttle1 = MIN_PULSE_LENGTH;
  }
  if (throttle1 > MAX_PULSE_LENGTH) {
    throttle1 = MAX_PULSE_LENGTH;
  }

  if (throttle2 < MIN_PULSE_LENGTH) {
    throttle2 = MIN_PULSE_LENGTH;
  }
  if (throttle2 > MAX_PULSE_LENGTH) {
    throttle2 = MAX_PULSE_LENGTH;
  }

  if (throttle3 < MIN_PULSE_LENGTH) {
    throttle3 = MIN_PULSE_LENGTH;
  }
  if (throttle3 > MAX_PULSE_LENGTH) {
    throttle3 = MAX_PULSE_LENGTH;
  }

  if (throttle4 < MIN_PULSE_LENGTH) {
    throttle4 = MIN_PULSE_LENGTH;
  }
  if (throttle4 > MAX_PULSE_LENGTH) {
    throttle4 = MAX_PULSE_LENGTH;
  }

  esc1.writeMicroseconds(throttle1);
  esc2.writeMicroseconds(throttle2);
  esc3.writeMicroseconds(throttle3);
  esc4.writeMicroseconds(throttle4);
}

void writeTo4Escs(int throttle) {
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  esc3.writeMicroseconds(throttle);
  esc4.writeMicroseconds(throttle);
}

// Init escs
void initEscs() {
  esc1.attach(ESC_PIN1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc2.attach(ESC_PIN2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc3.attach(ESC_PIN3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc4.attach(ESC_PIN4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

  // Init motors with 0 value
  writeTo4Escs(0);
}
