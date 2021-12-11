// ---------------------------------------------------------------------------
#include <Servo.h>
#include <PID_v1.h>
#include <Arduino_LSM9DS1.h>
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------

#define MIN_PULSE_LENGTH 1100 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 1600 // Maximum pulse length in µs (2000)

Servo esc1, esc2, esc3, esc4;

int esc_pin1 = 6;
int esc_pin2 = 9;
int esc_pin3 = 10;
int esc_pin4 = 11;

double pitch = 0, roll = 0, yaw = 0;
double pitch_out, roll_out, yaw_out;
double pitch_sp = 0, roll_sp = 0, yaw_sp = 0;

double kp = 0.1, ki = 0.5, kd = 0.3;
PID pitchPID(&pitch, &pitch_out, &pitch_sp, kp, ki, kd, DIRECT);
PID rollPID(&roll, &roll_out, &roll_sp, kp, ki, kd, DIRECT);
PID yawPID(&yaw, &yaw_out, &yaw_sp, kp, ki, kd, DIRECT);
// ---------------------------------------------------------------------------

//int esc1_val = 1400;
//int esc2_val = 1400;
//int esc3_val = 1400;
//int esc4_val = 1400;

void setup() {
  //Set up serial monitor
  Serial.begin(9600);
  Serial.setTimeout(500);
  while(!Serial);

  //Wait for IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    exit(0);
  }
  
  //Init escs
  initEscs();

  //Set PID Modes
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-100, 100);
  
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-100, 100);
  
  //yawPID.SetMode(AUTOMATIC);
  //yawPID.SetOutputLimits(-100, 100);
  while(Serial.available() == 0);

  for (int i = 1100; i <= 1300; i += 5) {
    Serial.print("Pulse length = ");
    Serial.println(i);    
    writeTo4Escs(i);
    delay(200);
  }
  writeTo4Escs(1300, 1304, 1301, 1300);
  delay(200);
  writeTo4Escs(1300, 1304, 1301, 1300);
  delay(200);
}

void loop() {
  //Read Sensors values and update input values for PID
  readSensors();
  Serial.print("pitch angle"); Serial.println(pitch);
  Serial.print("roll angle"); Serial.println(roll);

  //Compute PID values
  pitchPID.Compute();
  rollPID.Compute();
  //yawPID.Compute();
 
  Serial.print("pitch_out"); Serial.println(pitch_out);
  Serial.print("roll_out"); Serial.println(roll_out);

  Serial.print("esc1 ang: "); Serial.println(esc1.read());
  Serial.print("esc2 ang: "); Serial.println(esc2.read());
  Serial.print("esc3 ang: "); Serial.println(esc3.read());
  Serial.print("esc4 ang: "); Serial.println(esc4.read());
  
  //Read current esc values
  int esc1_val = int(map(esc1.read(), 0, 180, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH));
  int esc2_val = int(map(esc2.read(), 0, 180, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH));
  int esc3_val = int(map(esc3.read(), 0, 180, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH));
  int esc4_val = int(map(esc4.read(), 0, 180, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH));

  //Superimpose pitch, roll, yaw pid values
  Serial.print("esc1 val: "); Serial.println(esc1_val);
  Serial.print("esc2 val: "); Serial.println(esc2_val);
  Serial.print("esc3 val: "); Serial.println(esc3_val);
  Serial.print("esc4 val: "); Serial.println(esc4_val);
  Serial.println();
  
  //Pitch (-1 -2 +3 +4)
  esc1_val -= pitch_out;
  esc2_val -= pitch_out;
  esc3_val += pitch_out;
  esc4_val += pitch_out;

  //roll (-1 +2 +3 -4)
  esc1_val -= roll_out;
  esc2_val += roll_out;
  esc3_val += roll_out;
  esc4_val -= roll_out;

//  //yaw (-1 +2 -3 +4)
//  esc1_val -= yaw_out;
//  esc2_val += yaw_out;
//  esc3_val -= yaw_out;
//  esc4_val += yaw_out;

  writeTo4Escs(esc1_val, esc2_val, esc3_val, esc4_val);
  delay(100);
}

void readSensors() {
  if (!IMU.accelerationAvailable()) {
    Serial.println("Could not read from accel");
    return;
  }
  float x, y, z;
  IMU.readAcceleration(x, y, z);
  
  pitch = 180 * atan(x/sqrt(y*y + z*z))/M_PI;
  roll = 180 * atan(y/sqrt(x*x + z*z))/M_PI;
}

void writeTo4Escs(int throttle1, int throttle2, int throttle3, int throttle4) {
  if (throttle1 < MIN_PULSE_LENGTH) {
    throttle1 = MIN_PULSE_LENGTH;
  }
  if (throttle1 > MAX_PULSE_LENGTH) {
    throttle1 = MAX_PULSE_LENGTH;
  }

  if (throttle2 < MIN_PULSE_LENGTH) {
    throttle1 = MIN_PULSE_LENGTH;
  }
  if (throttle2 > MAX_PULSE_LENGTH) {
    throttle1 = MAX_PULSE_LENGTH;
  }

  if (throttle3 < MIN_PULSE_LENGTH) {
    throttle1 = MIN_PULSE_LENGTH;
  }
  if (throttle3 > MAX_PULSE_LENGTH) {
    throttle1 = MAX_PULSE_LENGTH;
  }

  if (throttle4 < MIN_PULSE_LENGTH) {
    throttle1 = MIN_PULSE_LENGTH;
  }
  if (throttle4 > MAX_PULSE_LENGTH) {
    throttle1 = MAX_PULSE_LENGTH;
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

//Init escs
void initEscs() {
  esc1.attach(esc_pin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc2.attach(esc_pin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc3.attach(esc_pin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc4.attach(esc_pin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  
  //Init motors with 0 value
  writeTo4Escs(0);
}