#include <Servo.h> 
 
//Create the 4 esc objects
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
 
//Esc pins
int escPin1 = 6;
int escPin2 = 9;
int escPin3 = 10;
int escPin4 = 11;

//Min and max pulse
int minPulseRate        = 1000;
int maxPulseRate        = 2000;
int throttleChangeDelay = 50;
 
//SETUP
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);
  
  //Init escs
  initEscs();
}
 
//LOOP
void loop() {
  // Wait for some input
  if (Serial.available() > 0) {
    
    // Read the new throttle value
    int dir = Serial.parseInt();
    
    // Print it out
    Serial.print("Setting direction to: ");
    Serial.println(dir);

    switch(dir) {
      
      // front
      case 0:
        break;
      
      // right
      case 1:
        break;

      // back
      case 2:
        break;

      //left
      case 3:
        break;

      default:
        break;
    }
    
    // Change throttle to the new value
    changeThrottle(dir);
  }
 
}
 
//Change throttle value
void changeThrottle(int throttle) {
  int currentThrottle = readThrottle();
  
  int step = 5;
  if(throttle < currentThrottle) {
    step = -5;
  }
  
  // Slowly move to the new throttle value 
  while(currentThrottle != throttle) {
    writeTo4Escs(currentThrottle + step);
    
    currentThrottle = readThrottle();
    delay(throttleChangeDelay);
  }
}

//Read the throttle value
int readThrottle() {
  int throttle = esc1.read();
  
  Serial.print("Current throttle is: ");
  Serial.println(throttle);
  
  return throttle;
}

//Change velocity of the 4 escs at the same time
void writeTo4Escs(int throttle) {
  esc1.write(throttle);
  esc2.write(throttle);
  esc3.write(throttle);
  esc4.write(throttle);
}

//Init escs
void initEscs() {
  esc1.attach(escPin1, minPulseRate, maxPulseRate);
  esc2.attach(escPin2, minPulseRate, maxPulseRate);
  esc3.attach(escPin3, minPulseRate, maxPulseRate);
  esc4.attach(escPin4, minPulseRate, maxPulseRate);
  
  //Init motors with 0 value
  writeTo4Escs(0);
}


//// ---------------------------------------------------------------------------
//#include <Arduino_LSM9DS1.h>
//// ------------------- Define some constants for convenience -----------------
////#define CHANNEL1 0
////#define CHANNEL2 1
////#define CHANNEL3 2
////#define CHANNEL4 3
//
//#define YAW      0
//#define PITCH    1
//#define ROLL     2
//#define THROTTLE 3
//
//#define X           0     // X axis
//#define Y           1     // Y axis
//#define Z           2     // Z axis
//#define FREQ        IMU.gyroscopeSampleRate()   // Sampling frequency
//#define SSF_GYRO    70  // Sensitivity Scale Factor of the gyro from datasheet
//
//#define STOPPED  0
//#define STARTING 1
//#define STARTED  2
//// ---------------- Receiver variables ---------------------------------------
//// Previous state of each channel (HIGH or LOW)
//volatile byte previous_state[4];
//
//// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
//volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};
//
//// Used to calculate pulse duration on each channel
//volatile unsigned long current_time;
//volatile unsigned long timer[4]; // Timer of each channel
//
//// Used to configure which control (yaw, pitch, roll, throttle) is on which channel
//int mode_mapping[4];
//
//// ----------------------- IMU variables -------------------------------------
//// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
//int gyro_raw[3] = {0,0,0};
//
//// Average gyro offsets of each axis in that order: X, Y, Z
//long gyro_offset[3] = {0, 0, 0};
//
//// Calculated angles from gyro's values in that order: X, Y, Z
//float gyro_angle[3]  = {0,0,0};
//
//// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
//float acc_raw[3] = {0 ,0 ,0};
//
//// Calculated angles from accelerometer's values in that order: X, Y, Z
//float acc_angle[3] = {0,0,0};
//
//// Total 3D acceleration vector in m/s²
//long acc_total_vector;
//
//// Calculated angular motion on each axis: Yaw, Pitch, Roll
//float angular_motions[3] = {0, 0, 0};
//
///**
// * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
// *  - Left wing up implies a positive roll
// *  - Nose up implies a positive pitch
// *  - Nose right implies a positive yaw
// */
//float measures[3] = {0, 0, 0};
//
//// MPU's temperature
//// int temperature;
//
//// Init flag set to TRUE after first loop
//boolean initialized;
//// ----------------------- Variables for servo signal generation -------------
//unsigned int  period; // Sampling period
//unsigned long loop_timer;
//unsigned long now, difference;
//
//unsigned long pulse_length_esc1 = 1000,
//        pulse_length_esc2 = 1000,
//        pulse_length_esc3 = 1000,
//        pulse_length_esc4 = 1000;
//
//// ------------- Global variables used for PID controller --------------------
//float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll
//// Errors
//float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
//float delta_err[3]      = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
//float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
//float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
//// PID coefficients
//float Kp[3] = {4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
//float Ki[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
//float Kd[3] = {0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll
//// ---------------------------------------------------------------------------
///**
// * Status of the quadcopter:
// *   - 0 : stopped
// *   - 1 : starting
// *   - 2 : started
// *
// * @var int
// */
//int status = STOPPED;
//// ---------------------------------------------------------------------------
//int battery_voltage;
//// ---------------------------------------------------------------------------
//
///**
// * Setup configuration
// */
//void setup() {
//  // IMU Setup
//  // check IMU
//  if (!IMU.begin()) {
//    Serial.println("Error initializing IMU");
//  }
//
//  // Initialize loop_timer
//  loop_timer = micros();
//}
//
///**
// * Main program loop
// */
//void loop() {
//    // 1. First, read raw values from Nano IMU (done)
//    readSensor();
//
//    // 2. Calculate angles from gyro & accelerometer's values (done)
//    calculateAngles();
//
//    // 3. Calculate set points of PID controller
//    calculateSetPoints();
//
//    // 4. Calculate errors comparing angular motions to set points
//    calculateErrors();
//
//    if (isStarted()) {
//        // 5. Calculate motors speed with PID controller
//        pidController();
//
//        compensateBatteryDrop();
//    }
//
//    // 6. Apply motors speed
//    applyMotorSpeed();
//}
//
///**
// * Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
// * Direct port manipulation is used for performances.
// *
// * This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.
// *
// * @see https:// www.arduino.cc/en/Reference/PortManipulation
// */
//void applyMotorSpeed() {
//    // Refresh rate is 250Hz: send ESC pulses every 4000µs
//    while ((now = micros()) - loop_timer < period);
//
//    // Update loop timer
//    loop_timer = now;
//
//    // Set pins #4 #5 #6 #7 HIGH
//    PORTD |= B11110000;
//
//    // Wait until all pins #4 #5 #6 #7 are LOW
//    while (PORTD >= 16) {
//        now        = micros();
//        difference = now - loop_timer;
//
//        if (difference >= pulse_length_esc1) PORTD &= B11101111; // Set pin #4 LOW
//        if (difference >= pulse_length_esc2) PORTD &= B11011111; // Set pin #5 LOW
//        if (difference >= pulse_length_esc3) PORTD &= B10111111; // Set pin #6 LOW
//        if (difference >= pulse_length_esc4) PORTD &= B01111111; // Set pin #7 LOW
//    }
//}
//
///**
// * Request raw values from MPU6050.
// */
//void readSensor() {
//  // wait until IMU data is available
//  while (!IMU.accelerationAvailable());
//  IMU.readAcceleration(acc_raw[X], acc_raw[Y], acc_raw[Z]);
//
//  while (!IMU.gyroscopeAvailable());
//  IMU.readGyroscope(gyro_raw[X], gyro_raw[Y], gyro_raw[Z]);
//  
//  // temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
//}
//
///**
// * Calculate real angles from gyro and accelerometer's values
// */
//void calculateAngles() {
//    calculateGyroAngles();
//    calculateAccelerometerAngles();
//
//    if (initialized) {
//        // Correct the drift of the gyro with the accelerometer
//        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
//        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
//    } else {
//        // At very first start, init gyro angles with accelerometer angles
//        resetGyroAngles();
//
//        initialized = true;
//    }
//
//    // To dampen the pitch and roll angles a complementary filter is used
//    measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
//    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
//    measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; // Store the angular motion for this axis
//
//    // Apply low-pass filter (10Hz cutoff frequency)
//    angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * gyro_raw[X] / SSF_GYRO;
//    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO;
//    angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * gyro_raw[Z] / SSF_GYRO;
//}
//
///**
// * Calculate pitch & roll angles using only the gyro.
// */
//void calculateGyroAngles() {   
//    // Subtract offsets
//    gyro_raw[X] -= gyro_offset[X];
//    gyro_raw[Y] -= gyro_offset[Y];
//    gyro_raw[Z] -= gyro_offset[Z];
//
//    // Angle calculation using integration
//    gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
//    gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one
//
//    // Transfer roll to pitch if IMU has yawed
//    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
//    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
//}
//
///**
// * Calculate pitch & roll angles using only the accelerometer.
// */
//void calculateAccelerometerAngles() {
////  float x_deg = atanf(x_g / sqrt(pow(y_g, 2) + pow(z_g, 2))) * 180 / PI;
////  float y_deg = atanf(y_g / sqrt(pow(x_g, 2) + pow(z_g, 2))) * 180 / PI;
////  float z_deg = atanf(sqrt(pow(y_g, 2) + pow(x_g, 2)) / z_g ) * 180 / PI;
//
//  // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
//  acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));
//  
//  // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
//  if (abs(acc_raw[X]) < acc_total_vector) {
//      acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
//  }
//  
//  if (abs(acc_raw[Y]) < acc_total_vector) {
//      acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
//  }
//}
//
///**
// * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
// * by applying PID control.
// *
// * (A) (B)     x
// *   \ /     z ↑
// *    X       \|
// *   / \       +----→ y
// * (C) (D)
// *
// * Motors A & D run clockwise.
// * Motors B & C run counter-clockwise.
// *
// * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
// */
//void pidController() {
//    float yaw_pid      = 0;
//    float pitch_pid    = 0;
//    float roll_pid     = 0;
//    int   throttle     = pulse_length[mode_mapping[THROTTLE]];
//
//    // Initialize motor commands with throttle
//    pulse_length_esc1 = throttle;
//    pulse_length_esc2 = throttle;
//    pulse_length_esc3 = throttle;
//    pulse_length_esc4 = throttle;
//
//    // Do not calculate anything if throttle is 0
//    if (throttle >= 1012) {
//        // PID = e.Kp + ∫e.Ki + Δe.Kd
//        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
//        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
//        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);
//
//        // Keep values within acceptable range. TODO export hard-coded values in variables/const
//        yaw_pid   = minMax(yaw_pid, -400, 400);
//        pitch_pid = minMax(pitch_pid, -400, 400);
//        roll_pid  = minMax(roll_pid, -400, 400);
//
//        // Calculate pulse duration for each ESC
//        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
//        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
//        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
//        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;
//    }
//
//    // Prevent out-of-range-values
//    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
//    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
//    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
//    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
//}
//
///**
// * Calculate errors used by PID controller
// */
//void calculateErrors() {
//    // Calculate current errors
//    errors[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
//    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
//    errors[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];
//
//    // Calculate sum of errors : Integral coefficients
//    error_sum[YAW]   += errors[YAW];
//    error_sum[PITCH] += errors[PITCH];
//    error_sum[ROLL]  += errors[ROLL];
//
//    // Keep values in acceptable range
//    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
//    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
//    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);
//
//    // Calculate error delta : Derivative coefficients
//    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
//    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
//    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];
//
//    // Save current error as previous_error for next time
//    previous_error[YAW]   = errors[YAW];
//    previous_error[PITCH] = errors[PITCH];
//    previous_error[ROLL]  = errors[ROLL];
//}
//
///**
// * Customize mapping of controls: set here which command is on which channel and call
// * this function in setup() routine.
// */
//void configureChannelMapping() {
//    mode_mapping[YAW]      = CHANNEL4;
//    mode_mapping[PITCH]    = CHANNEL2;
//    mode_mapping[ROLL]     = CHANNEL1;
//    mode_mapping[THROTTLE] = CHANNEL3;
//}
//
///**
// * Make sure that given value is not over min_value/max_value range.
// *
// * @param float value     : The value to convert
// * @param float min_value : The min value
// * @param float max_value : The max value
// *
// * @return float
// */
//float minMax(float value, float min_value, float max_value) {
//    if (value > max_value) {
//        value = max_value;
//    } else if (value < min_value) {
//        value = min_value;
//    }
//
//    return value;
//}
//
///**
// * Return whether the quadcopter is started.
// * To start the quadcopter, move the left stick in bottom left corner then, move it back in center position.
// * To stop the quadcopter move the left stick in bottom right corner.
// *
// * @return bool
// */
//bool isStarted() {
//    // When left stick is moved in the bottom left corner
//    if (status == STOPPED && pulse_length[mode_mapping[YAW]] <= 1012 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
//        status = STARTING;
//    }
//
//    // When left stick is moved back in the center position
//    if (status == STARTING && pulse_length[mode_mapping[YAW]] == 1500 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
//        status = STARTED;
//
//        // Reset PID controller's variables to prevent bump start
//        resetPidController();
//
//        resetGyroAngles();
//    }
//
//    // When left stick is moved in the bottom right corner
//    if (status == STARTED && pulse_length[mode_mapping[YAW]] >= 1988 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
//        status = STOPPED;
//        // Make sure to always stop motors when status is STOPPED
//        stopAll();
//    }
//
//    return status == STARTED;
//}
//
///**
// * Reset gyro's angles with accelerometer's angles.
// */
//void resetGyroAngles() {
//    gyro_angle[X] = acc_angle[X];
//    gyro_angle[Y] = acc_angle[Y];
//}
//
///**
// * Reset motors' pulse length to 1000µs to totally stop them.
// */
//void stopAll() {
//    pulse_length_esc1 = 1000;
//    pulse_length_esc2 = 1000;
//    pulse_length_esc3 = 1000;
//    pulse_length_esc4 = 1000;
//}
//
///**
// * Reset all PID controller's variables.
// */
//void resetPidController() {
//    errors[YAW]   = 0;
//    errors[PITCH] = 0;
//    errors[ROLL]  = 0;
//
//    error_sum[YAW]   = 0;
//    error_sum[PITCH] = 0;
//    error_sum[ROLL]  = 0;
//
//    previous_error[YAW]   = 0;
//    previous_error[PITCH] = 0;
//    previous_error[ROLL]  = 0;
//}
//
///**
// * Calculate PID set points on axis YAW, PITCH, ROLL
// */
//void calculateSetPoints() {
//    pid_set_points[YAW]   = calculateYawSetPoint(pulse_length[mode_mapping[YAW]], pulse_length[mode_mapping[THROTTLE]]);
//    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse_length[mode_mapping[PITCH]]);
//    pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL], pulse_length[mode_mapping[ROLL]]);
//}
//
///**
// * Calculate the PID set point in °/s
// *
// * @param float angle         Measured angle (in °) on an axis
// * @param int   channel_pulse Pulse length of the corresponding receiver channel
// * @return float
// */
//float calculateSetPoint(float angle, int channel_pulse) {
//    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
//    float set_point    = 0;
//
//    // Need a dead band of 16µs for better result
//    if (channel_pulse > 1508) {
//        set_point = channel_pulse - 1508;
//    } else if (channel_pulse <  1492) {
//        set_point = channel_pulse - 1492;
//    }
//
//    set_point -= level_adjust;
//    set_point /= 3;
//
//    return set_point;
//}
//
///**
// * Calculate the PID set point of YAW axis in °/s
// *
// * @param int yaw_pulse      Receiver pulse length of yaw's channel
// * @param int throttle_pulse Receiver pulse length of throttle's channel
// * @return float
// */
//float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
//    float set_point = 0;
//
//    // Do not yaw when turning off the motors
//    if (throttle_pulse > 1050) {
//        // There is no notion of angle on this axis as the quadcopter can turn on itself
//        set_point = calculateSetPoint(0, yaw_pulse);
//    }
//
//    return set_point;
//}
//
///**
// * Compensate battery drop applying a coefficient on output values
// */
//void compensateBatteryDrop() {
//    if (isBatteryConnected()) {
//        pulse_length_esc1 += pulse_length_esc1 * ((1240 - battery_voltage) / (float) 3500);
//        pulse_length_esc2 += pulse_length_esc2 * ((1240 - battery_voltage) / (float) 3500);
//        pulse_length_esc3 += pulse_length_esc3 * ((1240 - battery_voltage) / (float) 3500);
//        pulse_length_esc4 += pulse_length_esc4 * ((1240 - battery_voltage) / (float) 3500);
//    }
//}
//
///**
// * Read battery voltage & return whether the battery seems connected
// *
// * @return boolean
// */
//bool isBatteryConnected() {
//    // Reduce noise with a low-pass filter (10Hz cutoff frequency)
//    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
//
//    return battery_voltage < 1240 && battery_voltage > 800;
//}
//
///**
// * This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
// * Read the receiver signals in order to get flight instructions.
// *
// * This routine must be as fast as possible to prevent main program to be messed up.
// * The trick here is to use port registers to read pin state.
// * Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
// * It is less convenient but more efficient, which is the most important here.
// *
// * @see https://www.arduino.cc/en/Reference/PortManipulation
// * @see https://www.firediy.fr/article/utiliser-sa-radiocommande-avec-un-arduino-drone-ch-6
// */
//ISR(PCINT0_vect) {
//        current_time = micros();
//
//        // Channel 1 -------------------------------------------------
//        if (PINB & B00000001) {                                        // Is input 8 high ?
//            if (previous_state[CHANNEL1] == LOW) {                     // Input 8 changed from 0 to 1 (rising edge)
//                previous_state[CHANNEL1] = HIGH;                       // Save current state
//                timer[CHANNEL1] = current_time;                        // Save current time
//            }
//        } else if (previous_state[CHANNEL1] == HIGH) {                 // Input 8 changed from 1 to 0 (falling edge)
//            previous_state[CHANNEL1] = LOW;                            // Save current state
//            pulse_length[CHANNEL1] = current_time - timer[CHANNEL1];   // Calculate pulse duration & save it
//        }
//
//        // Channel 2 -------------------------------------------------
//        if (PINB & B00000010) {                                        // Is input 9 high ?
//            if (previous_state[CHANNEL2] == LOW) {                     // Input 9 changed from 0 to 1 (rising edge)
//                previous_state[CHANNEL2] = HIGH;                       // Save current state
//                timer[CHANNEL2] = current_time;                        // Save current time
//            }
//        } else if (previous_state[CHANNEL2] == HIGH) {                 // Input 9 changed from 1 to 0 (falling edge)
//            previous_state[CHANNEL2] = LOW;                            // Save current state
//            pulse_length[CHANNEL2] = current_time - timer[CHANNEL2];   // Calculate pulse duration & save it
//        }
//
//        // Channel 3 -------------------------------------------------
//        if (PINB & B00000100) {                                        // Is input 10 high ?
//            if (previous_state[CHANNEL3] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
//                previous_state[CHANNEL3] = HIGH;                       // Save current state
//                timer[CHANNEL3] = current_time;                        // Save current time
//            }
//        } else if (previous_state[CHANNEL3] == HIGH) {                 // Input 10 changed from 1 to 0 (falling edge)
//            previous_state[CHANNEL3] = LOW;                            // Save current state
//            pulse_length[CHANNEL3] = current_time - timer[CHANNEL3];   // Calculate pulse duration & save it
//        }
//
//        // Channel 4 -------------------------------------------------
//        if (PINB & B00001000) {                                        // Is input 11 high ?
//            if (previous_state[CHANNEL4] == LOW) {                     // Input 11 changed from 0 to 1 (rising edge)
//                previous_state[CHANNEL4] = HIGH;                       // Save current state
//                timer[CHANNEL4] = current_time;                        // Save current time
//            }
//        } else if (previous_state[CHANNEL4] == HIGH) {                 // Input 11 changed from 1 to 0 (falling edge)
//            previous_state[CHANNEL4] = LOW;                            // Save current state
//            pulse_length[CHANNEL4] = current_time - timer[CHANNEL4];   // Calculate pulse duration & save it
//        }
//}
