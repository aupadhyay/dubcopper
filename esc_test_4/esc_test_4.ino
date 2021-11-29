#include <Servo.h> 
 
//Create the 4 esc objects
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
 
//Esc pins
int escPin1 = 6;
int escPin2 = 7;
int escPin3 = 8;
int escPin4 = 9;

//Min and max pulse
int minPulseRate        = 1100;
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
    int throttle = normalizeThrottle(Serial.parseInt());
    
    // Print it out
    Serial.print("Setting throttle to: ");
    Serial.println(throttle);
    
    // Change throttle to the new value
    changeThrottle(throttle);
  }
//  int throttle = 50;
//  changeThrottle(throttle);
//  writeTo4Escs(throttle);
//  delay(10);
 
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
  Serial.print(throttle);
  Serial.print(esc2.read());
  Serial.print(" ");
  Serial.print(esc3.read());
  Serial.print(" ");
  Serial.print(esc4.read());
  Serial.print(" ");
  Serial.println();
  
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

//Start the motors
void startUpMotors() {
  writeTo4Escs(50);
}
 
// Ensure the throttle value is between 0 - 180
int normalizeThrottle(int value) {
  if(value < 0) {
    return 0;
    
  } else if(value > 180) {
    return 180;
  }
  
  return value;
}
