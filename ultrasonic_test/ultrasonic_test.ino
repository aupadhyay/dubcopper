/***
 * Code to test HC SR-04 ultrasonic sensors with arduino
 * 
 * Initially did not use NewPing library, but used it later on 
 *       after research showed that it works more reliably
 * 
 * This code also resets the sensor when it reads 0 values
 * 
 * Old code is commented below
 */

#include <NewPing.h>

#define TRIGGER_PIN 8
#define ECHO_PIN 9
#define MAX_DISTANCE 300

int trigPin0 = 8;
int echoPin0 = 9;

int trigPin1 = 10;
int echoPin1 = 11;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonar0(trigPin0, echoPin0, MAX_DISTANCE);
NewPing sonar1(trigPin1, echoPin1, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(150);
  int uS0 = sonar0.ping();
  if (uS0 == 0) {
    Serial.println("MAX: resetting sensor0");
    pinMode(ECHO_PIN, OUTPUT);
    delay(150);
    digitalWrite(ECHO_PIN, LOW);
    delay(150);
    pinMode(ECHO_PIN, INPUT);
    delay(150);
  } else {
    Serial.print(" ");
    Serial.print("Ping 0: ");
    Serial.print(uS0 / US_ROUNDTRIP_CM);
    Serial.println("cm");
  }

  int uS1 = sonar1.ping();
  if (uS1 == 0) {
    Serial.println("MAX: resetting sensor1");
    pinMode(ECHO_PIN, OUTPUT);
    delay(150);
    digitalWrite(ECHO_PIN, LOW);
    delay(150);
    pinMode(ECHO_PIN, INPUT);
    delay(150);
  } else {
    Serial.print(" ");
    Serial.print("Ping 1: ");
    Serial.print(uS1 / US_ROUNDTRIP_CM);
    Serial.println("cm");
  }
}

/***
 * Old code that did not use NewPing and did not reset sensor
 */
 
//int trigPins[] = {12};
//int echoPins[]= {11};
//int len = 1;
//
//void setup() {
//  Serial.begin (9600);
//  for (int i=0; i<len; i++) {
//    pinMode(trigPins[i], OUTPUT);
//    pinMode(echoPins[i], INPUT);
//  }
////  pinMode(trigPin2, OUTPUT);
////  pinMode(echoPin2, INPUT);
////  pinMode(trigPin3, OUTPUT);
////  pinMode(echoPin3, INPUT);
//}
//
//void loop() {
//  float RightSensor = distance(trigPins[0], echoPins[0]);
//  
////  SonarSensor(trigPin2, echoPin2);
////  LeftSensor = distance;
////  SonarSensor(trigPin3, echoPin3);
////  FrontSensor = distance;
//
//}
//
//float distance(int trigPin,int echoPin) {
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  
//  float duration = pulseIn(echoPin, HIGH);
//  //float distance = (duration/2) / 29.1;
//  float distance = (duration / 2) * 0.0343;
//
//  Serial.print("detected object ");
//  Serial.print(distance);
//  Serial.println(" cm away");
//  
//  return distance;
//}
