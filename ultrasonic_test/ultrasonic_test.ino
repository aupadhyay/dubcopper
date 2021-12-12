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

int trigPin0 = 0;  int echoPin0 = 1; // front
int trigPin1 = 2;  int echoPin1 = 3; // right
int trigPin2 = 4;  int echoPin2 = 5;  // back
int trigPin3 = 12; int echoPin3 = 13; // left


float dist0 = 0, dist1 = 0, dist2 = 0, dist3 = 0;

NewPing sonar0(trigPin0, echoPin0, MAX_DISTANCE);
NewPing sonar1(trigPin1, echoPin1, MAX_DISTANCE);
NewPing sonar2(trigPin2, echoPin2, MAX_DISTANCE);
NewPing sonar3(trigPin3, echoPin3, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
  resetSensor(echoPin0);
  resetSensor(echoPin1);
  resetSensor(echoPin2);
  resetSensor(echoPin3);
}

void loop() {
  delay(1000);
  dist0 = sonar0.ping_cm();
  if (dist0 == 0) {
    resetSensor(echoPin0);
    delay(15);
    dist0 = sonar0.ping_cm();
  }
  Serial.print("dist0: "); Serial.println(dist0);

  dist1 = sonar1.ping_cm();
  if (dist1 == 0) {
    resetSensor(echoPin1);
    delay(15);
    dist1 = sonar1.ping_cm();
  }
  Serial.print("dist1: "); Serial.println(dist1);

  dist2 = sonar2.ping_cm();
  if (dist2 == 0) {
    resetSensor(echoPin2);
    delay(15);
    dist2 = sonar2.ping_cm();
  }
  Serial.print("dist2: "); Serial.println(dist2);

  dist3 = sonar3.ping_cm();
  if (dist3 == 0) {
    resetSensor(echoPin3);
    delay(15);
    dist3 = sonar3.ping_cm();
  }
  Serial.print("dist3: "); Serial.println(dist3);

  Serial.println();
}

void resetSensor(int echoPin) {
  Serial.println("MAX: resetting sensor");
  pinMode(echoPin, OUTPUT);
  delay(15);
  digitalWrite(echoPin, LOW);
  delay(15);
  pinMode(echoPin, INPUT);
  delay(15);
}
