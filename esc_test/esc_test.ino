#include <Servo.h>
 
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int escNum = 4;
int escPins[] = {6, 7, 8, 9};
Servo escs[] = {esc1, esc2, esc3, esc4};
 
void setup() {
  Serial.println("start of setup");

  esc1.attach(6);
  esc2.attach(7);
  esc3.attach(8);
  esc4.attach(9);
  
  esc1.write(0);
  delay(1000);
  esc2.write(0);
  delay(1000);
  esc3.write(0);
  delay(1000);
  esc4.write(0);
  delay(1000);
  
//  for (int i=0; i<escNum; i++) {
//    escs[i].attach(escPins[i]);
//    escs[i].write(0);
//    delay(1000);
//  }
  
  Serial.println("end of setup");
}
 
void loop(){
  
  Serial.println("start of loop");
  //setSpeed(100);
  setSpeedN(100, 0);
  delay(1000);
  setSpeedN(100, 1);
  delay(1000);
  setSpeedN(100, 2);
  delay(1000);
  setSpeedN(100, 3);
  delay(1000);

//  for(int angle = 30; angle < 120; angle += 5) {
//    Serial.println(angle);
//    setSpeed(angle);
//    delay(100);
//  }

  Serial.println("end of loop");
}

void setSpeed(int angle) {
  if (angle < 180) {
    for (int i=0; i<escNum; i++) {
      escs[i].write(angle);
      delay(1000);
    }
  }
}

void setSpeedN(int angle, int i) {
  if (angle < 180) {
      escs[i].write(angle);
  }
}
