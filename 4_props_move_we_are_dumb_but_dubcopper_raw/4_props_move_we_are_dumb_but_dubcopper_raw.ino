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

// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 1200 // Maximum pulse length in µs (2000)
// ---------------------------------------------------------------------------

char data;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);
  while(!Serial) delay(1);
  
  //Init escs
  initEscs();
}

void loop() {
  if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
        }
  }

}

void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        esc1.writeMicroseconds(i);
        esc2.writeMicroseconds(i);
        esc3.writeMicroseconds(i);
        esc4.writeMicroseconds(i);
        
        delay(200);
    }

    Serial.println("STOP");
    esc1.writeMicroseconds(MIN_PULSE_LENGTH);
    esc2.writeMicroseconds(MIN_PULSE_LENGTH);
    esc3.writeMicroseconds(MIN_PULSE_LENGTH);
    esc4.writeMicroseconds(MIN_PULSE_LENGTH);
}

void writeTo4Escs(int throttle) {
  esc1.write(throttle);
  Serial.print("wrote to esc1:");
  Serial.println(throttle);
  
  esc2.write(throttle);
  Serial.print("wrote to esc2:");
  Serial.println(throttle);
  
  esc3.write(throttle);
  Serial.print("wrote to esc3:");
  Serial.println(throttle);
  
  esc4.write(throttle);
  Serial.print("wrote to esc3:");
  Serial.println(throttle);
}

//Init escs
void initEscs() {
  esc1.attach(escPin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc2.attach(escPin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc3.attach(escPin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc4.attach(escPin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  
  //Init motors with 0 value
  writeTo4Escs(0);
  Serial.println("Initialized");
}
