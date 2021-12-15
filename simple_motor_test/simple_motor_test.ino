/**
 *     1. Plug Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
 *     2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK
 *     3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
 *     4. Type 0 to send min throttle
 *     5. Several "beep" tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
 *     6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
 *     7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them
 */
 
// ---------------------------------------------------------------------------
#include <Servo.h>
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
Servo esc1, esc2, esc3, esc4;
char data;

int escPin1 = 8;
int escPin2 = 9;
int escPin3 = 10;
int escPin4 = 11;

#define MIN_PULSE_LENGTH 1100 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 1400 // Maximum pulse length in µs (2000)
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);
  while(!Serial);
  
  //Init escs
  initEscs();
}

void loop() {
  if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      writeTo4Escs(MIN_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      writeTo4Escs(MAX_PULSE_LENGTH);
            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;

            // 3
            case 51: Serial.println("EXITING");
                     writeTo4Escs(0);
            break;

            case 52:  {
              Serial.print("Trying motor ");

              while(1) {
                writeToEsc(esc3, MIN_PULSE_LENGTH);
                delay(200);
              }
              break;
            }

            //5
            case 53:  {
              Serial.print("loop ");

              while(1) {
                writeTo4Escs(1200);
                delay(200);

                int esc1Read = esc1.readMicroseconds(), esc2Read = esc2.readMicroseconds(), esc3Read = esc3.readMicroseconds(), esc4Read = esc4.readMicroseconds();

                Serial.print("esc1 read: "); Serial.println(esc1Read);
                Serial.print("esc2 read: "); Serial.println(esc2Read);
                Serial.print("esc3 read: "); Serial.println(esc3Read);
                Serial.print("esc4 read: "); Serial.println(esc4Read);
              }
              break;
            }
        }
  }

}

void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        writeTo4Escs(i);

        Serial.println(esc1.read());
        
        delay(200);
    }

    Serial.println("STOP");
    writeTo4Escs(0);
}

void writeTo4Escs(int throttle) {
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  esc3.writeMicroseconds(throttle);
  esc4.writeMicroseconds(throttle);

}

void writeToEsc(Servo i, int throttle) {
  i.write(throttle);
}

//Init escs
void initEscs() {
  esc1.attach(escPin1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc2.attach(escPin2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc3.attach(escPin3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc4.attach(escPin4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  
  //Init motors with 0 value
  writeTo4Escs(0);
}
