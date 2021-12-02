/**
 * Usage, according to documentation(https://www.firediy.fr/files/drone/HW-01-V4.pdf) : 
 *     1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
 *     2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK
 *     3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
 *     4. Type 0 to send min throttle
 *     5. Several "beep" tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
 *     6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
 *     7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them
 *
 * @author lobodol <grobodol@gmail.com>
 */
// ---------------------------------------------------------------------------
#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 1200 // Maximum pulse length in µs (2000)
// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
char data;
// ---------------------------------------------------------------------------

/**
 * Initialisation routine
 */
void setup() {
    Serial.begin(9600);
    Serial.println("Test");
    displayInstructions();
    motA.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
//    motC.attach(11, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
//    motD.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motA.write(0);
    motB.write(0);
//    motC.write(0);
//    motD.write(0);
}

/**
 * Main function
 */
void loop() {
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      motA.writeMicroseconds(MIN_PULSE_LENGTH);
                      motB.writeMicroseconds(MIN_PULSE_LENGTH);
//                      motC.writeMicroseconds(MIN_PULSE_LENGTH);
//                      motD.writeMicroseconds(MIN_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      motA.writeMicroseconds(MAX_PULSE_LENGTH);
                      motB.writeMicroseconds(MAX_PULSE_LENGTH);
//                      motC.writeMicroseconds(MAX_PULSE_LENGTH);
//                      motD.writeMicroseconds(MAX_PULSE_LENGTH);
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
            case 51 : Serial.print("Running single test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test_single(motA);
            break;

            case 52 : Serial.print("Running new test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      testWrite();
            break;
        }
    }
    

}

/**
 * Test function: send min throttle to max throttle to each ESC.
 */
void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(i);
//        motD.writeMicroseconds(i);
//        motC.writeMicroseconds(i);
        
        delay(200);
    }

    Serial.println("STOP");
    motA.writeMicroseconds(MIN_PULSE_LENGTH);
    motB.writeMicroseconds(MIN_PULSE_LENGTH);
//    motD.writeMicroseconds(MIN_PULSE_LENGTH);
//    motC.writeMicroseconds(MIN_PULSE_LENGTH);
}

/**
 * Test single function: send min throttle to max throttle to each ESC.
 */
void test_single(Servo mot)
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        mot.writeMicroseconds(i);
        delay(200);
    }

    Serial.println("STOP");
    mot.writeMicroseconds(MIN_PULSE_LENGTH);
}

void testWrite()
{
    int m = 0; 
    int mm = 150;
    for (int i = m; i <= mm; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        motA.write(i);
        motB.write(i);
//        motC.write(i);
//        motD.write(i);
        
        delay(200);
    }

    Serial.println("STOP");
    motA.write(m);
    motB.write(m);
//    motC.write(m);
//    motD.write(m);
}

/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
}

//void writeTo4Escs(int throttle) {
//  esc1.write(throttle);
//  esc2.write(throttle);
//  esc3.write(throttle);
//  esc4.write(throttle);
//}

//void initEscs() {
//  esc1.attach(escPin1, minPulseRate, maxPulseRate);
//  esc2.attach(escPin2, minPulseRate, maxPulseRate);
//  esc3.attach(escPin3, minPulseRate, maxPulseRate);
//  esc4.attach(escPin4, minPulseRate, maxPulseRate);
//  
//  //Init motors with 0 value
//  writeTo4Escs(0);
//}
