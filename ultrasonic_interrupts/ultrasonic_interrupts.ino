/***
 * Ultrasonic w interrupts on Arduino
 */

#include <NewPing.h>

#define tINT 2            // Total number of interrupts
#define triggerPin 4      // Pin number for the common trigger
#define pingDelay 50      // How many milliseconds between each measurement ; keep > 5ms
#define debugDelay 200    // How many milliseconds between each Serial.print ; keep > 200ms
#define soundSpeed 343.0  // Speed of sound in m/s

#define MAX_DISTANCE 300

int trigPin = 8;

int echoPin0 = 9;
int echoPin1 = 10;
int echoPin2 = 11;

NewPing sonar0(trigPin, echoPin0, MAX_DISTANCE);
NewPing sonar1(trigPin, echoPin1, MAX_DISTANCE);
NewPing sonar2(trigPin, echoPin2, MAX_DISTANCE);

volatile unsigned long travelTime[tINT];  // Place to store traveltime of the pusle
volatile unsigned long startTime[tINT];   // Place to store ping times (interrupt)
float distance[tINT];                     // Calculated distances in cm
unsigned long lastPollMillis;
unsigned long lastDebugMillis;



/****************************************************************
      SETUP
****************************************************************/
void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial
  }
  Serial.println("--- Serial monitor started ---");
  pinMode(triggerPin, OUTPUT);   // Set common triggerpin as output

  // Manage interrupt pins here
  pinMode(2, INPUT);    // Set interrupt pin 2 (INT0) as INPUT (sensor 1)
  pinMode(3, INPUT);    // Set interrupt pin 3 (INT1) as INPUT (sensor 2)
  attachInterrupt(0, call_INT0, CHANGE );   // ISR for INT0
  attachInterrupt(1, call_INT1, CHANGE );   // ISR for INT1
  // END

  lastPollMillis = millis();
  lastDebugMillis = millis();
}

/****************************************************************
      LOOP
****************************************************************/
void loop()
{
  // Poll every x ms
  if (millis() - lastPollMillis >= pingDelay)
  {
    doMeasurement();
    lastPollMillis = millis();
  }

  // Print every y ms (comment out in production)
  if (millis() - lastDebugMillis >= debugDelay)
  {
    for (int i = 0; i < tINT; i++) {
      Serial.print(distance[i]);
      Serial.print(" - ");
    }
    Serial.println();
    lastDebugMillis = millis();
  }
}

/****************************************************************
      Retrieve measurement and set next trigger
****************************************************************/
void doMeasurement()
{
  // First read will be 0 (no distance  calculated yet)

  // Read the previous result (pause interrupts while doing so)
  noInterrupts();   // cli()
  for (int i = 0; i < tINT; i++)
  {
    distance[i] = travelTime[i] / 2.0 * (float)soundSpeed / 10000.0;   // in cm
  }
  interrupts();   // sei();

  // Initiate next trigger
  // digitalWrite(triggerPin, LOW);  // rest of loop already takes > 2µs
  // delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);    // HIGH pulse for at least 10µs
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);     // Set LOW again
}

/****************************************************************
      INTERRUPT handling
****************************************************************/
// INTerrupt 0 (pin 2 on Uno)
void call_INT0()
{
  byte pinRead;
  // pinRead = digitalRead(2);      // Slower ; Read pin 2
  pinRead = PIND >> 2 & B00000001;  // Faster ; Read pin 2/PD2
  interruptHandler(pinRead, 0);
}

// INTerrupt 1 (pin 3 on Uno)
void call_INT1()
{
  byte pinRead;
  // pinRead = digitalRead(3);      // Slower ; Read pin 3
  pinRead = PIND >> 3 & B00000001;  // Faster ; Read pin 3/PD3
  interruptHandler(pinRead, 1);
}

// Common function for interrupts
void interruptHandler(bool pinState, int nIRQ)
{
  unsigned long currentTime = micros();  // Get current time (in µs)
  if (pinState)
  {
    // If pin state has changed to HIGH -> remember start time (in µs)
    startTime[nIRQ] = currentTime;
  }
  else
  {
    // If pin state has changed to LOW -> calculate time passed (in µs)
    travelTime[nIRQ] = currentTime - startTime[nIRQ];
  }
}
// END
