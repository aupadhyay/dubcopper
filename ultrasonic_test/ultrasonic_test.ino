#define trigPin1 2
#define echoPin1 3
#define trigPin2 4
#define echoPin2 5
#define trigPin3 7
#define echoPin3 8

int trigPins[] = {10};
int echoPins[]= {9};
int len = 1;

void setup() {
  Serial.begin (9600);
  for (int i=0; i<len; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
//  pinMode(trigPin2, OUTPUT);
//  pinMode(echoPin2, INPUT);
//  pinMode(trigPin3, OUTPUT);
//  pinMode(echoPin3, INPUT);
}

void loop() {
  float RightSensor = distance(trigPins[0], echoPins[0]);
  
//  SonarSensor(trigPin2, echoPin2);
//  LeftSensor = distance;
//  SonarSensor(trigPin3, echoPin3);
//  FrontSensor = distance;
  
//  Serial.print(LeftSensor);
//  Serial.print(" - ");
//  Serial.print(FrontSensor);
//  Serial.print(" - ");
//  Serial.println(RightSensor);
}

float distance(int trigPin,int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  //float distance = (duration/2) / 29.1;
  float distance = (duration / 2) * 0.0343;

  Serial.print("detected object ");
  Serial.print(distance);
  Serial.println(" cm away");
  
  return distance;
}
