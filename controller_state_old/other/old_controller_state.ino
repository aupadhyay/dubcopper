#include <NewPing.h>

// configs
bool point_delay = false;

// state variables
enum state_enum {
  START,
  LIFTOFF,
  MOVE_FORWARD,
  WALL_LEFT,
  OPEN, // LEFT, FRONT not blocked
  BLOCKED, // LEFT, FRONT blocked
  // BLOCKED_FRONT, // LEFT not blocked, FRONT blocked
  FINISHED,
  LOWER,
  TERMINATE
};
state_enum state = START;

enum motor_enum {
  OFF,
  LIFT,
  FORWARD,
  BRAKE, // not yet used
  DROP
};
motor_enum motor = OFF;

// ultrasonic sensor setup
#define TRIGGER_PIN 8
#define ECHO_PIN 9
#define MAX_DISTANCE 300

int trigPin0 = 8;
int echoPin0 = 9;

int trigPin1 = 10;
int echoPin1 = 11;

float front_sensor_dist; //*
float left_sensor_dist; //*

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonar0(trigPin0, echoPin0, MAX_DISTANCE);
NewPing sonar1(trigPin1, echoPin1, MAX_DISTANCE);

void measureSensor(float* var) {
  int uS = -1;
  if (var == &front_sensor_dist) {
    uS = sonar0.ping();
  } else if (var == &left_sensor_dist) {
    uS = sonar1.ping();
  }
  if (uS == 0) {
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
    &var = uS / US_ROUNDTRIP_CM;
    Serial.print(&var);
    Serial.println("cm");
  }
}

// start
bool armed = false;
bool lblock = false;
bool fblock = false;
const int off_speed = 0;

// liftoff

int liftoff_counter = 0;
const int lift_speed = 90; // technically should be zero for 2D kabuku
const int LIFTOFF_TIME = 3000; // ms

// move_forward
int forward_counter = 0;
const int forward_speed = 75;

// wall_left
int orientation = 0;
int num_turns = 0;
int curr_x = 0; int curr_y = 0;
int max_x = curr_x; int max_y = curr_y;
int min_x = curr_x; int min_y = curr_y;

// finished
int backward_counter = 0;

// lower
const int lower_speed = 50;
int lower_counter = 0;

// methods
void motorUpdate(motor_enum m) {}
void pointUpdate(int x_val, int y_val) {
  Serial.print(state); Serial.print(": ");
  Serial.print(x_val); Serial.print(", ");
  Serial.println(y_val);
}
void rotateRight() {
  orientation = (orientation + 1) % 4;
}
void rotateLeft() {
  orientation = (orientation + 3) % 4;
}

bool frontBlocked() {
  measureSensor(&front_sensor_dist);
  if (front_sensor_dist < 20) {
    return true;
  } else {
    return false;
  }
}
  
bool leftBlocked() {
  measureSensor(&left_sensor_dist);
  if (left_sensor_dist < 20) {
    return true;
  } else {
    return false;
  }
}

// rotateRight()
// rotateLeft()
// frontBlocked()
// rightBlocked()
// pointUpdate(bool output)
//getSpeed() // use IMU for speed

void setup() {
  // setup
  Serial.begin(9600);
  while(!Serial);
  armed = true;

}

void loop() {
  // main
  lblock = leftBlocked(); fblock = frontBlocked();

  switch (state) {
    case START:
      if (armed) { // waits for serial input to start
        state = LIFTOFF;
        motor = LIFT; motorUpdate(motor);
      } else {
        state = START;
        motor = OFF; motorUpdate(motor);
      }
      break;

    case LIFTOFF:
      if (liftoff_counter >= LIFTOFF_TIME) {
        state = MOVE_FORWARD;
        motor = FORWARD; motorUpdate(motor);
      } else {
        state = LIFTOFF;
        motor = LIFT; motorUpdate(motor);
        liftoff_counter += 10;
        if (point_delay) delay(10);
      }
      break;

    case MOVE_FORWARD:
      if (frontBlocked()) {
        state = WALL_LEFT; 
        rotateRight();
        motor = FORWARD; motorUpdate(motor);
      } else {
        state = MOVE_FORWARD;
        motor = FORWARD; motorUpdate(motor);
        forward_counter += 10;
        if (point_delay) delay(10);
      }
      break;

    case WALL_LEFT:
      bool lblock = leftBlocked();
      bool fblock = frontBlocked();
      if (num_turns == 4 && abs(curr_x) < .2 * abs(max_x-min_x) && abs(curr_y) < .2 * abs(max_y-min_y)) {
        // move forward some amount and go to return/move_backward state
        state = FINISHED;
        rotateRight();
        motor = FORWARD; motorUpdate(motor);

      } else if (lblock && fblock) {
        // rotateRight() and transition to BLOCKED
        state = BLOCKED;
        rotateRight(); num_turns += 1;
        motor = FORWARD; motorUpdate(motor);
        pointUpdate(curr_x, curr_y);
      } else if ((!lblock) && (!fblock)) {
        // rotateLeft() and transition to OPEN
        state = OPEN;
        rotateLeft(); num_turns -= 1;
        motor = FORWARD; motorUpdate(motor);
        pointUpdate(curr_x, curr_y);
      } else {
        // maintain distance and stay in WALL_LEFT
        state = WALL_LEFT; // lblock && (!fblock)
        if ((!lblock) && fblock) { // BLOCKED_FRONT case
          rotateRight(); num_turns += 1; 
        }
        motor = FORWARD; motorUpdate(motor);
        curr_x = curr_x + (orientation % 2) * (2 - orientation);
        curr_y = curr_y + ((orientation + 1) % 2) * (1 - orientation);
        max_x = max(curr_x, max_x); min_x = min(curr_x, min_x);
        max_y = max(curr_y, max_y); min_y = min(curr_y, min_y);
        pointUpdate(curr_x, curr_y);
      }
      break;

    case OPEN:
      if (!lblock) {
        state = OPEN;
        motor = FORWARD; motorUpdate(motor);
        curr_x = curr_x + (orientation % 2) * (2 - orientation);
        curr_y = curr_y + ((orientation + 1) % 2) * (1 - orientation);
        max_x = max(curr_x, max_x); min_x = min(curr_x, min_x);
        max_y = max(curr_y, max_y); min_y = min(curr_y, min_y);
        pointUpdate(curr_x, curr_y);
      } else {
        state = WALL_LEFT;
        motor = FORWARD; motorUpdate(motor);
        pointUpdate(curr_x, curr_y);
      }
      break;

    case BLOCKED:
      // bool lblock = leftBlocked();
      // bool fblock = frontBlocked();
      state = WALL_LEFT;
      motor = FORWARD; motorUpdate(motor);
      break;

    case FINISHED:
      if (backward_counter >= forward_counter) {
        state = LOWER; 
        motor = DROP; motorUpdate(motor);
      } else {
        state = FINISHED;
        motor = FORWARD; motorUpdate(motor);
        backward_counter += 10;
        if (point_delay) delay(10);
      }
      break;

    case LOWER:
      if (lower_counter >= LIFTOFF_TIME * 1.25) {
        state = TERMINATE;
        motor = OFF; motorUpdate(motor);
      } else {
        state = LOWER;
        motor = DROP; motorUpdate(motor);
        lower_counter += 10;
        if (point_delay) delay(10);
      }
      break;

    case TERMINATE:
      motor = OFF; motorUpdate(motor);
      break;
  }

  delay(20);
}
