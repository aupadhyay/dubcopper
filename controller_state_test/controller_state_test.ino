#include <NewPing.h>

#define MAX_DISTANCE 300

//* change for drone

// configs
bool point_delay = false;

// start
bool armed = false;
bool lblock = false;
bool fblock = false;
const int off_speed = 0;

// liftoff
int liftoff_counter = 0;
const int lift_speed = 90; // technically should be zero for 2D kabuku
const int LIFTOFF_TIME = 0; // 3000 ms

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

// ultrasonic sensor setup
const int numSensors = 4;
const float blockThreshold = 50;

int trigPins[] = {12, 0, 2, 4}; //*
int echoPins[] = {13, 1, 3, 5}; //*

float sensorDists[] = {0.0, 0.0, 0.0, 0.0};

NewPing sonar0(trigPins[0], echoPins[0], MAX_DISTANCE);
NewPing sonar1(trigPins[1], echoPins[1], MAX_DISTANCE);
NewPing sonar2(trigPins[2], echoPins[2], MAX_DISTANCE);
NewPing sonar3(trigPins[3], echoPins[3], MAX_DISTANCE);

NewPing sonars[] = {sonar0, sonar1, sonar2, sonar3};


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
} state = START;

enum motor_enum {
  OFF,
  LIFT,
  FORWARD,
  BRAKE, // not yet used
  DROP
} motor = OFF;

int front() { return orientation % 4; }
int right() { return (orientation + 1) % 4; }
int back()  { return (orientation + 3) % 4; }
int left()  { return (orientation + 2) % 4; }

void setup() {
  Serial.begin(9600);
  while(!Serial);
  armed = true;

  for (int i = 0; i < numSensors; i++) {
    resetSensor(echoPins[i]);
  }
}

void loop() {
  Serial.println("testing");

  switch (state) {
    case START: Serial.println("START"); break;
    case LIFTOFF: Serial.println("LIFTOFF"); break;
    case MOVE_FORWARD: Serial.println("MOVE_FORWARD"); break;
    case WALL_LEFT: Serial.println("WALL_LEFT"); break;
    case OPEN: Serial.println("OPEN"); break;
    case BLOCKED: Serial.println("BLOCKED"); break;
    case FINISHED: Serial.println("FINISHED"); break;
    case LOWER: Serial.println("LOWER"); break;
    case TERMINATE: Serial.println("TERMINATE"); break; 
  }
  

  lblock = leftBlocked(); 
  fblock = frontBlocked();


  switch (state) {
    
    case START: {
      if (armed) { // waits for serial input to start
        state = LIFTOFF;
        motor = LIFT; motorUpdate(motor);
      } else {
        state = START;
        motor = OFF; motorUpdate(motor);
      }
      break;
    }

    case LIFTOFF: {
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
    }

    case MOVE_FORWARD: {
      if (fblock) {
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
    }

    case WALL_LEFT: {
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
        // pointUpdate(curr_x, curr_y);
      } else if ((!lblock) && (!fblock)) {
        // rotateLeft() and transition to OPEN
        state = OPEN;
        rotateLeft(); num_turns -= 1;
        motor = FORWARD; motorUpdate(motor);
        // pointUpdate(curr_x, curr_y);
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
    }

    case OPEN: {
      if (lblock && fblock) {
        state = BLOCKED;
        motor = FORWARD; motorUpdate(motor);
        // pointUpdate(curr_x, curr_y);
      } else if (lblock) {
        state = WALL_LEFT; // case: entering wall_left off left turn
        motor = FORWARD; motorUpdate(motor);
        // pointUpdate(curr_x, curr_y);
      } else if (fblock) {
        state = WALL_LEFT; // case: readjustment from wall_left to wall_left
        rotateRight(); num_turns += 1;
        motor = FORWARD; motorUpdate(motor);
        // pointUpdate(curr_x, curr_y);
      } else {
        state = WALL_LEFT;
        motor = FORWARD; motorUpdate(motor);
        curr_x = curr_x + (orientation % 2) * (2 - orientation);
        curr_y = curr_y + ((orientation + 1) % 2) * (1 - orientation);
        max_x = max(curr_x, max_x); min_x = min(curr_x, min_x);
        max_y = max(curr_y, max_y); min_y = min(curr_y, min_y);
        pointUpdate(curr_x, curr_y);
      }
      break;
      // if (!lblock) {
      //   state = OPEN;
      //   motor = FORWARD; motorUpdate(motor);
      //   curr_x = curr_x + (orientation % 2) * (2 - orientation);
      //   curr_y = curr_y + ((orientation + 1) % 2) * (1 - orientation);
      //   max_x = max(curr_x, max_x); min_x = min(curr_x, min_x);
      //   max_y = max(curr_y, max_y); min_y = min(curr_y, min_y);
      //   pointUpdate(curr_x, curr_y);
      // } else if (!fblock) {
        
      
      // } else {
      //   state = WALL_LEFT;
      //   motor = FORWARD; motorUpdate(motor);
      //   // pointUpdate(curr_x, curr_y);
      // }
      // break;
    }

    case BLOCKED: {
      // bool lblock = leftBlocked();
      // bool fblock = frontBlocked();
      state = WALL_LEFT;
      motor = FORWARD; motorUpdate(motor);
      break;
    }

    case FINISHED: {
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
    }

    case LOWER: {
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
    }

    case TERMINATE: {
      state = START; armed = false; // disarmed and idle in start
      motor = OFF; motorUpdate(motor);
      break;
    }
  }

  // pointUpdate(curr_x, curr_y);
  delay(200);
}

void resetSensor(int index) {
  Serial.print("MAX: resetting sensor - echo: ");
  int echoPin = echoPins[index];
  Serial.println(echoPin);
  pinMode(echoPin, OUTPUT);   delay(50);
  digitalWrite(echoPin, LOW); delay(50);
  pinMode(echoPin, INPUT);    delay(50);
}
void measureSensor(int index) {
  float dist = sonars[index].ping_cm();
  if (dist == 0) {
    resetSensor(echoPins[index]); delay(15);
    dist = sonars[index].ping_cm();
  }
  sensorDists[index] = dist;
  Serial.print("DIST: "); Serial.print(index); Serial.print(" "); Serial.println(dist);
}

void motorUpdate(motor_enum m) {} //*

void pointUpdate(int x_val, int y_val) {
  Serial.print("(");
  Serial.print(x_val);
  Serial.print(", ");
  Serial.print(y_val);
  Serial.println(")");
//  Serial.print(state); Serial.print(": ");
//  Serial.print(x_val); Serial.print(", ");
//  Serial.println(y_val);
}

void rotateRight() {
  orientation = (orientation + 1) % 4;
}

void rotateLeft() {
  orientation = (orientation + 3) % 4;
}

bool frontBlocked() {
  int index = front();
  measureSensor(index);
  float dist = sensorDists[index];
  return dist < blockThreshold && dist > 0;
}

bool leftBlocked() {
  int index = left();
  measureSensor(index);
  float dist = sensorDists[index];
  return dist < blockThreshold;
}
