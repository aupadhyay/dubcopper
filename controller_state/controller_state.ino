// state variables
enum state_enum {
  START,
  LIFTOFF,
  MOVE_FORWARD,
  WALL_LEFT,
  OPEN,
  BLOCKED,
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

bool frontBlocked() {}
bool leftBlocked() {}

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
        delay(10);
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
        delay(10);
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
        // rotate_right() and transition to WALL_LEFT
        state = BLOCKED;
        rotateRight(); num_turns += 1;
        motor = FORWARD; motorUpdate(motor);
        pointUpdate(curr_x, curr_y);
      } else if ((!lblock) && (!fblock)) {
        // turn left and transition to WALL_LEFT
        state = OPEN;
        rotateLeft(); num_turns -= 1;
        motor = FORWARD; motorUpdate(motor);
        pointUpdate(curr_x, curr_y);
      } else {
        // maintain distance and stay in WALL_LEFT
        state = WALL_LEFT; // lblock && (!fblock)
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
        delay(10);
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
        delay(10);
      }
      break;

    case TERMINATE:
      motor = OFF; motorUpdate(motor);
      break;
  }
}
