#include "kobukiSensorTypes.h"

// Robot states
// Add your own states here
typedef enum {
  START,
  LIFTOFF,
  MOVE_FORWARD,
  WALL_LEFT,
  OPEN,
  BLOCKED,
  FINISHED,
  LOWER,
  TERMINATE,
} robot_state_t;