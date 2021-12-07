#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "controller.h"
#include "lsm9ds1.h"
#include "nrf_delay.h"
 
#define SIMULATION false
 
#if SIMULATION
#include "simulatorFunctions.h"
#else
#include "kobukiSensorTypes.h"
#include "display.h"
#endif
 
// // Configure initial state
// KobukiSensors_t sensors = {0};
 
// // You may need to add additional variables to keep track of state here
// uint16_t forwardDist = 0;
// uint16_t backDist = 0;
// int rotateDirection = -1;
// bool cliff_right = false;
// float uphill_orientation = 0;
// bool uphill = true;
// float min_tilt = 100000;
// bool overshoot = false;
// bool find_uphill = false;
 
// int rotate_speed = 50;
// int drive_speed = 125;

// float arr[10];
// int count;

// static float avg(float* arr, int size) {
//   float total = 0;
//   for (int i= 0; i < size; i++) {
//     total += arr[i];
//   }
//   return total / size;
// }

 
// // Return distance traveled between two encoder values
// static float measure_distance(uint16_t current, uint16_t previous) {
//   const float CONVERSION = 0.0006108;
 
//   // Your code here
//   uint32_t adjusted = 0;
//   adjusted += current;
//   if (current < previous) {
//   adjusted += (1 << 16);
//   }
//   return (current - previous) * CONVERSION;
// }

// static int obstacle(KobukiSensors_t sensors) {
// // if (sensors.bump_wheelDrops.bumpLeft) {
// // return 0;
// // } else if (sensors.bump_wheelDrops.bumpRight) {
// // return 2;
// // } else if (sensors.bump_wheelDrops.bumpCenter) {
// // return 1;
// // } return -1;
// return -1;
// }
 
// // Return true if a cliff has been seen
// // Save information about which cliff
// static bool check_and_save_bump(KobukiSensors_t* sensors, bool* obstacle_is_right) {
//   // Your code here
//   // duplicate header
//   return false;
//   // *cliff_is_right = sensors->cliffRight;
//   // return sensors->cliffLeft || sensors->cliffCenter || sensors->cliffRight;
// }
 
// // Return true if a cliff has been seen
// // Save information about which cliff
// static bool check_cliff(KobukiSensors_t* sensors, bool* cliff_is_right) {
//   // Your code here
// *cliff_is_right = sensors->cliffRight;
// return sensors->cliffLeft || sensors->cliffCenter || sensors->cliffRight;
// }
 
// // Read accelerometer value and calculate and return tilt (along axis corresponding to climbing the hill)
// static float read_tilt() {
//   // Your code here
// lsm9ds1_measurement_t accel = lsm9ds1_read_accelerometer();
// return 180/M_PI * atan2(accel.y_axis, sqrt(accel.x_axis*accel.x_axis + accel.z_axis*accel.z_axis));
// }
 
// Robot controller
// State machine running on robot_state_t state
// This is called in a while loop


// sensor methods
// left_blocked (bool) uses read_front (int)
// front_blocked (bool) uses read_front (int)
// rotate_right() should update front and left sensor values (or orientation value)
// rotate_left() should update front and left sensor values (or orientation value)


// not storing: init_x, init_y (0, 0)
// store: curr_x, curr_y
// store: max_x, max_y
// store: min_x, min_y

// overlooked factors: top/bottom dist * autoadjust in all states
// // if detect top/bottom in any state decrease/increase resting float speed
// overlooked factors: 

// 0 is N, 1 is E, 2 is S, 3 is W

// state variables

// liftoff
int lift_speed = 90; // technically should be zero for 2D kabuku
int liftoff_counter = 0;
int LIFTOFF_TIME = 3000; // ms

// move_forward
int forward_speed = 75;
int forward_counter = 0;

// wall_left
int orientation = 0;
int num_turns = 0;
int curr_x = 0; int curr_y = 0;
int max_x = curr_x; int max_y = curr_y;
int min_x = curr_x; int min_y = curr_y;

// finished
int backward_counter = 0;

// lower
int lower_speed = 50;
int lower_counter = 0;

robot_state_t controller(robot_state_t state) {
 
  kobukiSensorPoll(&sensors);
  float tilt = read_tilt();

  switch(state) {
    case START: {
      if (is_button_pressed(&sensors)) {
        state = LIFTOFF;
        kobukiDriveDirect(lift_speed, lift_speed);
      } else {
        state = START;
        kobukiDriveDirect(0, 0);
      }
      break;
    }

    case LIFTOFF: {
      if (liftoff_counter >= LIFTOFF_TIME) {
        state = MOVE_FORWARD;
        kobukiDriveDirect(forward_speed, forward_speed);
      } else {
        state = LIFTOFF;
        kobukiDriveDirect(lift_speed, lift_speed);
        liftoff_counter += 10;
        nrf_delay_ms(10);
      }
      break;
    }

    case MOVE_FORWARD: {
      if (front_blocked()) {
        state = WALL_LEFT; 
        rotateRight();
        kobukiDriveDirect(forward_speed, forward_speed);
      } else {
        state = MOVE_FORWARD;
        kobukiDriveDirect(forward_speed, forward_speed);
        forward_counter += 10;
        nrf_delay_ms(10);
      }
      break;
    }

    case WALL_LEFT: {
      bool lblock = left_blocked();
      bool fblock = front_blocked();
      if (num_turns == 4 && fabs(curr_x) < .2 * fabs(max_x-min_x) && fabs(curr_y) < .2 * fabs(max_y-min_y)) {
        // move forward some amount and go to return/move_backward state
        state = FINISHED;
        rotateRight();
        kobukiDriveDirect(forward_speed, forward_speed);
      } else if (lblock && (!fblock)) {
        // maintain distance and stay in WALL_LEFT
        state = WALL_LEFT;
        kobukiDriveDirect(forward_speed, forward_speed);
        curr_x = curr_x + (orientation % 2) * (2 - orientation);
        curr_y = curr_y + ((orientation + 1) % 2) * (1 - orientation);
        max_x = fmax(curr_x, max_x);
        min_x = fmin(curr_x, min_x);
        max_y = fmax(curr_y, max_y);
        min_y = fmin(curr_y, min_y);
      } else if (lblock && fblock) {
        // rotate_right() and transition to WALL_LEFT
        state = BLOCKED;
        rotateRight(); num_turns += 1;
        kobukiDriveDirect(forward_speed, forward_speed);
      } else if ((!lblock) && (!fblock)) {
        // turn left and transition to WALL_LEFT
        state = OPEN;
        rotateLeft(); num_turns -= 1;
        kobukiDriveDirect(forward_speed, forward_speed);
      }
      break;
    }

    case OPEN: {
      bool lblock = left_blocked();
      bool fblock = front_blocked();
      if (!lblock) {
        state = OPEN;
        kobukiDriveDirect(forward_speed, forward_speed);
        curr_x = curr_x + (orientation % 2) * (2 - orientation);
        curr_y = curr_y + ((orientation + 1) % 2) * (1 - orientation);
        max_x = fmax(curr_x, max_x);
        min_x = fmin(curr_x, min_x);
        max_y = fmax(curr_y, max_y);
        min_y = fmin(curr_y, min_y);
      } else {
        state = WALL_LEFT;
        kobukiDriveDirect(forward_speed, forward_speed);
      }
      break;
    }

    case BLOCKED: {
      // bool lblock = left_blocked();
      // bool fblock = front_blocked();
      state = WALL_LEFT;
      kobukiDriveDirect(forward_drive, forward_drive);
      break;
    }

    case FINISHED: {
      if (backward_counter >= forward_counter) {
        state = LOWER; 
        kobukiDriveDirect(lower_speed, lower_speed);
      } else {
        state = FINISHED;
        kobukiDriveDirect(forward_speed, forward_speed);
        backward_counter += 10;
        nrf_delay_ms(10);
      }
      break;
    }

    case LOWER: {
      if (lower_counter >= LIFTOFF_TIME * 1.25) {
        state = TERMINATE;
        kobukiDriveDirect(0, 0);
      } else {
        state = LOWER;
        kobukiDriveDirect(lower_speed, lower_speed);
        lower_counter += 10;
        nrf_delay_ms(10);
      }
      break;
    }

    case TERMINATE: {
      kobukiDriveDirect(0, 0);
      break;
    }
  }

  // // nrf_delay_ms(1); // in while loop?
 
  // // handle states
  // switch(state) {
  //   case OFF: {
  //     printf("OFF\n");
 
  //     // transition logic
  //     if (is_button_pressed(&sensors)) {
  //       state = FIND_UPHILL;
  //       lsm9ds1_start_gyro_integration();
  //       kobukiDriveDirect(-rotate_speed,rotate_speed);
  //     } else {
  //       state = OFF;
  //       // perform state-specific actions here
  //       kobukiDriveDirect(0, 0);
  //     }
  //     break; // each case needs to end with break!
  //   }
 
  //   case FIND_UPHILL: {
  //     printf("Looking for uphill\n");
  //     find_uphill = true;
  //     lsm9ds1_measurement_t val = lsm9ds1_read_gyro_integration();
  //     printf("gyro val: %f, orientation: %f, tilt: %f", val.z_axis, uphill_orientation, read_tilt());
  //     // transition logic
  //     if (is_button_pressed(&sensors)) {
  //       lsm9ds1_stop_gyro_integration();
  //       state = OFF;
  //       kobukiDriveDirect(0, 0);
  //     } else if (check_cliff(&sensors, &cliff_right)) {
  //         // default cliff backup
  //       lsm9ds1_stop_gyro_integration();
  //       state = BACKUP;
  //       // nrf_delay_ms(200);
  //       // kobukiSensorPoll(&sensors);
  //       backDist = sensors.leftWheelEncoder;
  //       kobukiDriveDirect(-rotate_speed, -rotate_speed);
  //     } else if (fabs(val.z_axis) >= 360) {
  //     find_uphill = false;
  //       lsm9ds1_stop_gyro_integration();
  //       lsm9ds1_start_gyro_integration();
  //       state = ROTATE_TO_UPHILL;
  //       kobukiDriveDirect(-rotate_speed, rotate_speed);
  //       //not sure if need to restart gyro integration
  //     } else {
  //       if (tilt < min_tilt) {
  //         uphill_orientation = val.z_axis;
  //         min_tilt = tilt;
  //       }
  //       char buf[16];
  //       snprintf(buf, 16, "%f", val.z_axis);
  //       display_write(buf, DISPLAY_LINE_0);
  //       state = FIND_UPHILL;
  //       kobukiDriveDirect(-rotate_speed, rotate_speed);
  //     }
  //     break; // each case needs to end with break!
  //   }
 
  //   case ROTATE_TO_UPHILL: {
  //     printf("Turning to uphill\n");

  //     lsm9ds1_measurement_t val = lsm9ds1_read_gyro_integration();
  //     printf("Gyro: %f, Desired orientation: %f", val.z_axis, uphill_orientation);
  //     if (is_button_pressed(&sensors)) {
  //       lsm9ds1_stop_gyro_integration();
  //       state = OFF;
  //       kobukiDriveDirect(0, 0);
  //     } else if (check_cliff(&sensors, &cliff_right)) {
  //         // default cliff backup
  //       printf("BACKUP");
  //       lsm9ds1_stop_gyro_integration();
  //       state = BACKUP;
  //       // nrf_delay_ms(200);
  //       // kobukiSensorPoll(&sensors);
  //       backDist = sensors.leftWheelEncoder;
  //       kobukiDriveDirect(-drive_speed, -drive_speed);
       
       
       
       
  //     // } else if (val.z_axis < uphill_orientation + 1 && val.z_axis > uphill_orientation - 1) {
  //     // printf("OVERSHOTOTOTOT");
  //     //   lsm9ds1_stop_gyro_integration();
  //     //   state = OVERSHOOT;
  //     //   uphill = true;
  //     //   kobukiDriveDirect(drive_speed, drive_speed);
       
       
       
       
  //     } else if (fabs(val.z_axis) >= uphill_orientation) {
  //       // working in degrees, error of +/- 1 degree
  //       lsm9ds1_stop_gyro_integration();
  //       printf("FOUND UPHILL");
  //       state = DRIVING;
  //       count = 0;
  //       uphill = true;
  //       forwardDist = sensors.leftWheelEncoder;
  //       kobukiDriveDirect(drive_speed, drive_speed);
  //     } else {
  //       char buf[16];
  //       snprintf(buf, 16, "%f", val.z_axis);
  //       display_write(buf, DISPLAY_LINE_0);
  //       state = ROTATE_TO_UPHILL;
  //       kobukiDriveDirect(-rotate_speed, rotate_speed);
  //     }
  //     // printf("valZ: %f\n", fabs(val.z_axis));
  //     // printf("uphO: %f\n", uphill_orientation);
  //     break;
  //   }
 
  //   case BACKUP: {
  //     float currDist = measure_distance(sensors.leftWheelEncoder, backDist);
  //     printf("currDist: %f\n", currDist);
  //     char buf[16];
  //     snprintf(buf, 16, "%f", currDist);
  //     display_write(buf, DISPLAY_LINE_0);
  //     state = BACKUP;
 
  //     if (is_button_pressed(&sensors)) {
  //       lsm9ds1_stop_gyro_integration();
  //       state = OFF;
  //       kobukiDriveDirect(0, 0);
  //     // } else if (check_cliff(&sensors, &cliff_right)){
  //     //   lsm9ds1_stop_gyro_integration();
  //     //   state = BACKUP;
  //     //   backDist = sensors.leftWheelEncoder;
  //     //   kobukiDriveDirect(-rotate_speed, -rotate_speed);
  //     } else if (fabs(currDist) >= 0.2) {
  //       printf("cliff_right: %d\n", cliff_right);
  //    state = TURN45;
  //       currDist = 0;
       
  //       if (cliff_right) {
  //         kobukiDriveDirect(-drive_speed, drive_speed);
  //       } else {
  //         kobukiDriveDirect(drive_speed, -drive_speed);
  //       }
  //       lsm9ds1_start_gyro_integration();
  //     } else {
  //       char buf[16];
  //       snprintf(buf, 16, "%f", currDist);
  //       display_write(buf, DISPLAY_LINE_0);
  //       state = BACKUP;
  //       kobukiDriveDirect(-100, -100);
  //     }
  //     break;
  //   }
   
  //   case TURN45: {
  //     lsm9ds1_measurement_t val = lsm9ds1_read_gyro_integration();
  //     if (is_button_pressed(&sensors)) {
  //       lsm9ds1_stop_gyro_integration();
  //       state = OFF;
  //       kobukiDriveDirect(0, 0);
  //     // } else if (check_cliff(&sensors, &cliff_right)){
  //     //   lsm9ds1_stop_gyro_integration();
  //     //   state = BACKUP;
  //     //   backDist = sensors.leftWheelEncoder;
  //     //   kobukiDriveDirect(-drive_speed, -drive_speed);
  //     } else if (fabs(val.z_axis) >= 15) {
  //       lsm9ds1_stop_gyro_integration();
  //       if (find_uphill) {
  //         state = FIND_UPHILL;
  //         lsm9ds1_start_gyro_integration();
  //         kobukiDriveDirect(drive_speed, drive_speed);
  //       } else {
  //         state = DRIVING;
  //         count = 0;
  //         kobukiDriveDirect(drive_speed, drive_speed);
  //         forwardDist = sensors.leftWheelEncoder;
  //       }
  //       if (overshoot) {
  //         state = OVERSHOOT;
  //         kobukiDriveDirect(drive_speed, drive_speed);
  //       } else {
  //         state = DRIVING;
  //         count = 0;
  //         kobukiDriveDirect(drive_speed, drive_speed);
  //         forwardDist = sensors.leftWheelEncoder;
  //       }
  //     } else {
  //       state = TURN45;
  //       if (cliff_right) {
  //         kobukiDriveDirect(-drive_speed, drive_speed);
  //       } else {
  //         kobukiDriveDirect(drive_speed, -drive_speed);
  //       }
  //       char buf[16];
  //       snprintf(buf, 16, "%f", val.z_axis);
  //       display_write(buf, DISPLAY_LINE_0);
  //     }
  //     break;
  //   }
     
  //   case DRIVING: {
  //     float currDist = measure_distance(sensors.leftWheelEncoder, forwardDist);
  //     char buf[16];
  //     snprintf(buf, 16, "%s", "DRIVING");
  //     display_write(buf, DISPLAY_LINE_0);
 
  //     if (is_button_pressed(&sensors)) {
  //       state = OFF;
  //       kobukiDriveDirect(0, 0);
  //     } else if(count > 10 && uphill && fabs(avg(arr, 10)) <= 2) {
  //       printf("OVERSHOOOOTING");
  //       state = OVERSHOOT;
  //       uphill = false;
  //       forwardDist = sensors.leftWheelEncoder;
  //       char buf[16];
  //       snprintf(buf, 16, "%f", fabs(avg(arr, 10)));
  //       display_write(buf, DISPLAY_LINE_1);
  //       kobukiDriveDirect(drive_speed, drive_speed);
  //     } else if (check_cliff(&sensors, &cliff_right)) {
  //       state = BACKUP;
  //       // nrf_delay_ms(200);
  //       // kobukiSensorPoll(&sensors);
  //       backDist = sensors.leftWheelEncoder;
  //       kobukiDriveDirect(-100, -100);
  //     } else {
  //       state = DRIVING;
  //       printf("DRIVING");
  //       arr[count % 10] = read_tilt();
  //       count += 1;
  //       char buf[16];
  //       snprintf(buf, 16, "%d", count);
  //       display_write(buf, DISPLAY_LINE_1);
  //       kobukiDriveDirect(drive_speed, drive_speed);
  //     }
  //     break;
  //   }
     
  //   case OVERSHOOT: {
  //     float currDist = measure_distance(sensors.leftWheelEncoder, forwardDist);
  //     char buf[16];
  //     snprintf(buf, 16, "%s", "OVERSHOOT");
  //     display_write(buf, DISPLAY_LINE_0);
  //     overshoot = true;
      
  //     if (is_button_pressed(&sensors)) {
  //       state = OFF;
  //       kobukiDriveDirect(0, 0);
  //     } else if (check_cliff(&sensors, &cliff_right)) {
  //       state = BACKUP;
  //       // nrf_delay_ms(200);
  //       // kobukiSensorPoll(&sensors);
  //       backDist = sensors.leftWheelEncoder;
  //       kobukiDriveDirect(-100, -100);
  //     } else if(currDist >= 0.3) {
  //       state = TURN180;
  //       overshoot = false;
  //       uphill = false;
  //       kobukiDriveDirect(rotate_speed, -rotate_speed);
  //       lsm9ds1_start_gyro_integration();
  //     } else {
  //       state = OVERSHOOT;
  //       kobukiDriveDirect(drive_speed, drive_speed);
  //     }
  //     break;
  //   }
     
  //   case TURN180: {
  //     lsm9ds1_measurement_t val = lsm9ds1_read_gyro_integration();
  //     if (is_button_pressed(&sensors)) {
  //       lsm9ds1_stop_gyro_integration();
  //       state = OFF;
  //       kobukiDriveDirect(0, 0);
  //     } else if (check_cliff(&sensors, &cliff_right)){
  //       lsm9ds1_stop_gyro_integration();
  //       state = BACKUP;
  //       // nrf_delay_ms(200);
  //       // kobukiSensorPoll(&sensors);
  //       backDist = sensors.leftWheelEncoder;
  //       kobukiDriveDirect(-drive_speed, -drive_speed);
  //     } else if (fabs(val.z_axis) >= 180) {
  //       lsm9ds1_stop_gyro_integration();
  //       state = DRIVING;
  //       count = 0;
  //       kobukiDriveDirect(drive_speed, drive_speed);
  //       forwardDist = sensors.leftWheelEncoder;
  //     } else {
  //       state = TURN180;
  //       kobukiDriveDirect(rotate_speed, -rotate_speed);
  //     }
  //     break;
  //   }
  // }
  // return state;
}
