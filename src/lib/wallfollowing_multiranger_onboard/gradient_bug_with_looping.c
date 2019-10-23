/*
 * com_bug_with_looping.c
 *
 *  Created on: Nov 8, 2018
 *      Author: knmcguire
 */
#include "gradient_bug_with_looping.h"
#include "wallfollowing_multiranger_onboard.h"
//#include "median_filter.h"

#include <math.h>
#include <stdlib.h>

#ifndef GB_ONBOARD
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#else
#include "usec_time.h"
#endif

#ifndef GB_ONBOARD
struct timeval state_start_time;
struct timeval now_time;
#else
float state_start_time;
#endif

//static variables only used for initialization
static bool first_run = true;
static float ref_distance_from_wall = 0;
static float max_speed = 0.5;

#ifndef GB_ONBOARD
uint8_t rssi_threshold = 55;// normal batteries 50/52/53/53 bigger bbatteries 55/57/59
uint8_t rssi_collision_threshold = 46; // normal batteris 43/45/45/46 bigger batteries 48/50/52
#else
uint8_t rssi_threshold = 59;// normal batteries 50/52/53/53 bigger bbatteries 55/57/59
uint8_t rssi_collision_threshold = 54; // normal batteris 43/45/45/46 bigger batteries 48/50/52

#endif
// Converts degrees to radians.
#define deg2rad(angleDegrees) (angleDegrees * (float)M_PI / 180.0f)

// Converts radians to degrees.
#define rad2deg(angleRadians) (angleRadians * 180.0f / (float)M_PI)

//struct MedianFilterInt medFiltdiffRssi;

#ifndef GB_ONBOARD

// Helper functions
static int diff_ms(struct timeval t1, struct timeval t2)
{
  return (((t1.tv_sec - t2.tv_sec) * 1000000) +
          (t1.tv_usec - t2.tv_usec)) / 1000;
}
#endif

static int transition(int new_state)
{
#ifndef GB_ONBOARD
  gettimeofday(&state_start_time, NULL);
#else
  float t =  usecTimestamp() / 1e6;
  state_start_time = t;
#endif

  return new_state;

}



// Static helper functions
static bool logicIsCloseTo(float real_value, float checked_value, float margin)
{
  if (real_value > checked_value - margin && real_value < checked_value + margin) {
    return true;
  } else {
    return false;
  }
}

static float wraptopi(float number)
{
  if (number > (float)M_PI) {
    return (number - (float)(2 * M_PI));
  } else if (number < (float)(-1 * M_PI)) {
    return (number + (float)(2 * M_PI));
  } else {
    return (number);
  }

}


// Command functions
static void commandTurn(float *vel_w, float max_rate)
{
  *vel_w = max_rate;
}

/*
// heading stuff
static float meanAngle (double *angles, int size)
{
  double y_part = 0, x_part = 0;
  int i;

  for (i = 0; i < size; i++)
    {
      x_part += cos (angles[i] * M_PI / 180);
      y_part += sin (angles[i] * M_PI / 180);
    }

  return atan2 (y_part / size, x_part / size) * 180 / M_PI;
}
*/

uint8_t maxValue(uint8_t myArray[], int size)
{
  /* enforce the contract */
  //assert(myArray && size);
  int i;
  uint8_t maxValue = myArray[0];

  for (i = 1; i < size; ++i) {
    if (myArray[i] > maxValue) {
      maxValue = myArray[i];
    }
  }
  return maxValue;
}

static float fillHeadingArray(uint8_t *correct_heading_array, float rssi_heading, int diff_rssi, int max_meters)
{

	//Heading array of action choices
  static float heading_array[8] = { -135.0f, -90.0f, -45.0f, 0.0f, 45.0f, 90.0f, 135.0f, 180.0f};
  float rssi_heading_deg = rad2deg(rssi_heading);
  //printf("%f \n", rssi_heading_deg);

  for (int it = 0; it < 8; it++) {

	  // Fill array based on heading and rssi heading
    if ((rssi_heading_deg >= heading_array[it] - 22.5f && rssi_heading_deg < heading_array[it] + 22.5f && it != 7) || (
          it == 7 && (rssi_heading_deg >= heading_array[it] - 22.5f || rssi_heading_deg < -135.0f - 22.5f))) {
      uint8_t temp_value_forward = correct_heading_array[it];
      uint8_t temp_value_backward = correct_heading_array[(it + 4) % 8];

      // if gradient is good, increment the array corresponding to the current heading and decrement the exact opposite
      if (diff_rssi > 0) {
        correct_heading_array[it] = temp_value_forward + 1; //(uint8_t)abs(diff_rssi);
        if (temp_value_backward > 0) {
          correct_heading_array[(it + 4) % 8] = temp_value_backward - 1;  //(uint8_t)abs(diff_rssi);
        }
        // if gradient is bad, decrement the array corresponding to the current heading and increment the exact opposite

      } else if (diff_rssi < 0) {
        if (temp_value_forward > 0) {
          correct_heading_array[it] = temp_value_forward - 1;  //(uint8_t)abs(diff_rssi);
        }
        correct_heading_array[(it + 4) % 8] = temp_value_backward + 1; //(uint8_t)abs(diff_rssi);
      }

    }
  }

// degrading function
  //    If one of the arrays goes over maximum amount of points (meters), then decrement all values
  if (maxValue(correct_heading_array, 8) > max_meters) {
    for (int it = 0; it < 8; it++) {
      if (correct_heading_array[it] > 0) {
        correct_heading_array[it] = correct_heading_array[it] - 1;
      }
    }
  }


  // Calculate heading where the beacon might be
  int count = 0;
  float y_part = 0, x_part = 0;

  for (int it = 0; it < 8; it++) {
    if (correct_heading_array[it] > 0) {
      x_part += (float)correct_heading_array[it] * (float)cos(heading_array[it] * (float)M_PI / 180.0f);
      y_part += (float)correct_heading_array[it] * (float)sin(heading_array[it] * (float)M_PI / 180.0f);

      //sum += heading_array[it];
      count = count + correct_heading_array[it];

    }
  }
  float wanted_angle_return = 0;
  if (count != 0) {
    wanted_angle_return = atan2(y_part / (float)count, x_part / (float)count);
  }


  return wanted_angle_return;

}

// statemachine functions
static float wanted_angle = 0;

void init_gradient_bug_loop_controller(float new_ref_distance_from_wall, float max_speed_ref,
                                       float begin_wanted_heading)
{
  ref_distance_from_wall = new_ref_distance_from_wall;
  max_speed = max_speed_ref;
  wanted_angle = begin_wanted_heading;
  //init_median_filter_i(&medFiltdiffRssi,101);
  first_run = true;
}


int gradient_bug_loop_controller(float *vel_x, float *vel_y, float *vel_w, float *rssi_angle, int *state_wallfollowing,
                                 float front_range, float left_range, float right_range, float back_range,
                                 float current_heading, float current_pos_x, float current_pos_y, uint8_t rssi_beacon,
                                 uint8_t rssi_inter, float rssi_angle_inter, bool priority, bool outbound)
{

  // Initalize static variables
  static int state = 2;
  //static float previous_heading = 0;
  static int state_wf = 0;
  static float wanted_angle_dir = 0;
  static float pos_x_hit = 0;
  static float pos_y_hit = 0;
  static float pos_x_sample = 0;
  static float pos_y_sample = 0;
  //static float pos_x_move = 0;
  //static float pos_y_move = 0;
  static bool overwrite_and_reverse_direction = false;
  static float direction = 1;
  static bool cannot_go_to_goal = false;
  static uint8_t prev_rssi = 150;
  static int diff_rssi = 0;
  static bool rssi_sample_reset = false;
  static float heading_rssi = 0;
  static uint8_t correct_heading_array[8] = {0};

  static bool first_time_inbound = true;
  static float wanted_angle_hit = 0;

  /*
  #ifndef GB_ONBOARD
    gettimeofday(&now_time,NULL);
  #else
    float now = usecTimestamp() / 1e6;
  #endif
  */


  // if it is reinitialized
  if (first_run) {

    wanted_angle_dir = wraptopi(current_heading - wanted_angle); // to determine the direction when turning to goal

    overwrite_and_reverse_direction = false;
    state = 2;
#ifndef GB_ONBOARD
    gettimeofday(&state_start_time, NULL);
#else
    float t =  usecTimestamp() / 1e6;
    state_start_time = t;
#endif
    first_run = false;
  }

  if (first_time_inbound) {
    wraptopi(wanted_angle - 3.14f);
    wanted_angle_dir = wraptopi(current_heading - wanted_angle);
    state = transition(2);
    first_time_inbound = false;
  }

  /***********************************************************
   * State definitions
   ***********************************************************/
  // 1 = forward
  // 2 = rotate_to_goal
  // 3 = wall_following
  // 4 = move out of way

  /***********************************************************
   * Handle state transitions
   ***********************************************************/

  if (state == 1) {     //FORWARD
    if (front_range < ref_distance_from_wall + 0.2f) {

// if looping is detected, reverse direction (only on outbound)
      if (overwrite_and_reverse_direction) {
        direction = -1.0f * direction;
        overwrite_and_reverse_direction = false;
      } else {
        if (left_range < right_range && left_range < 2.0f) {
          direction = -1.0f;
        } else if (left_range > right_range && right_range < 2.0f) {
          direction = 1.0f;

        } else if (left_range > 2.0f && right_range > 2.0f) {
          direction = 1.0f;
        } else {

        }
      }

      pos_x_hit = current_pos_x;
      pos_y_hit = current_pos_y;
      wanted_angle_hit = wanted_angle;

      wall_follower_init(0.4, 0.5);

      for (int it = 0; it < 8; it++) { correct_heading_array[it] = 0; }

      state = transition(3); //wall_following

    }
  } else if (state == 2) { //ROTATE_TO_GOAL
    // check if heading is close to the preferred_angle
    bool goal_check = logicIsCloseTo(wraptopi(current_heading - wanted_angle), 0, 0.1f);
    if (front_range < ref_distance_from_wall + 0.2f) {
      cannot_go_to_goal =  true;
      wall_follower_init(0.4, 0.5);

      state = transition(3); //wall_following

    }
    if (goal_check) {
      state = transition(1); //forward
    }
  } else if (state == 3) {      //WALL_FOLLOWING

    // if another drone is close and there is no right of way, move out of the way
    if (priority == false && rssi_inter < rssi_threshold) {
      if (outbound) {
        if ((rssi_angle_inter < 0 && wanted_angle < 0) || (rssi_angle_inter > 0 && wanted_angle > 0)) {
          wanted_angle = -1 * wanted_angle;
          wanted_angle_dir = wraptopi(current_heading - wanted_angle);
          //state= transition(2);
        }
      }
      if (rssi_inter < rssi_collision_threshold) {
        state = transition(4);

      }
    }

    // If going forward with wall following and cannot_go_to_goal bool is still on
    //    turn it off!
    if (state_wf == 5 && cannot_go_to_goal) {
      cannot_go_to_goal  = false;
    }



    // Check if the goal is reachable from the current point of view of the agent
    float bearing_to_goal = wraptopi(wanted_angle - current_heading);
    bool goal_check_WF = false;
    if (direction == -1) {
      goal_check_WF = (bearing_to_goal < 0 && bearing_to_goal > -1.57f);
    } else {
      goal_check_WF = (bearing_to_goal > 0 && bearing_to_goal < 1.57f);
    }

    // Check if bug went into a looping while wall following,
    //    if so, then forse the reverse direction predical.
    float rel_x_loop = current_pos_x - pos_x_hit;   //  diff_rssi = (int)prev_rssi - (int)rssi_beacon;
    float rel_y_loop = current_pos_y - pos_y_hit;
    float loop_angle = wraptopi(atan2(rel_y_loop, rel_x_loop));

    //if(outbound)
    //{


    if (fabs(wraptopi(wanted_angle_hit + 3.14f - loop_angle)) < 1.0) {
      overwrite_and_reverse_direction = true;
    } else {
     // TODO: check following line causes deadlocks!!!!
      //overwrite_and_reverse_direction = false; // this didn't really work...
    }
    //}else{
    //  overwrite_and_reverse_direction = false;
    //}

    // if during wallfollowing, agent goes around wall, and heading is close to rssi _angle
    //      got to rotate to goal
    // TODO check if the cannot go to goal still does anything useful...
    if ((state_wf == 6 || state_wf == 8) && goal_check_WF && front_range > ref_distance_from_wall + 0.4f
        && !cannot_go_to_goal) {
      wanted_angle_dir = wraptopi(current_heading - wanted_angle); // to determine the direction when turning to goal
      state = transition(2); //rotate_to_goal
    }

    // If going straight
    //    determine by the gradient of the crazyradio what the approx direction is.
    if (state_wf == 5) {


      if (!outbound) {
        // Reset sample gathering
        if (rssi_sample_reset) {
          pos_x_sample = current_pos_x;
          pos_y_sample = current_pos_y;
          rssi_sample_reset = false;
          prev_rssi = rssi_beacon;
        }


        // if the crazyflie traveled for 1 meter, than measure if it went into the right path
        float rel_x_sample = current_pos_x - pos_x_sample;
        float rel_y_sample = current_pos_y - pos_y_sample;
        float distance = sqrt(rel_x_sample * rel_x_sample + rel_y_sample * rel_y_sample);
        if (distance > 1.0f) {
          rssi_sample_reset = true;
          heading_rssi = current_heading;
          int diff_rssi_unf = (int)prev_rssi - (int)rssi_beacon;

          //rssi already gets filtered at the radio_link.c
          diff_rssi = diff_rssi_unf;

          // Estimate the angle to the beacon
          wanted_angle = fillHeadingArray(correct_heading_array, heading_rssi, diff_rssi, 4);


          // for simulation
          //printf("wanted_angle %f, heading_rssi %f, diff_rssi, %d \n",wanted_angle,heading_rssi,diff_rssi);
          //for(int it=0;it<8;it++)printf("%d, ",correct_heading_array[it]);printf("\n");
        }
      }

    } else {
      rssi_sample_reset = true;
    }
  } else if (state == 4) {    //MOVE_OUT_OF_WAY
    // once the drone has gone by, rotate to goal
    if (rssi_inter >= rssi_collision_threshold) {

      state = transition(2); //rotate_to_goal
    }

  }



  /***********************************************************
   * Handle state actions
   ***********************************************************/

  float temp_vel_x = 0;
  float temp_vel_y = 0;
  float temp_vel_w = 0;

  if (state == 1) {        //FORWARD
    // stop moving if there is another drone in the way
    // forward max speed
    if (left_range < ref_distance_from_wall) {
      temp_vel_y = -0.2f;
    }
    if (right_range < ref_distance_from_wall) {
      temp_vel_y = 0.2f;
    }
    temp_vel_x = 0.5;
    //}

  } else  if (state == 2) {  //ROTATE_TO_GOAL
    // rotate to goal, determined on the sign
    if (wanted_angle_dir < 0) {
      commandTurn(&temp_vel_w, 0.5);
    } else {
      commandTurn(&temp_vel_w, -0.5);
    }


  } else  if (state == 3) {       //WALL_FOLLOWING
    //Get the values from the wallfollowing
    if (direction == -1) {
      state_wf = wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, left_range, current_heading, direction);
    } else {
      state_wf = wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, right_range, current_heading, direction);
    }
  } else if (state == 4) {      //MOVE_AWAY

    float save_distance = 0.7f;
    if (left_range < save_distance) {
      temp_vel_y = temp_vel_y - 0.5f;
    }
    if (right_range < save_distance) {
      temp_vel_y = temp_vel_y + 0.5f;
    }
    if (front_range < save_distance) {
      temp_vel_x = temp_vel_x - 0.5f;
    }
    if (back_range < save_distance) {
      temp_vel_x = temp_vel_x + 0.5f;
    }

  }

#ifndef GB_ONBOARD

  //printf("state %d\n",state);
  // printf("direction %f\n",direction);

#endif
  *rssi_angle = wanted_angle;
  *state_wallfollowing = state_wf;

  *vel_x = temp_vel_x;
  *vel_y = temp_vel_y;
  *vel_w = temp_vel_w;

  return state;
}
