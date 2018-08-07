/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#include "wallfollowing_multiranger_onboard.h"
#include <math.h>

// variables
float ref_distance_from_wall = 0;
float max_speed = 0.5;
float max_rate = 0.5;
float direction = 1;

void testRange(float front_range, float right_range, float left_range)
{
	printf("range %f %f %f\n",front_range, right_range, left_range);
}

void wall_follower_init(float new_ref_distance_from_wall, float max_speed_ref)
{
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
}

// Static command functions
static void commandTurn(float* vel_x, float* vel_w, float ref_rate)
{
	*vel_x = 0.0;
	*vel_w = direction*ref_rate;

}

void wall_follower(float* vel_x, float* vel_y, float* vel_w, float front_range, float side_range, float current_heading, int direction_turn)
{
   static int state = 1;
   static float previous_heading = 0;
   static angle = 0;
	//states
   // 1 = forward
   // 2 = hover
   // 3 = turn_to_find_wall

   // Handle state transitions
   if (state == 1) 			//FORWARD
   {
	   if(front_range<ref_distance_from_wall)
	   {
		   state = 3;
	   }
   }else if(state == 2) 	// HOVER
   {

   }else if(state==3)		// TURN_TO_FIND_WALL
   {

	   // check if wall is found
	   bool side_range_check = side_range < ref_distance_from_wall/cos(0.78)+0.2;
	   bool front_range_check = front_range < ref_distance_from_wall/cos(0.78)+0.2;
	   if(side_range_check && front_range_check)
	   {
		   previous_heading = current_heading;
           angle = direction*( 1.57 - atan(front_range/side_range));
           state = 2;
	   }
   }

   float temp_vel_x, temp_vel_y, temp_vel_w;

   // Handle state actions
   if (state == 1) 			//FORWARD
   {
	   *vel_x = max_speed;
	   *vel_y = 0.0;
	   *vel_w = 0.0;

   }else if(state == 2) 	// HOVER
   {
	   *vel_x = 0.0;
	   *vel_y = 0.0;
	   *vel_w = 0.0;

   }else if(state==3)		// TURN_TO_FIND_WALL
   {
	   commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
	   *vel_x = temp_vel_x;
	   *vel_y = 0.0;
	   *vel_w = temp_vel_w;


   }



}
