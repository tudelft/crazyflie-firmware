/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#include "wallfollowing_multiranger_onboard.h"

// variables
float ref_distance_from_wall = 0;
float max_speed = 0;

void testRange(float front_range, float right_range, float left_range)
{
	printf("range %f %f %f\n",front_range, right_range, left_range);
}

void wall_follower_init(float new_ref_distance_from_wall, float max_speed_ref)
{
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
}

void wall_follower(float* vel_x, float* vel_y, float front_range, float side_range, float current_heading, int direction_turn)
{
   static int state = 1;
	//states
   // 1 = forward
   // 2 = hover

   // Handle state transitions
   if (state == 1) 			//FORWARD
   {
	   if(front_range<ref_distance_from_wall)
	   {
		   state = 2;
	   }
   }else if(state == 2) 	// HOVER
   {

   }


   // Handle state actions
   if (state == 1) 			//FORWARD
   {
	   *vel_x = max_speed;
	   *vel_y = 0.0;

   }else if(state == 2) 	// HOVER
   {
	   *vel_x = 0.0;
	   *vel_y = 0.0;
   }



}
