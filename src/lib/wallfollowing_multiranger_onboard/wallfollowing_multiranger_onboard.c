/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#include "wallfollowing_multiranger_onboard.h"
#include <math.h>
#include <time.h>

// variables
float ref_distance_from_wall = 0;
float max_speed = 0.5;
float max_rate = 0.5;
float direction = 1;
long int state_start_time = 0;

#define PI  3.14159265359

void testRange(float front_range, float right_range, float left_range)
{
	printf("range %f %f %f\n",front_range, right_range, left_range);
}

void wall_follower_init(float new_ref_distance_from_wall, float max_speed_ref)
{
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
}

// Static helper functions
static bool logicIsCloseTo(float real_value, float checked_value, float margin)
{
	if(real_value>checked_value-margin && real_value<checked_value+margin)
	{
		return true;
	}else
		return false;
}

static float wraptopi(float number)
{
	return (float)fmod(number + PI,(2*PI)-PI);
}


// Static command functions
static void commandTurn(float* vel_x, float* vel_w, float ref_rate)
{
	*vel_x = 0.0;
	*vel_w = direction*ref_rate;

}

static void commandHover(float* vel_x, float* vel_y, float* vel_w)
{
	*vel_x = 0.0;
	*vel_y = 0.0;
	*vel_w = 0.0;
}

static void commandForwardAlongWall(float* vel_x, float* vel_y, float range)
{
	*vel_x = max_speed;
	bool check_distance_wall = logicIsCloseTo(ref_distance_from_wall,range,0.1);
	if(!check_distance_wall)
	{
		if(range>ref_distance_from_wall)
			*vel_y = direction*(-1*max_speed/3);
		else
			*vel_y = direction*(max_speed/3);
	}
}



static int transition(int new_state)
{
	time_t now = time(0);
	struct tm *tm = localtime (&now);
	state_start_time = tm->tm_sec;
	return new_state;
}

void wall_follower(float* vel_x, float* vel_y, float* vel_w, float front_range, float side_range, float current_heading, int direction_turn)
{
   static int state = 1;
   static float previous_heading = 0;
   static float angle = 0;

	time_t now = time(0);
	struct tm *tm = localtime (&now);
	int current_time = tm->tm_sec;

	//  -->STATES<--
    // 1 = forward
    // 2 = hover
    // 3 = turn_to_find_wall
    // 4 = turn_to_allign_to_wall
	// 5 = forward along wall

   // Handle state transitions
   if (state == 1) 			//FORWARD
   {
	   if(front_range<ref_distance_from_wall)
	   {
		   state = transition(3);
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
           state = transition(4);
	   }
   }else if(state==4)		//TURN_TO_ALLIGN_TO_WALL
   {
	   bool allign_wall_check = logicIsCloseTo(wraptopi(current_heading-previous_heading),angle,0.1);
	   if(allign_wall_check)
	   {
		   state = transition(5);
	   }
   }else if(state==5)   	//FORWARD_ALONG_WALL
   {
	   if (front_range < ref_distance_from_wall)
		   state = transition(2);


   }

   float temp_vel_x, temp_vel_y, temp_vel_w;

   // Handle state actions
   if (state == 1) 			//FORWARD
   {
	   temp_vel_x= max_speed;
	   temp_vel_y = 0.0;
	   temp_vel_w = 0.0;

   }else if(state == 2) 	// HOVER
   {
	   commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);


   }else if(state==3)		// TURN_TO_FIND_WALL
   {
	   commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
	   temp_vel_y = 0.0;

   }else if(state==4)		//TURN_TO_ALLIGN_TO_WALL
   {
	   // hover first second to stabilize
	   if (current_time-state_start_time<1)
		   commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
	   else // then turn again
	   {
		   commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
		   temp_vel_y = 0;
	   }


   }else if(state==5)   	//FORWARD_ALONG_WALL
   {

       commandForwardAlongWall(&temp_vel_x, &temp_vel_y, side_range);
       temp_vel_w = 0.0;
   }



   *vel_x = temp_vel_x;
   *vel_y = temp_vel_y;
   *vel_w = temp_vel_w;




}
