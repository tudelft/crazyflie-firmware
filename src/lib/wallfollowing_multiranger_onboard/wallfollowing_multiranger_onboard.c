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

static void commandTurnAroundCorner(float* vel_x, float* vel_w, float radius)
{
	*vel_x = max_speed;
	*vel_w = direction*(-1*(*vel_x)/radius);
}

static void commandTurnAndAdjust(float* vel_y, float* vel_w,float rate,float range)
{
	*vel_w = direction*rate;
	bool check_distance_to_wall = logicIsCloseTo(ref_distance_from_wall, range, 0.1);
	if(!check_distance_to_wall)
	{
		if(range>ref_distance_from_wall)
			*vel_y = direction * (-1 * max_speed/3);
		else
			*vel_y = direction * (max_speed/3);

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

	direction = direction_turn;
   static int state = 1;
   static float previous_heading = 0;
   static float angle = 0;
   static bool around_corner_first_turn = false;
   static bool around_corner_go_back = false;
	time_t now = time(0);
	struct tm *tm = localtime (&now);
	int current_time = tm->tm_sec;

	/***********************************************************
	 * State definitions
	 ***********************************************************/
		// 1 = forward
		// 2 = hover
		// 3 = turn_to_find_wall
		// 4 = turn_to_allign_to_wall
		// 5 = forward along wall
		// 6 = rotate_around_wall
		// 7 = rotate_in_corner

   /***********************************************************
	* Handle state transitions
	***********************************************************/

	if (state == 1) 			//FORWARD
   {
	   if(front_range<ref_distance_from_wall)
	   {
		   state = transition(3);
	   }
   }else if(state == 2) 		// HOVER
   {

   }else if(state==3)			// TURN_TO_FIND_WALL
   {
	   // check if wall is found
	   bool side_range_check = side_range < ref_distance_from_wall/cos(0.78)+0.2;
	   bool front_range_check = front_range < ref_distance_from_wall/cos(0.78)+0.2;
	   if(side_range_check && front_range_check)
	   {
		   previous_heading = current_heading;
           angle = direction*( 1.57 - atan(front_range/side_range));
           state = transition(4); // go to turn_to_allign_to_wall
	   }
	   if (side_range<1.0 && front_range>2.0)
	   {
		   around_corner_first_turn = true;
		   around_corner_go_back = false;
		   previous_heading = current_heading;
		   state = transition(6); // go to rotate_around_wall
	   }
   }else if(state==4)			//TURN_TO_ALLIGN_TO_WALL
   {
	   bool allign_wall_check = logicIsCloseTo(wraptopi(current_heading-previous_heading),angle,0.1);
	   if(allign_wall_check)
	   {
		   state = transition(5);
	   }
   }else if(state==5)   		//FORWARD_ALONG_WALL
   {

	   // If side range is out of reach,
	   //    end of the wall is reached
       if(side_range > 2)
       {
           around_corner_first_turn = true;
           state = transition(6);
       }
       // If front range is small
       //    then corner is reached
	   if (front_range < ref_distance_from_wall)
	   {
		   previous_heading = current_heading;
		   state = transition(7);
	   }

   }else if(state==6)   		//ROTATE_AROUND_WALL
   {
	   if(front_range < ref_distance_from_wall + 0.3)
	   {
		   state=transition(3);
	   }


   }else if(state==7)    	   //ROTATE_IN_CORNER
   {
	   // Check if heading goes over 0.8 rad
	   bool check_heading_corner = logicIsCloseTo(fabs(wraptopi(current_heading-previous_heading)),0.8,0.1);
	   if(check_heading_corner)
		   state = transition(3);

   }else
   {
	   printf("STATE doesn't exist! \n");
   }



   /***********************************************************
    * Handle state actions
    ***********************************************************/

   float temp_vel_x, temp_vel_y, temp_vel_w;

   if (state == 1) 				//FORWARD
   {
	   temp_vel_x= max_speed;
	   temp_vel_y = 0.0;
	   temp_vel_w = 0.0;

   }else if(state == 2) 		// HOVER
   {
	   commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);


   }else if(state==3)			// TURN_TO_FIND_WALL
   {
	   commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
	   temp_vel_y = 0.0;

   }else if(state==4)			//TURN_TO_ALLIGN_TO_WALL
   {
	   // hover first second to stabilize
	   if (current_time-state_start_time<1)
		   commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
	   else // then turn again
	   {
		   commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
		   temp_vel_y = 0;
	   }

   }else if(state==5)   		//FORWARD_ALONG_WALL
   {

       commandForwardAlongWall(&temp_vel_x, &temp_vel_y, side_range);
       temp_vel_w = 0.0;

   }else if(state==6)   		//ROTATE_AROUND_WALL
   {
	   // If first time around corner
	   	   //first try to find the corner again
	   if(around_corner_first_turn)
	   {
		   commandTurn(&temp_vel_x, &temp_vel_w, -1*max_rate);
		   temp_vel_y = 0;
		   // If corner is found
		   // 	continue going around corner
		   if(side_range<=ref_distance_from_wall+0.5)
		   {
			   around_corner_first_turn = false;
			   previous_heading = current_heading;
		   }

	   }else
	   {
		   // if side range is larger than prefered distance from wall
		   if(side_range>ref_distance_from_wall+0.5)
		   {
			   // check if scanning has already occured
			   if(wraptopi(fabs(current_heading-previous_heading))>0.3)
			   {
				   around_corner_go_back = true;
			   }
			   // turn and adjust distnace to corner from that point
			   if(around_corner_go_back)
			   {
				   // go back if it already went into one direction
				   commandTurnAndAdjust(&temp_vel_y, &temp_vel_w, max_rate,side_range);
				   temp_vel_x = 0.0;
			   }else
			   {
				   commandTurnAndAdjust(&temp_vel_y, &temp_vel_w,-1* max_rate,side_range);
				   temp_vel_x = 0.0;			   }
		   }else
		   {
			   // continue to turn around corner
			   previous_heading = current_heading;
			   around_corner_go_back = false;
			   commandTurnAroundCorner(&temp_vel_x, &temp_vel_w, ref_distance_from_wall);
		   }


	   }





   }else if(state==7)      		 //ROTATE_IN_CORNER
   {
	   commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
	   temp_vel_y = 0;

   }else
   {
	   //State does not exist so hover!!
	   commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
   }




   *vel_x = temp_vel_x;
   *vel_y = temp_vel_y;
   *vel_w = temp_vel_w;


   printf("state = %d\n",state);



}
