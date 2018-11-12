/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#include "wallfollowing_multiranger_onboard.h"
#include <math.h>
//#include <time.h>
//#include <sys/time.h>
//#include "usec_time.h"

#ifndef GB_ONBOARD
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#else
#include "usec_time.h"
#endif

// variables
static float ref_distance_from_wall = 0;
static float max_speed = 0.5;
static float max_rate = 0.5;
static float direction = 1;
static float first_run = false;

#ifndef GB_ONBOARD
struct timeval state_start_time;
struct timeval now_time;
#else
float state_start_time;
#endif

#ifndef GB_ONBOARD

static int diff_ms(struct timeval t1, struct timeval t2)
{
    return (((t1.tv_sec - t2.tv_sec) * 1000000) +
            (t1.tv_usec - t2.tv_usec))/1000;
}

float get_sec(struct timeval t1)
{
	return (float)((((t1.tv_sec) * 1000000) +
            (t1.tv_usec))/1000)/1000.0f;
}
#endif


void testRange(float front_range, float right_range, float left_range)
{
	//printf("range %f %f %f\n",front_range, right_range, left_range);
}

void wall_follower_init(float new_ref_distance_from_wall, float max_speed_ref)
{
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
	first_run = true;
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
	// fmod() has difficulty with the sign...
	/*if(number>0)
		return (float)fmod(number + PI,(2*PI)-PI);
	else
		return (float)fmod(number + PI,(2*PI)+PI);*/
	if(number>(float)M_PI)
		return (number-(float)(2*M_PI));
	else if(number< (float)(-1*M_PI))
		return (number+(float)(2*M_PI));
	else
		return (number);

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
	*vel_y = 0;
	if(!check_distance_wall)
	{
		if(range>ref_distance_from_wall)
			*vel_y = direction*(-1*max_speed/3);
		else
			*vel_y = direction*(max_speed/3);
	}
}

/*static void commandForwardAlongWallHeadingSine(float* vel_x, float* vel_y, float* vel_w, float range)
{
	static int32_t counter =0;
	*vel_x = max_speed;
	bool check_distance_wall = logicIsCloseTo(ref_distance_from_wall,range,0.1);
	*vel_y = 0;
#ifndef GB_ONBOARD
	gettimeofday(&now_time,NULL);
	float now = get_sec(now_time);
	//printf("now %f\n",now);
#else
	float now = usecTimestamp() / 1e6;
#endif

	*vel_w = 0.1*sin(5*(float)counter/30.0f);
	printf("check vel %f\n",*vel_w);
	if(!check_distance_wall)
	{
		if(range>ref_distance_from_wall)
			*vel_y = direction*(-1*max_speed/3);
		else
			*vel_y = direction*(max_speed/3);
	}
	counter++;
}*/


/*static void commandForwardAlongWallHeadingAdjust(float* vel_x, float* vel_y, float* vel_w, float range, float diff_range)
{
	*vel_x = max_speed;
	bool check_distance_wall = logicIsCloseTo(ref_distance_from_wall,range,0.1);
	*vel_y = 0;
	if(!check_distance_wall)
	{
		if(diff_range>0)
			*vel_w = direction*(max_rate/3);
		else
			*vel_w = direction*(-1*max_rate/3);
	}
}*/


/*static void commandTurnAroundCorner(float* vel_x, float* vel_w, float radius)
{
	*vel_x = max_speed;
	*vel_w = direction*(-1*(*vel_x)/radius);
}*/

static void commandTurnAroundCornerAndAdjust(float* vel_x, float* vel_y, float* vel_w, float radius,float range)
{
	*vel_x = max_speed;
	*vel_w = direction*(-1*(*vel_x)/radius);
	bool check_distance_to_wall = logicIsCloseTo(ref_distance_from_wall, range, 0.1);
	if(!check_distance_to_wall)
	{
		if(range>ref_distance_from_wall)
		{
			*vel_y = direction * (-1* max_speed/3);

		}

		else
		{
			*vel_y = direction * (max_speed/3);

		}

	}
}

static void commandTurnAndAdjust(float* vel_y, float* vel_w,float rate,float range)
{
	*vel_w = direction*rate;
	*vel_y = 0;
	/*bool check_distance_to_wall = logicIsCloseTo(ref_distance_from_wall, range, 0.1);
	if(!check_distance_to_wall)
	{
		if(range>ref_distance_from_wall)
		{
			*vel_y = direction * (-1* max_speed/3);

		}

		else
		{
			*vel_y = direction * (max_speed/3);

		}

	}*/
}

static int transition(int new_state)
{
#ifndef GB_ONBOARD
	gettimeofday(&state_start_time,NULL);
#else
	float t =  usecTimestamp() / 1e6;
	state_start_time = t;
#endif

	return new_state;

}

int wall_follower(float* vel_x, float* vel_y, float* vel_w, float front_range, float side_range, float current_heading, int direction_turn)
{

	direction = direction_turn;
   static int state = 3;
   static float previous_heading = 0;
   static float angle = 0;
   static bool around_corner_first_turn = false;
   static bool around_corner_go_back = false;
   //static float prev_side_range = 0;

#ifndef GB_ONBOARD
	gettimeofday(&now_time,NULL);
#else
	float now = usecTimestamp() / 1e6;
#endif

	if(first_run)
	{
		previous_heading = current_heading;
		state = 3;
		around_corner_first_turn = false;
		around_corner_go_back = false;
		first_run = false;
	}


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
	   if(front_range<ref_distance_from_wall+0.2f)
	   {
		   state = transition(3);
	   }
   }else if(state == 2) 		// HOVER
   {

   }else if(state==3)			// TURN_TO_FIND_WALL
   {

	   // check if wall is found
	   bool side_range_check = side_range < ref_distance_from_wall/(float)cos(0.78f)+0.2f;
	   bool front_range_check = front_range < ref_distance_from_wall/(float)cos(0.78f)+0.2f;
	   if(side_range_check && front_range_check)
	   {
		   previous_heading = current_heading;
           angle = direction*( 1.57f - (float)atan(front_range/side_range)+0.1f);
           state = transition(4); // go to turn_to_allign_to_wall
	   }
	   if (side_range<1.0f && front_range>2.0f)
	   {
		   around_corner_first_turn = true;
		   around_corner_go_back = false;
		   previous_heading = current_heading;
		   state = transition(6); // go to rotate_around_wall
	   }
   }else if(state==4)			//TURN_TO_ALLIGN_TO_WALL
   {
	   bool allign_wall_check = logicIsCloseTo(wraptopi(current_heading-previous_heading),angle,0.1f);
	   if(allign_wall_check)
	   {
		  // prev_side_range = side_range;
		   state = transition(5);
	   }
   }else if(state==5)   		//FORWARD_ALONG_WALL
   {

	   // If side range is out of reach,
	   //    end of the wall is reached
       if(side_range > ref_distance_from_wall + 0.2f)
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
	   if(front_range < ref_distance_from_wall + 0.1f)
	   {
		   state=transition(3);
	   }


   }else if(state==7)    	   //ROTATE_IN_CORNER
   {
	   // Check if heading goes over 0.8 rad
	   bool check_heading_corner = logicIsCloseTo(fabs(wraptopi(current_heading-previous_heading)),0.8f,0.1f);
	   if(check_heading_corner)
		   state = transition(3);

   }else
   {
	  // printf("STATE doesn't exist! \n");
   }

#ifndef GB_ONBOARD
printf("state %d\n",state);
#endif


   /***********************************************************
    * Handle state actions
    ***********************************************************/

   float temp_vel_x=0;
   float temp_vel_y=0;
   float temp_vel_w=0;

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


	   // hover first second to stabilize (tv_usec i microseconds)
#ifndef GB_ONBOARD
	   if (diff_ms(now_time, state_start_time)<1000)

#else
	   if (now-state_start_time<1.0f)
#endif

		   commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
	   else // then turn again
	   {
		   commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
		   temp_vel_y = 0;
	   }

   }else if(state==5)   		//FORWARD_ALONG_WALL
   {
	  // float diff_range = prev_side_range-side_range;
	   //commandForwardAlongWallHeadingAdjust(&temp_vel_x, &temp_vel_y,&temp_vel_w, side_range, diff_range);
	   //prev_side_range = side_range;
       commandForwardAlongWall(&temp_vel_x, &temp_vel_y, side_range);
       temp_vel_w = 0.0f;

	   //commandForwardAlongWallHeadingSine(&temp_vel_x, &temp_vel_y,&temp_vel_w, side_range);

   }else if(state==6)   		//ROTATE_AROUND_WALL
   {
	   // If first time around corner
	   	   //first try to find the corner again
	   if(around_corner_first_turn)
	   {
		   commandTurn(&temp_vel_x, &temp_vel_w, -1*max_rate);
		   temp_vel_y = 0.0f;
		   // If corner is found
		   // 	continue going around corner
		   if(side_range<=ref_distance_from_wall+0.5f)
		   {
			   around_corner_first_turn = false;
			   previous_heading = current_heading;
		   }

	   }else
	   {
		   // if side range is larger than prefered distance from wall
		   if(side_range>ref_distance_from_wall+0.5f)
		   {

			   // check if scanning has already occured
			   if(wraptopi(fabs(current_heading-previous_heading))>0.3f)
			   {
				   around_corner_go_back = true;
			   }
			   // turn and adjust distnace to corner from that point
			   if(around_corner_go_back)
			   {
				   // go back if it already went into one direction
				   commandTurnAndAdjust(&temp_vel_y, &temp_vel_w, max_rate,side_range);
				   temp_vel_x = 0.0f;
			   }else
			   {
				   commandTurnAndAdjust(&temp_vel_y, &temp_vel_w,-1* max_rate,side_range);
				   temp_vel_x = 0.0f;			   }
		   }else
		   {
			   // continue to turn around corner
			   previous_heading = current_heading;
			   around_corner_go_back = false;
			   commandTurnAroundCornerAndAdjust(&temp_vel_x, &temp_vel_y, &temp_vel_w, ref_distance_from_wall, side_range);
			 //  temp_vel_w = temp_vel_w - 0.05f;
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


   //printf("state = %d\n",state);

return state;

}
