/*
 * wallfollowing_with_avoid.c
 *
 *  Created on: Nov 12, 2018
 *      Author: knmcguire
 */

#include "wallfollowing_multiranger_onboard.h"
#include "wallfollowing_with_avoid.h"

#include <math.h>

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
static float ref_distance_from_wall = 0.5;
static float max_speed = 0.5;
static float local_direction = 1;

#ifndef GB_ONBOARD

// Helper functions
static int diff_ms(struct timeval t1, struct timeval t2)
{
	return (((t1.tv_sec - t2.tv_sec) * 1000000) +
			(t1.tv_usec - t2.tv_usec))/1000;
}
#endif

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



// Static helper functions
/*static bool logicIsCloseTo(float real_value, float checked_value, float margin)
{
	if(real_value>checked_value-margin && real_value<checked_value+margin)
	{
		return true;
	}else
		return false;
}*/

/*
static float wraptopi(float number)
{
	if(number>(float)M_PI)
		return (number-(float)(2*M_PI));
	else if(number< (float)(-1*M_PI))
		return (number+(float)(2*M_PI));
	else
		return (number);

}
*/


// Command functions
static void commandTurn( float* vel_w, float max_rate)
{
	*vel_w = max_rate;
}


// statemachine functions
void init_wall_follower_and_avoid_controller(float new_ref_distance_from_wall, float max_speed_ref, float starting_local_direction)
{
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
    local_direction = starting_local_direction;
	first_run = true;
}


void wall_follower_and_avoid_controller(float* vel_x, float* vel_y, float* vel_w, float front_range, float left_range, float right_range,  float current_heading, float pos_x, float pos_y, uint8_t rssi_other_drone)
{

	// Initalize static variables
	static int state = 1;
	//static float previous_heading = 0;
	static int state_wf = 0;
	static bool already_turned = false;
	static float prev_pos_x = 0;

	static float prev_pos_y = 0;


#ifndef GB_ONBOARD
	gettimeofday(&now_time,NULL);
#else
	//float now = (float)usecTimestamp() / (float)1e6;
#endif

	// if it is reinitialized
	if (first_run)
	{
		//previous_heading = current_heading;
		state = 1;
#ifndef GB_ONBOARD
		gettimeofday(&state_start_time,NULL);
#else
		float t =  usecTimestamp() / 1e6;
		state_start_time = t;
#endif
		first_run = false;
	}


	/***********************************************************
	 * State definitions
	 ***********************************************************/
	// 1 = forward
	// 2 = wall_following
	// 3 = rotate_local_direction

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state == 1) 			//FORWARD
	{
		// if front range is close, start wallfollowing
		if(front_range<ref_distance_from_wall+0.2f)
		{
			wall_follower_init(ref_distance_from_wall,0.5);
			state = transition(2); //wall_following
		}
	}else if(state == 2)         //WALL_FOLLOWING
	{

		// After 10 seconds, turn already_turned back on again
		float diff_x= prev_pos_x - pos_x;
		float diff_y= prev_pos_y - pos_y;

		float distance_from_turning_point = sqrt(diff_x*diff_x + diff_y*diff_y);
	    if (distance_from_turning_point>2&&already_turned)

	    {
	    	already_turned=false;
	    }
		//printf("time %d  already turned %d ",diff_ms(now_time, state_start_time),already_turned);

		//printf("rssi other drone %d state %d\n",rssi_other_drone,state_wf);

		// if during wall-following, agent gets too close to another agent, change local direction
		if(rssi_other_drone<50&& state_wf == 5 && already_turned == false)
		{
			state = transition(3);
		}
	}else if(state == 3)         //ROTATE_LOCAL_DIRECTION
	{
		// Ones turned local direction, continues with wall follwoing
		if(front_range<ref_distance_from_wall+0.2f)
		{
			local_direction = local_direction*-1;
			wall_follower_init(ref_distance_from_wall,0.5);
			already_turned = true;
			prev_pos_x = pos_x;
			prev_pos_y =pos_y;
			state = transition(2);
		}
	}


	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x=0;
	float temp_vel_y=0;
	float temp_vel_w=0;

	if (state == 1) 		     //FORWARD
	{
		// forward max speed
		temp_vel_x= 0.5;

	}else  if(state == 2)          //WALL_FOLLOWING
	{
		//Get the values from the wallfollowing
		if(local_direction == 1)
		{
		   state_wf= wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, right_range, current_heading, local_direction);
		}else if(local_direction == -1)
		{
			state_wf=	wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, left_range, current_heading, local_direction);
		}

	}else if(state == 3)          //ROTATE_LOCAL_DIRECTION
	{
		//Turn to see the other side of the wall
		commandTurn( &temp_vel_w, local_direction*-0.5f);
	}

#ifndef GB_ONBOARD

	printf("state wf %d\n",state);

#endif

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;


}

