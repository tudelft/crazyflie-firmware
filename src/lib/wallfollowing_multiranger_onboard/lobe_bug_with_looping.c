/*
 * com_bug_with_looping.c
 *
 *  Created on: Nov 8, 2018
 *      Author: knmcguire
 */
#include "lobe_bug_with_looping.h"
#include "wallfollowing_multiranger_onboard.h"

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
static float ref_distance_from_wall = 0;
static float max_speed = 0.5;


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
	if(number>(float)M_PI)
		return (number-(float)(2*M_PI));
	else if(number< (float)(-1*M_PI))
		return (number+(float)(2*M_PI));
	else
		return (number);

}


// Command functions
static void commandTurn( float* vel_w, float max_rate)
{
	*vel_w = max_rate;
}
// Command functions
static void commandTurnCircle(float* vel_x,  float* vel_w, float max_speed, float radius)
{
	*vel_x = max_speed;
	*vel_w = ((*vel_x)/radius);
}


// statemachine functions
void init_lobe_bug_loop_controller(float new_ref_distance_from_wall, float max_speed_ref)
{
	ref_distance_from_wall = new_ref_distance_from_wall;
	max_speed = max_speed_ref;
	first_run = true;
}


int lobe_bug_loop_controller(float* vel_x, float* vel_y, float* vel_w, float* rssi_angle_save, float front_range, float left_range, float right_range,
		float current_heading, float current_pos_x, float current_pos_y, uint8_t rssi_beacon)
{

	// Initalize static variables
	static int state = 2;
	static float previous_heading = 0;
	static int state_wf=0;
	static float wanted_angle =0;
	static float wanted_angle_dir = 0;
	static float pos_x_hit = 0;
	static float pos_y_hit = 0;
	static bool overwrite_and_reverse_direction = false;
	static float direction = -1;
	static bool cannot_go_to_goal = false;
	static uint8_t temp_rssi = 150;
	static float temp_heading =0;
	static bool already_did_circle = false;
	static float prev_pos_x = 0;
	static float prev_pos_y = 0;

#ifndef GB_ONBOARD
	gettimeofday(&now_time,NULL);
#else
	float now = usecTimestamp() / 1e6;
#endif


	// if it is reinitialized
	if (first_run)
	{
		//previous_heading = current_heading;
		overwrite_and_reverse_direction = false;
		state = 2;
#ifndef GB_ONBOARD
		gettimeofday(&state_start_time,NULL);
#else
		float t =  usecTimestamp() / 1e6;
		state_start_time = t;
#endif
		first_run = false;
		prev_pos_x = current_pos_x;
		prev_pos_y = current_pos_y;

	}


	/***********************************************************
	 * State definitions
	 ***********************************************************/
	// 1 = forward
	// 2 = rotate_to_goal
	// 3 = wall_following

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state == 1) 			//FORWARD
	{
		// Calculate the traveled distance
		float pos_x_diff = current_pos_x-prev_pos_x;
		float pos_y_diff = current_pos_y-prev_pos_y;
		float distance = sqrt(pos_x_diff*pos_x_diff+pos_y_diff*pos_y_diff);





		//printf("%f\n",front_range);
		if(front_range<ref_distance_from_wall+0.2f)
		{



			if (overwrite_and_reverse_direction)
			{
				direction = -1.0f*direction;
				overwrite_and_reverse_direction = false;
			}



			pos_x_hit = current_pos_x;
			pos_y_hit = current_pos_y;

			wall_follower_init(0.4,0.5);

			state = transition(3); //wall_following

		}

		if (distance>1.0f&& already_did_circle==false)
		{
			state = transition(4);
		}
	}else if(state == 2) 		//ROTATE_TO_GOAL
	{
		// check if heading is close to the preferred_angle
		bool goal_check = logicIsCloseTo(wraptopi(current_heading- wanted_angle),0,0.1f);
		if(front_range<ref_distance_from_wall+0.3f)
		{
			//pos_x_hit = current_pos_x;
			//pos_y_hit = current_pos_y;
			cannot_go_to_goal=  true;
			wall_follower_init(0.4,0.5);

			state = transition(3); //wall_following

		}
		if(goal_check)
		{
			prev_pos_x = current_pos_x;
			prev_pos_y = current_pos_y;

			state = transition(1); //forward
		}
	}else if(state == 3)         //WALL_FOLLOWING
	{
		// if during wallfollowing, agent goes around wall, and heading is close to rssi _angle
		//      got to rotate to goal

		float bearing_to_goal = wraptopi(wanted_angle-current_heading);
		bool goal_check_WF=false;
		if (direction == -1)
			goal_check_WF= (bearing_to_goal<0 && bearing_to_goal>-1.57f);
		else
			goal_check_WF = (bearing_to_goal>0 && bearing_to_goal<1.57f);


		float rel_x_loop = current_pos_x- pos_x_hit;
		float rel_y_loop = current_pos_y -pos_y_hit;

		float loop_angle = wraptopi(atan2(rel_y_loop,rel_x_loop));

		if (fabs(wraptopi(bearing_to_goal+3.14f-loop_angle))<0.5)
		{
			overwrite_and_reverse_direction = true;
		}


		if(state_wf == 5 && cannot_go_to_goal)
		{
			cannot_go_to_goal  = false;
		}

		//logicIsCloseTo(wraptopi(current_heading-wanted_angle),0,0.3f);
		if((state_wf == 6||state_wf == 8) && goal_check_WF && front_range>ref_distance_from_wall+0.4f && !cannot_go_to_goal)
		{
			already_did_circle = false;

			wanted_angle_dir = wraptopi(current_heading-wanted_angle); // to determine the direction when turning to goal
			state = transition(2); //rotate_to_goal
		}
	}else if(state == 4)         //HOVER_BEFORE_ROTATE
	{

#ifndef GB_ONBOARD
		if (diff_ms(now_time, state_start_time)>1000)
#else
			if (now-state_start_time>1.0f)
#endif
			{
				previous_heading = current_heading;
				temp_rssi = 150;
				state = transition(5); //rotate_360
			}
	}else if(state==5)             // ROTATE_360
	{

#ifndef GB_ONBOARD
		if (diff_ms(now_time, state_start_time)>1000)
#else
			if (now-state_start_time>1.0f)
#endif
			{

				bool circle_check = logicIsCloseTo(wraptopi(current_heading-previous_heading),0,0.1f);
				if(circle_check)
				{
					wanted_angle = wraptopi(temp_heading-3.14f);
					//printf("wanted angle %f\n", wanted_angle);
					already_did_circle = true;
					wanted_angle_dir = wraptopi(current_heading-wanted_angle); // to determine the direction when turning to goal

					state = transition(2); //rotate_to_goal

				}



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
		if (left_range<ref_distance_from_wall)
		{
			temp_vel_y = -0.2f;
		}
		if (right_range<ref_distance_from_wall)
		{
			temp_vel_y = 0.2f;
		}
		temp_vel_x= 0.5;

	}else  if(state == 2)			//ROTATE_TO_GOAL
	{
		// rotate to goal, determined on the sign
		if (wanted_angle_dir<0)
			commandTurn(&temp_vel_w, 0.5);
		else
			commandTurn(&temp_vel_w, -0.5);


	}else  if(state == 3)          //WALL_FOLLOWING
	{
		//Get the values from the wallfollowing
		if(direction == -1)
			state_wf = wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, left_range, current_heading, direction);
		else
			state_wf = wall_follower(&temp_vel_x, &temp_vel_y, &temp_vel_w, front_range, right_range, current_heading, direction);
	}else if(state==4)				//HOVER_BEFORE_ROTATE
	{

	}else if(state==5)				//ROTATE_360
	{
		commandTurnCircle(&temp_vel_x, &temp_vel_w, 0.25,0.25);

		//printf("rssi_beacon %d temp_rssi %d current_heading%f\n",rssi_beacon,temp_rssi, current_heading);
		if(rssi_beacon<temp_rssi){
			temp_rssi = rssi_beacon;
			temp_heading = current_heading;
		}
		//printf("temp_rssi %d temp_heading %f\n",temp_rssi,temp_heading);
	}

#ifndef GB_ONBOARD

	printf("state %d\n",state);

#endif

	*rssi_angle_save = wanted_angle;

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;

	return state;
}
