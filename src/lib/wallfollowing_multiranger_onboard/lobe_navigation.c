/*
 * lobe_navigation.c
 *
 *  Created on: Nov 7, 2018
 *      Author: knmcguire
 */
#include "lobe_navigation.h"
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

bool first_run = true;

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

// Math functions

static void swap(uint8_t *p,uint8_t *q) {
	int t;

	t=*p;
	*p=*q;
	*q=t;
}

static void sort(uint8_t a[],int32_t n) {
	int i,j;

	for(i = 0;i < n-1;i++) {
		for(j = 0;j < n-i-1;j++) {
			if(a[j] > a[j+1])
				swap(&a[j],&a[j+1]);
		}
	}
}

static uint8_t median(uint8_t array[], int32_t size)
{
	sort(array,size);
	uint32_t index = (size+1) / 2 - 1;
	return array[index];
}

static int32_t mod_floor(int32_t a, int32_t n) {
	return ((a % n) + n) % n;
}

/*static int find_maximum(uint8_t a[], int32_t n) {
	int32_t c, max, index;

	max = a[0];
	index = 0;

	for (c = 1; c < n; c++) {
		if (a[c] > max) {
			index = c;
			max = a[c];
		}
	}

	return index;
}*/

static int find_minimum(uint8_t a[], int32_t n) {
	int32_t c, min, index;

	min = a[0];
	index = 0;

	for (c = 1; c < n; c++) {
		if (a[c] < min) {
			index = c;
			min = a[c];
		}
	}

	return index;
}

static void medianArray(uint8_t array[], uint8_t array_filtered[], int32_t size, int32_t filter_size)
{
	uint8_t temp_array[360];
	int32_t it;

	for(it = 0; it<size;it++)
	{
		int32_t it_temp;

		for(it_temp = it-filter_size/2; it_temp<it+filter_size/2;it_temp++)
		{
			int32_t index_temp = it_temp - it + filter_size/2;
			temp_array[index_temp] = array[ mod_floor(it_temp,size)];
		}

		array_filtered[it]=median(temp_array,filter_size);
	}
}



// Command functions
static void commandTurnCircle(float* vel_x,  float* vel_w, float max_speed, float radius)
{
	*vel_x = max_speed;
	*vel_w = ((*vel_x)/radius);
}

static void commandTurn( float* vel_w, float max_rate)
{
	*vel_w = max_rate;
}

// statemachine
void init_lobe_navigator()
{


	first_run = true;

}

void lobe_navigator(float* vel_x, float* vel_y, float* vel_w, float front_range, float current_heading, float current_pos_x, float current_pos_y, uint8_t rssi)
{
	static int state = 2;
	static float previous_heading = 0;
	static uint8_t rssi_array[360];
	static float heading_array[360];
	static int32_t it_array = 0;
	static float rssi_angle = 0;
	static float prev_pos_x = 0;
	static float prev_pos_y = 0;
	float rssi_angle_dir=0;


#ifndef GB_ONBOARD
	gettimeofday(&now_time,NULL);
#else
	float now = usecTimestamp() / 1e6;
#endif

	if (first_run)
	{
		previous_heading = current_heading;
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
	// 2 = rotate_360
	// 3 = rotate_to_goal

	/***********************************************************
	 * Handle state transitions
	 ***********************************************************/

	if (state == 1) 			//FORWARD
	{
		float pos_x_diff = current_pos_x-prev_pos_x;
		float pos_y_diff = current_pos_y-prev_pos_y;
		float distance = sqrt(pos_x_diff*pos_x_diff+pos_y_diff*pos_y_diff);
		if (distance>0.5f)
		{
			previous_heading = current_heading;
			int32_t it;for(it=0;it<360;it++){rssi_array[it]=(uint8_t)0;heading_array[it]=(float)0;}
			it_array=0;
			state = transition(2);

		}


	}else if(state == 2)         //ROTATE_360
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

					uint8_t rssi_array_filtered[360];

					medianArray(rssi_array, rssi_array_filtered, it_array, 10);
					//uint32_t it_check;for(it_check=0;it_check<it_array;it_check++)printf("%d,",rssi_array_filtered[it_check]);printf("\n");

					int32_t min_index = find_minimum(rssi_array_filtered,it_array);
					rssi_angle = wraptopi(heading_array[min_index]+3.14f);

					state = transition(3);

					previous_heading = current_heading;
					rssi_angle_dir = wraptopi(current_heading-rssi_angle);
				}

			}

	}else if(state == 3)         //ROTATE_TO_GOAL
	{
		bool goal_check = logicIsCloseTo(wraptopi(current_heading-previous_heading),rssi_angle,0.1f);
		if(goal_check)
		{
			prev_pos_x = current_pos_x;
			prev_pos_y = current_pos_y;

			state = transition(1);

		}


	}


	/***********************************************************
	 * Handle state actions
	 ***********************************************************/

	float temp_vel_x=0;
	float temp_vel_y=0;
	float temp_vel_w=0;

	if (state == 1) 				//FORWARD
	{
		temp_vel_x= 0.5;



	}else if(state == 2) 		// ROTATE_360
	{

		commandTurnCircle(&temp_vel_x, &temp_vel_w, 0.5,0.5);
		rssi_array[it_array]=rssi;
		heading_array[it_array]=current_heading;
		it_array++;



	}else if(state==3)			//ROTATE_TO_GOAL
	{

		if (rssi_angle_dir<0)
			commandTurn(&temp_vel_w, 0.5);
		else
			commandTurn(&temp_vel_w, -0.5);


	}



#ifndef GB_ONBOARD

	printf("state %d\n",state);

#endif

	*vel_x = temp_vel_x;
	*vel_y = temp_vel_y;
	*vel_w = temp_vel_w;

}
