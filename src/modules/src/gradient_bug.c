/*
 * gradient_bug.c
 *
 *  Created on: Aug 9, 2018
 *      Author: knmcguire
 */


#include <string.h>
#include <errno.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "gradient_bug.h"
#include "commander.h"
#include "sensors.h"
#include "stabilizer_types.h"

#include "estimator_kalman.h"
#include "stabilizer.h"

#include "wallfollowing_multiranger_onboard.h"
#include "lobe_navigation.h"

#include "oa.h"
#include "multiranger.h"

#include "radiolink.h"

//#define GRADIENT_BUG_NAME "GRADIENTBUG"
//#define GRADIENT_TASK_PRI 2
#define GRADIENT_BUG_COMMANDER_PRI 3
static bool isInit = false;

static bool keep_flying = false;


float height;

static bool taken_off = false;
static float nominal_height = 0.5;
static void take_off(setpoint_t *sp, float velocity)
{
	sp->mode.x = modeVelocity;
	sp->mode.y = modeVelocity;
	sp->mode.z = modeVelocity;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->velocity.z = velocity;
	sp->mode.yaw = modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}

static void land(setpoint_t *sp, float velocity)
{
	sp->mode.x = modeVelocity;
	sp->mode.y = modeVelocity;
	sp->mode.z = modeVelocity;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->velocity.z = - velocity;
	sp->mode.yaw = modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}


static void hover(setpoint_t *sp, float height)
{
	sp->mode.x = modeVelocity;
	sp->mode.y = modeVelocity;
	sp->mode.z = modeAbs;
	sp->velocity.x = 0.0;
	sp->velocity.y = 0.0;
	sp->position.z = height;
	sp->mode.yaw = modeVelocity;
	sp->attitudeRate.yaw = 0.0;
}

static void vel_command(setpoint_t *sp, float vel_x, float vel_y, float yaw_rate, float height)
{
	sp->mode.x = modeVelocity;
	sp->mode.y = modeVelocity;
	sp->mode.z = modeAbs;
	sp->velocity.x = vel_x;
	sp->velocity.y = vel_y;
	sp->position.z = height;
	sp->mode.yaw = modeVelocity;
	sp->attitudeRate.yaw = yaw_rate;
}

static void shut_off_engines(setpoint_t *sp)
{
	sp->mode.x = modeDisable;
	sp->mode.y = modeDisable;
	sp->mode.z = modeDisable;
	sp->mode.yaw = modeDisable;

}

setpoint_t setpoint_BG;
float vel_x_cmd, vel_y_cmd, vel_w_cmd;
float current_heading;
float right_range;
float front_range;
float left_range;
float up_range;
float rssi_angle;


bool manual_startup = false;
bool on_the_ground = true;
uint32_t time_stamp_manual_startup_command = 0;
#define MANUAL_STARTUP_TIMEOUT  M2T(3000)

void gradientBugTask(void *param)
{
	systemWaitStart();
	vTaskDelay(M2T(3000));
	while(1) {
		vTaskDelay(30);
		//getStatePosition(&position);
		height = estimatorKalmanGetElevation();
		current_heading = getHeading() * (float)M_PI / 180.0f;
		front_range = (float)rangeFront/1000.0f;
		right_range = (float)rangeRight/1000.0f;
		left_range = (float)rangeLeft/1000.0f;
		up_range = (float)rangeUp/1000.0f;

		point_t pos;
		estimatorKalmanGetEstimatedPos(&pos);


		memset(&setpoint_BG, 0, sizeof(setpoint_BG));

		//***************** Manual Startup procedure*************//

		// indicate if top range is hit while it is not flying yet, then start counting
		if (keep_flying == false && manual_startup==false && up_range <0.2f && on_the_ground == true)
		{
			manual_startup = true;
			time_stamp_manual_startup_command = xTaskGetTickCount();
		}

		// While still on the ground, but indicated that manual startup is requested, keep checking the time
		if (keep_flying == false && manual_startup == true)
		{
			  uint32_t currentTime = xTaskGetTickCount();
			  // If 3 seconds has passed, start flying.
			  if ((currentTime -time_stamp_manual_startup_command) > MANUAL_STARTUP_TIMEOUT)
			  {
				  keep_flying = true;
				  manual_startup = false;
			  }
		}

		// Don't fly if multiranger is not connected or the uprange is activated
		if (keep_flying == true && (multiranger_isinit == false || up_range <0.2f))
			keep_flying = 0;

		if(keep_flying)
		{
			if(taken_off)
			{
				/*
				 * If the flight is given a OK
				 * 	and the crazyflie has taken off
				 * 	 then perform state machine
				 */
				hover(&setpoint_BG, nominal_height);

				// wall following state machine
				//wall_follower(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range, left_range, current_heading, -1);
				//uint8_t dummy_rssi = 44+(3.14-fabs(current_heading))*20;
				rssi_angle =lobe_navigator(&vel_x_cmd, &vel_y_cmd, &vel_w_cmd, front_range,left_range, current_heading, (float)pos.x, (float)pos.y, rssi_ext);


				// convert yaw rate commands to degrees
				float vel_w_cmd_convert = -1* vel_w_cmd * 180.0f / (float)M_PI;

				// Convert relative commands to world commands
				float psi = current_heading;
				float vel_x_cmd_convert =  cosf(-psi) * vel_x_cmd + sinf(-psi) * vel_y_cmd;
				float vel_y_cmd_convert = -sinf(-psi) * vel_x_cmd + cosf(-psi) * vel_y_cmd;
				//float vel_y_cmd_convert = -1 * vel_y_cmd;
				vel_command(&setpoint_BG, vel_x_cmd_convert, vel_y_cmd_convert,vel_w_cmd_convert, nominal_height);
				on_the_ground = false;
			}else
			{
				/*
				 * If the flight is given a OK
				 * 	but the crazyflie  has not taken off
				 * 	 then take off
				 */
				take_off(&setpoint_BG, 0.2f);
				if(height>nominal_height)
				{
					taken_off = true;
					//wall_follower_init(0.4,0.5);
					init_lobe_navigator();

				}
				on_the_ground = false;

			}
		}else{
			if(taken_off)
			{
				/*
				 * If the flight is given a not OK
				 * 	but the crazyflie  has already taken off
				 * 	 then land
				 */
				land(&setpoint_BG, 0.2f);
				if(height<0.1f)
				{
					shut_off_engines(&setpoint_BG);
					taken_off = false;
				}
				on_the_ground = false;

			}else
			{
				/*
				 * If the flight is given a not OK
				 * 	and crazyflie has landed
				 * 	 then keep engines off
				 */
				shut_off_engines(&setpoint_BG);
				on_the_ground = true;

			}

		}

		commanderSetSetpoint(&setpoint_BG, GRADIENT_BUG_COMMANDER_PRI);


	}
}

void gradientBugInit()
{
	if (isInit)
	{
		return;
	}

	xTaskCreate(gradientBugTask, GRADIENT_BUG_NAME, CMD_HIGH_LEVEL_TASK_STACKSIZE, NULL,
			GRADIENT_TASK_PRI, NULL);
	isInit = true;

}

PARAM_GROUP_START(gbug)
PARAM_ADD(PARAM_UINT8, keep_flying, &keep_flying)
PARAM_GROUP_STOP(gbug)


LOG_GROUP_START(gradientbug)
LOG_ADD(LOG_UINT8, rssi, &rssi_ext)
LOG_ADD(LOG_FLOAT, rssi_angle, &rssi_angle)

/*LOG_ADD(LOG_FLOAT, height, &height)
LOG_ADD(LOG_FLOAT, heading, &current_heading)
LOG_ADD(LOG_FLOAT, right_range, &right_range)
LOG_ADD(LOG_FLOAT, front_range, &front_range)*/

LOG_GROUP_STOP(gradientbug)


