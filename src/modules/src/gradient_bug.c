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

#include "estimator_complementary.h"

//#define GRADIENT_BUG_NAME "GRADIENTBUG"
//#define GRADIENT_TASK_PRI 2
#define GRADIENT_BUG_COMMANDER_PRI 3
static bool isInit = false;

static bool keep_flying = false;


point_t position;

/*
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

static void shut_off_engines(setpoint_t *sp)
{
	sp->mode.x = modeDisable;
	sp->mode.y = modeDisable;
	sp->mode.z = modeDisable;
	sp->mode.yaw = modeDisable;

}

setpoint_t setpoint_BG;

*/
void gradientBugTask(void *param)
{
	systemWaitStart();
	while(1) {
		vTaskDelay(10);
		/*getStatePosition(&position);


		sensorsAcquire(sensorData, tick);

		memset(&setpoint_BG, 0, sizeof(setpoint_BG));

		if(keep_flying && !taken_off)
		{
			take_off(&setpoint_BG, 0.2f);
			if(sensorData->zrange.distance>nominal_height)
			{
				hover(&setpoint_BG, nominal_height);
				taken_off = true;
			}
		}else if(!keep_flying && taken_off)
		{

			land(&setpoint_BG, 0.2f);
			if(sensorData->zrange.distance<0.1f)
			{
				shut_off_engines(&setpoint_BG);
				taken_off = false;
			}
		}else{
			shut_off_engines(&setpoint_BG);

		}



		commanderSetSetpoint(&setpoint_BG, GRADIENT_BUG_COMMANDER_PRI);
*/

	}
}

void gradientBugInit()
{
	if (isInit)
	{
		return;
	}

	/*xTaskCreate(gradientBugTask, GRADIENT_BUG_NAME, CMD_HIGH_LEVEL_TASK_STACKSIZE, NULL,
			GRADIENT_TASK_PRI, NULL);*/
	  isInit = true;

}

PARAM_GROUP_START(gradientbug_param)
PARAM_ADD(PARAM_UINT8, keep_flying, &keep_flying)
PARAM_GROUP_STOP(gradientbug_param)


LOG_GROUP_START(gradientbug)
LOG_ADD(LOG_FLOAT, height, &position.z)
LOG_GROUP_STOP(gradientbug)


