#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_localization.h"
#include "num.h"
#include "param.h"
#include "debug.h"
#include <stdlib.h> // random
#include "lpsTwrTag.h" // UWBNum
#include "configblock.h"
#include "uart2.h"
#include "log.h"
#include <math.h>
#define USE_MONOCAM 0

static bool isInit;
static bool onGround = true;
static bool keepFlying = false;
static setpoint_t setpoint;
static float_t relaVarInCtrl[NumUWB][STATE_DIM_rl];
static float_t inputVarInCtrl[NumUWB][STATE_DIM_rl];
static uint8_t selfID;
static float_t height;

static float relaCtrl_p = 2.0f;
static float relaCtrl_i = 0.0001f;
static float relaCtrl_d = 0.01f;
// static float NDI_k = 2.0f;
static char c = 0; // monoCam
float search_range = 3.0; // search range in meters

struct Point
{
    float x;
    float y;
};

struct Point agent_pos,goal, random_point;

static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}

static void setvelHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}


static void flyRandomIn1meter(void){
  float_t rand_x = (rand()/(float)RAND_MAX)*(1.0f)-0.5f;
  float_t rand_y = (rand()/(float)RAND_MAX)*(1.0f)-0.5f;

  for (int i=1; i<100; i++) {
    setHoverSetpoint(&setpoint, rand_x, rand_y, height, 0);
    vTaskDelay(M2T(10));
  }

}

void flyVerticalInterpolated(float startz, float endz, float interpolate_time) {
    setpoint_t setpoint;
    int CMD_TIME = 100;   // new command ever CMD_TIME seconds
    int NSTEPS = (int) interpolate_time / CMD_TIME; 
    for (int i = 0; i < NSTEPS; i++) {
        float curr_height = startz + (endz - startz) * ((float) i / (float) NSTEPS);
        setvelHoverSetpoint(&setpoint, 0, 0, curr_height, 0); 
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(100);
    }
}

bool check_collision(void)
{
  bool collision = false;
  float distance = 100.;
  for(int i = 0; i<NumUWB; i++)
  {
    if ( i != selfID)
    {
       distance = sqrtf(powf(relaVarInCtrl[i][STATE_rlX],2) + powf(relaVarInCtrl[i][STATE_rlY],2));
       if (distance < 0.5f)
       {
         collision = true;
       }       
    }
  }
  return collision;
}


void relativeControlTask(void* arg)
{
  static uint32_t ctrlTick;
  systemWaitStart();
  // height = (float)selfID*0.1f+0.2f;
  height = 0.5f;
  while(1) {
    vTaskDelay(10);

    keepFlying = command_share(selfID, keepFlying);
    DEBUG_PRINT("%d %d \n", keepFlying,relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl) );
    if(relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl) && keepFlying){
      // take off
      if(onGround){
        for (int i=0; i<5; i++) {
          setvelHoverSetpoint(&setpoint, 0, 0, 0.3f, 0);
          vTaskDelay(M2T(100));
        }
        for (int i=0; i<10*selfID; i++) {
          setvelHoverSetpoint(&setpoint, 0, 0, 0.3f, 0);
          vTaskDelay(M2T(100));
        }
        onGround = false;
        ctrlTick = xTaskGetTickCount();
      }

      // control loop
      // setHoverSetpoint(&setpoint, 0, 0, height, 0); // hover
      uint32_t tickInterval = xTaskGetTickCount() - ctrlTick;
      if( tickInterval < 20000){
        flyRandomIn1meter(); // random flight within first 10 seconds
      }
      else
      {
        random_point.x = (rand()/(float)RAND_MAX)*search_range;
        random_point.y = (rand()/(float)RAND_MAX)*search_range;
        for (int i = 0; i< 70; i++)
        {
          relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl);
          if(check_collision())
          {
            random_point.x = (rand()/(float)RAND_MAX)*search_range;
            random_point.y = (rand()/(float)RAND_MAX)*search_range;
            // give some time to get away
            for (int i = 0; i <10 ; i++)
            {
              setHoverSetpoint(&setpoint,random_point.x,random_point.y,height,0);
              vTaskDelay(M2T(100));
            }
          }
          setHoverSetpoint(&setpoint,random_point.x,random_point.y,height,0);
          vTaskDelay(M2T(100));
        }
      }
      
    }
    else{
      // landing procedure
      if(!onGround){
        flyVerticalInterpolated(height,0.0f,6000.0f);
        onGround = true;
      } 
    }
  }
}

void relativeControlInit(void)
{
  if (isInit)
    return;
  selfID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f) - 5);
#if USE_MONOCAM
  if(selfID==0)
    uart2Init(115200); // only CF0 has monoCam and usart comm
#endif
  xTaskCreate(relativeControlTask,"relative_Control",configMINIMAL_STACK_SIZE, NULL,3,NULL );
  isInit = true;
}

PARAM_GROUP_START(relative_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_ADD(PARAM_FLOAT, relaCtrl_p, &relaCtrl_p)
PARAM_ADD(PARAM_FLOAT, relaCtrl_i, &relaCtrl_i)
PARAM_ADD(PARAM_FLOAT, relaCtrl_d, &relaCtrl_d)
PARAM_GROUP_STOP(relative_ctrl)

LOG_GROUP_START(mono_cam)
LOG_ADD(LOG_UINT8, charCam, &c)
LOG_GROUP_STOP(mono_cam)