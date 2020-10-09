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
#include "range.h"
#define USE_MONOCAM 0
#include "stabilizer_types.h"
#include "estimator_kalman.h"

// static float RAD2DEG = 57.29578049;
// static float critical_laser = 0.5; // no laser ranger should ever see lower than this
static float warning_laser = 1.3; // start correcting if a laser ranger sees smaller than this
// static float critical_laser = 1.0;
static float desired_velocity = 0.5; // speed in m/s that we aim for

// static int status = 0;
static bool isInit;
static bool onGround = true;
static bool keepFlying = false;
static int heighest_reached = 0;
// static bool follow_x = true;
// static bool wall_following = false;
static setpoint_t setpoint;
static float_t relaVarInCtrl[NumUWB][STATE_DIM_rl];
static float_t inputVarInCtrl[NumUWB][STATE_DIM_rl];
static uint8_t selfID;
static float_t height;
static int start_checking;
static int max_turns = 2;
static int free_lasers[4];
static int start_laser = 0;
static int previous_free_laser = 0;
float lasers[4];
static float relaCtrl_p = 2.0f;
static float relaCtrl_i = 0.0001f;
static float relaCtrl_d = 0.01f;
static float wp_reached_thres = 0.2; // [m]
static bool turn_positive = true;
// static float NDI_k = 2.0f;
static char c = 0; // monoCam
float search_range = 10.0; // search range in meters

float min_laser = 10.0f;
int laser_decision;

struct Point
{
    float x;
    float y;
};

// front, left, back right (ENU-based)
void getDistances(float* d) {
    *(d+0) = rangeGet(rangeFront)*0.001f;
    *(d+1) = rangeGet(rangeLeft)*0.001f;
    *(d+2) = rangeGet(rangeBack)*0.001f;
    *(d+3) = rangeGet(rangeRight)*0.001f;
}




float get_min(float* d)
{
  float min = d[0];
  if (d[1] < min)
  {
    min = d[1];
  }
  if (d[2] < min)
  {
    min = d[2];
  }
  if (d[3] < min)
  {
    min = d[3];
  }
  return min;
}

// function to determine if, given an obstacle, 'yawing' right or left is best
// d is given as front, left, back, right
// return 0 if 'yawing' positive is desired (ENU)
// return 1 if 'yawing' negative is desired (ENU)
// return 2 if no danger is present in the current movement direction


struct Point agent_pos,goal, random_point;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
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
static void setHover_pos_Setpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->position.x = vx;
  setpoint->position.y = vy;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}


static void flyRandomIn1meter(void){
  float_t rand_x = (rand()/(float)RAND_MAX)*(1.0f)-0.5f;
  float_t rand_y = (rand()/(float)RAND_MAX)*(1.0f)-0.5f;

  for (int i=1; i<100; i++) {
    setHover_pos_Setpoint(&setpoint, rand_x, rand_y, height, 0);
    vTaskDelay(M2T(10));
  }
}


void flyVerticalInterpolated(float startz, float endz, float interpolate_time) {
    setpoint_t setpoint;
    int CMD_TIME = 100;   // new command ever CMD_TIME seconds
    int NSTEPS = (int) interpolate_time / CMD_TIME; 
    for (int i = 0; i < NSTEPS; i++) {
        float curr_height = startz + (endz - startz) * ((float) i / (float) NSTEPS);
        setHoverSetpoint(&setpoint, 0, 0, curr_height, 0); 
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

float get_distance_points(struct Point p1, struct Point p2)
{
  return sqrtf(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
}

void relativeControlTask(void* arg)
{
  systemWaitStart();
  // height = (float)selfID*0.1f+0.2f;
  height = 0.5f;

  float heading = 0.f;
  float vx = 0.f;
  float vy = 0.f;
  float wp_dist = 0.f;

  getDistances(lasers);
  while(1) {
    vTaskDelay(10);
    getDistances(lasers);
    keepFlying = command_share(selfID, keepFlying);
    // DEBUG_PRINT("%d %d \n", keepFlying,relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl) );
    if(relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl) && keepFlying){
    // if(keepFlying){
      // take off
      if(onGround){
        // flyVerticalInterpolated(0.0,height,3000.0f);
        estimatorKalmanInit(); // reseting kalman filter
        for (int i=0; i<50; i++) {
          setHover_pos_Setpoint(&setpoint, 0, 0, 0.3f, 0);
          vTaskDelay(M2T(100));
        }
        for (int i=0; i<20; i++) {
          flyRandomIn1meter();
        }

        onGround = false;
      }
      else
      {
        point_t state;
        random_point.x = (rand()/(float)RAND_MAX)*search_range-0.5f*search_range;
        random_point.y = (rand()/(float)RAND_MAX)*search_range-0.5f*search_range;
        goal = random_point;
        // float accumulator_obs_avoidance = 0.0f;

        for (int i = 0; i< 70; i++) //time before time-out
        {
          estimatorKalmanGetEstimatedPos(&state); //read agent state from kalman filter
          relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl); //get relative state from other agents
          getDistances(lasers); // get laser ranger readings, order: front, left, back, right
          // min_laser = get_min(lasers); // get minimum value of laser rangers
          agent_pos.x = state.x;
          agent_pos.y = state.y;
          wp_dist = get_distance_points(agent_pos,random_point);

          if (check_collision())
          {
            flyVerticalInterpolated(height,0.0f,6000.0f);
            keepFlying = 0;
            break;
          }
          int first_free_laser = -1;
          // DEBUG_PRINT("%d",turn_positive);
          if ( wp_dist > wp_reached_thres)
          {
            if (heighest_reached > start_laser)
            {
              start_checking = heighest_reached -1;
            }
            else
            {
              start_checking = start_laser;
            }
            
            if (turn_positive)
            {
              for (int i = start_checking; i < (start_laser+4); i++)
              {
                DEBUG_PRINT("%d \n",i);
                int laser_idx;
                if (i > 3)
                {
                  laser_idx = i - 4;
                }
                else if(i < 0)
                {
                  laser_idx = i + 4;
                }
                else
                {
                  laser_idx = i;
                }
                

                if (lasers[laser_idx] > warning_laser)
                {
                  free_lasers[laser_idx] = 1;
                  if (first_free_laser == -1)
                  {
                    first_free_laser = i;
                  }
                }
                else
                {
                  free_lasers[laser_idx] = 0;
                }
                
              }
            }
            else
            {
              if (heighest_reached < start_laser)
              {
                start_checking = heighest_reached + 1;
              }
              else
              {
                start_checking = start_laser;
              }
            
              for (int i = start_checking; i > (start_laser-4); i--)
              {
                
                int laser_idx;
                if (i > 3)
                {
                  laser_idx = i - 4;
                }
                else if(i < 0)
                {
                  laser_idx = i + 4;
                }
                else
                {
                  laser_idx = i;
                }
                

                if (lasers[laser_idx] > warning_laser)
                {
                  free_lasers[laser_idx] = 1;
                  if (first_free_laser == -1)
                  {
                    
                    DEBUG_PRINT("%d \n",i);
                    first_free_laser = i;
                  }
                }
                else
                {
                  free_lasers[laser_idx] = 0;
                }
              }
            }
            
            if (turn_positive)
            {
              previous_free_laser = first_free_laser;
              if (first_free_laser > heighest_reached)
              {
                heighest_reached = first_free_laser;
              }
              if ((abs(heighest_reached - start_laser)) > max_turns)
              {
                turn_positive = false;
                // start_laser = heighest_reached - 1;
                max_turns = 2;
              }

            }

            else
            {
              previous_free_laser = first_free_laser;

              if (first_free_laser < heighest_reached)
              {
                heighest_reached = first_free_laser;
              }
              if ((abs(heighest_reached - start_laser)) > max_turns)
              {
                turn_positive = true;
                // start_laser = heighest_reached + 1;
                // heighest_reached = start_laser;
                max_turns = 2;
                // max_turns = 4;
              }

            }

            heading = M_PI_2*first_free_laser;

            // heading = atan2f((goal.y-agent_pos.y),(goal.x-agent_pos.x)); // heading in deg (ENU) to desired waypoint
            vx = desired_velocity*cosf(heading);
            vy = desired_velocity*sinf(heading);
          
            setHoverSetpoint(&setpoint,vx,vy,height,0);
          }
          else
          {
            setHoverSetpoint(&setpoint,0,0,height,0);
          }
          
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

LOG_GROUP_START(lasers)
LOG_ADD(LOG_FLOAT,front,&lasers[0])
LOG_ADD(LOG_FLOAT,left,&lasers[1])
LOG_ADD(LOG_FLOAT,back,&lasers[2])
LOG_ADD(LOG_FLOAT,right,&lasers[3])
LOG_GROUP_STOP(lasers)