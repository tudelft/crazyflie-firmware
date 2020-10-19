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
#include "deck_analog.h"

// static float RAD2DEG = 57.29578049;
// static float critical_laser = 0.5; // no laser ranger should ever see lower than this
static float warning_laser = 2.5; // start correcting if a laser ranger sees smaller than this
// static float critical_laser = 1.0;
static float desired_velocity = 0.8; // speed in m/s that we aim for

// static int status = 0;
static bool isInit;
static bool onGround = true;
static bool keepFlying = false;

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
static bool turn_positive = true;
static int heighest_reached = 0;

float lasers[4];
static float relaCtrl_p = 2.0f;
static float desired_heading = 0.0f;
static float relaCtrl_i = 0.0001f;
static float relaCtrl_d = 0.01f;
static float wp_reached_thres = 0.5; // [m]
static float all_RS[NumUWB];
static float voltage_bias[NumUWB] = {0.0,0.0,0.0};
static float RS_lp[NumUWB] = {0.0,0.0,0.0};
// static float NDI_k = 2.0f;
static char c = 0; // monoCam
float search_range = 10.0; // search range in meters

float min_laser = 10.0f;
int laser_decision;
float r_p, r_g, v_x, v_y;
// PSO-Specific
float rand_p = 0.1;
float omega = 0.4;
float phi_p = 0.4;
float phi_g = 0.6;
float old_vx = 0.0;
float old_vy = 0.0;

bool fly_repulsion = true;
float laser_repulsion_thresh = 2.0;

struct Point
{
    float x;
    float y;
};

struct gas_Point
{
    float x ;
    float y ;
    float gas_conc;
};

struct Point agent_pos,random_point;
struct Point goal = {.x = 0.0, .y= 0.0};

struct gas_Point agent_best = {.x = 0.0f, .y=0.0f, .gas_conc = 0.0f};
struct gas_Point swarm_best = {.x = 0.0f, .y=0.0f, .gas_conc = 0.0f};

void get_RS(float* R_s)
{
  uint32_t pin = 10;
  *(R_s) = analogReadVoltage(pin) - voltage_bias[selfID];
}


void get_all_RS(float* all_RS)
{
  get_swarm_gas(all_RS); // load all swarm gas data
  get_RS(all_RS + selfID); // load individual swarm gas reading
}

void update_lowpass(float* all_RS)
{
  for (int i = 0; i<NumUWB; i++)
  {
    RS_lp[i] = RS_lp[i]*0.9f + 0.1f*(*(all_RS+i));
  }
}

void set_voltage_offset(void)
{
  for (int i = 0; i <NumUWB; i++)
  {
    voltage_bias[i] = RS_lp[i];
  }
}

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




static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yawrate;
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
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yawrate;
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->position.x = vx;
  setpoint->position.y = vy;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}

void reset_lp(void)
{
  for (int i = 0; i<NumUWB; i++)
  {
    RS_lp[i] = 0.0f;
  }
}

static void flyRandomIn1meter(void){
  float_t rand_x = (rand()/(float)RAND_MAX)*(1.0f)-0.5f;
  float_t rand_y = (rand()/(float)RAND_MAX)*(1.0f)-0.5f;
  
  for (int i=1; i<100; i++) {
    setHover_pos_Setpoint(&setpoint, rand_x, rand_y, height, 0);
    vTaskDelay(M2T(10));
    get_all_RS(all_RS);
    update_lowpass(all_RS);
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
       if (distance < 2.0f)
       {
         collision = true;
       }       
    }
  }
  return collision;
}

void update_wps(void)
{
  // update local wp
  if (RS_lp[selfID] > agent_best.gas_conc)
  {
    agent_best.gas_conc = RS_lp[selfID];
    agent_best.x = agent_pos.x;
    agent_best.y = agent_pos.y;
  }

  // update global wp
  for (int i = 0; i < NumUWB ; i++)
  {
    if ( RS_lp[i] > swarm_best.gas_conc)
    {
      if (i != selfID)
      {
        swarm_best.gas_conc = RS_lp[i];
        swarm_best.x = agent_pos.x + relaVarInCtrl[i][STATE_rlX];
        swarm_best.y = agent_pos.y + relaVarInCtrl[i][STATE_rlY];
      }
      else
      {
        swarm_best.gas_conc = RS_lp[i];
        swarm_best.x = agent_pos.x; 
        swarm_best.y = agent_pos.y;
      }
      
    }
  }

}

void update_wps_test(void)
{
  float min = 0.0;
  // update global wp
  for (int i = 0; i < NumUWB ; i++)
  {
    if ( RS_lp[i] > min)
    {
      if (i != selfID)
      {
        swarm_best.gas_conc = RS_lp[i];
        min = swarm_best.gas_conc;
        swarm_best.x = agent_pos.x + relaVarInCtrl[i][STATE_rlX];
        swarm_best.y = agent_pos.y + relaVarInCtrl[i][STATE_rlY];
        agent_best = swarm_best;
      }
      else
      {
        swarm_best.gas_conc = RS_lp[i];
        swarm_best.x = agent_pos.x; 
        swarm_best.y = agent_pos.y;
        agent_best = swarm_best;
        min = swarm_best.gas_conc;
      } 
    }
  }
}



float get_distance_points(struct Point p1, struct Point p2)
{
  return sqrtf(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
}

void cap_heading(float* heading)
{
  if (*heading > 2.0f*(float)(M_PI))
  {
    *heading = *heading - 2.0f*(float)(M_PI);
  }
  else if (*heading < 0.0f)
  {
    *heading = *heading + 2.0f*(float)(M_PI);
  }
}

void cap_laser(int* laser)
{
  if (*laser > 3)
  {
    *laser = *laser - 4;
  }
  else if (*laser < 0)
  {
    *laser = *laser + 4;
  }
}



// turn_positive
// start_laser
void determine_direction(void)
{
  float capped_heading;
  capped_heading = desired_heading;
  cap_heading(&capped_heading);

  int lower_idx = (int)(capped_heading/((float)M_PI_2)); // lower index of heading in ENU
  float diff = capped_heading - (float)(lower_idx)*(float)(M_PI_2);

  if (diff >(float)(M_PI_2))
  {
    start_laser = lower_idx + 1;
    turn_positive = false;
  }
  else
  {
    start_laser = lower_idx;
    turn_positive = true;
  }

  cap_laser(&start_laser);  

}

void reset_follow_vars(void)
{
  start_checking = start_laser;
  max_turns = 2;
  heighest_reached = start_laser;
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
    
    get_all_RS(all_RS);
    update_lowpass(all_RS);
    
    // DEBUG_PRINT("%d %d \n", keepFlying,relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl) );
    if(relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl) && keepFlying){
    // if(keepFlying){
      // take off
      
      if(onGround){
        // flyVerticalInterpolated(0.0,height,3000.0f);
        set_voltage_offset();
        reset_lp();
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
        // update all gas readings and agent positions
        get_all_RS(all_RS);
        update_lowpass(all_RS);
        point_t state;
        estimatorKalmanGetEstimatedPos(&state);
        agent_pos.x = state.x;
        agent_pos.y = state.y;

        update_wps();
        // compute next wp
        random_point.x = (rand()/(float)RAND_MAX)*search_range-0.5f*search_range;
        random_point.y = (rand()/(float)RAND_MAX)*search_range-0.5f*search_range;

        r_g = (rand()/(float)RAND_MAX);
        r_p = (rand()/(float)RAND_MAX);

        if (swarm_best.gas_conc < 0.2f)
        {
          v_x = 0.5f*(random_point.x)+0.5f*(goal.x-agent_pos.x);
          v_y = 0.5f*(random_point.y)+0.5f*(goal.y-agent_pos.y);
          goal.x = agent_pos.x + v_x;
          goal.y = agent_pos.y + v_y; 
        }
        else
        {
          v_x = rand_p*(random_point.x)+omega*(goal.x-agent_pos.x)+phi_p*r_p*(agent_best.x-agent_pos.x)+phi_g*r_g*(swarm_best.x-agent_pos.x);
          v_y = rand_p*(random_point.y)+omega*(goal.y-agent_pos.y)+phi_p*r_p*(agent_best.y-agent_pos.y)+phi_g*r_g*(swarm_best.y-agent_pos.y);
          goal.x = agent_pos.x + v_x;
          goal.y = agent_pos.y + v_y; 
        }
        



        // float accumulator_obs_avoidance = 0.0f;
        determine_direction();
        reset_follow_vars();
        for (int i = 0; i< 50; i++) //time before time-out
        {
          get_all_RS(all_RS);
          update_lowpass(all_RS);
          update_wps_test(); // update to new individual and swarm best
          estimatorKalmanGetEstimatedPos(&state); //read agent state from kalman filter
          relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl); //get relative state from other agents
          getDistances(lasers); // get laser ranger readings, order: front, left, back, right
          // min_laser = get_min(lasers); // get minimum value of laser rangers
          agent_pos.x = state.x;
          agent_pos.y = state.y;
          wp_dist = get_distance_points(agent_pos,goal);
          
          if (wp_dist < wp_reached_thres )
          {
            break;
          }

          // if some other member of the swarm is in close proximity
          
          if (fly_repulsion)
          {
            // attraction to source
            desired_heading = atan2f((goal.y-agent_pos.y),(goal.x-agent_pos.x));
            vx = desired_velocity*cosf(desired_heading);
            vy = desired_velocity*sinf(desired_heading);
            // fly_repulsion = false;
            // repulsion from other agents
            for (int i = 0; i<NumUWB; i++)
            {
              if ( i != selfID)
              {
                float distance = sqrtf(powf(relaVarInCtrl[i][STATE_rlX],2) + powf(relaVarInCtrl[i][STATE_rlY],2));
                if (distance < 1.0f)
                {
                  fly_repulsion = true;
                  float heading_to_agent = atan2f(relaVarInCtrl[i][STATE_rlY],relaVarInCtrl[i][STATE_rlX]);
                  float repulsion_heading = heading_to_agent + (float)(M_PI);
                  vx += 15.0f*((1.0f-distance))*cosf(repulsion_heading);
                  vy += 15.0f*((1.0f-distance))*sinf(repulsion_heading);
                }
       
              }
            }

            // repulsion from lasers
            for (int i = 0; i < 4; i++)
            {
              if (lasers[i] < warning_laser)
              {
                float laser_heading = (float)(i)*(float)(M_PI_2);
                float laser_repulse_heading = laser_heading + (float)(M_PI);
                vx += cosf(laser_repulse_heading)*3.0f*powf((warning_laser-lasers[i]),2);
                vy += sinf(laser_repulse_heading)*3.0f*powf((warning_laser-lasers[i]),2);
              }
            }

            float vector_size = sqrtf(powf(vx,2) + powf(vy,2));
            vx = vx/vector_size*desired_velocity;
            vy = vy/vector_size*desired_velocity;
            
            vx = 0.9f*old_vx + 0.1f*vx;
            vy = 0.9f*old_vy + 0.1f*vy;
            old_vx = vx;
            old_vy = vy;

            setHoverSetpoint(&setpoint,vx,vy,height,0);
            
            if (fly_repulsion == false)
            {
              determine_direction();
              reset_follow_vars();
            }
            // float effective_heading = atan2f(vy,vx);
            // if (abs(effective_heading-desired_heading)>M_PI_2)
            // {
            //   break;
            // }
 
          }
          else
          {
          desired_heading = atan2f((goal.y-agent_pos.y),(goal.x-agent_pos.x));
 
          
          int first_free_laser = -1;
          // DEBUG_PRINT("%d",turn_positive);
         
            if (turn_positive)
            {
            
              if (heighest_reached > start_laser)
              {
                start_checking = heighest_reached -1;
              }
              else
              {
                start_checking = start_laser;
              }

              for (int i = start_checking; i < (start_checking+4); i++)
              {
                // DEBUG_PRINT("%d \n",i);
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
            
              for (int i = start_checking; i > (start_checking-4); i--)
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
                    
                    // DEBUG_PRINT("%d \n",i);
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
              if (first_free_laser > heighest_reached)
              {
                heighest_reached = first_free_laser;
              }
              if ((abs(heighest_reached - start_laser)) > max_turns)
              {
                determine_direction();
                reset_follow_vars();
              }

            }

            else
            {
              if (first_free_laser < heighest_reached)
              {
                heighest_reached = first_free_laser;
              }
              if ((abs(heighest_reached - start_laser)) > max_turns)
              {
                determine_direction();
                reset_follow_vars();
              }

            }

            if (check_collision())
            {
              fly_repulsion = true;
            }

            heading = M_PI_2*first_free_laser;

            // heading = atan2f((goal.y-agent_pos.y),(goal.x-agent_pos.x)); // heading in deg (ENU) to desired waypoint
            vx = desired_velocity*cosf(heading);
            vy = desired_velocity*sinf(heading);
          
            setHoverSetpoint(&setpoint,vx,vy,height,0);
     

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
LOG_ADD(LOG_FLOAT,gas0,&all_RS[0])
LOG_ADD(LOG_FLOAT,gas1,&all_RS[1])
LOG_ADD(LOG_FLOAT,gas2,&all_RS[2])

LOG_ADD(LOG_FLOAT,front,&lasers[0])
LOG_ADD(LOG_FLOAT,left,&lasers[1])
LOG_ADD(LOG_FLOAT,back,&lasers[2])
LOG_ADD(LOG_FLOAT,right,&lasers[3])

LOG_GROUP_STOP(lasers)

LOG_GROUP_START(PSO)
LOG_ADD(LOG_FLOAT,goal_x,&goal.x)
LOG_ADD(LOG_FLOAT,goal_y,&goal.y)
LOG_ADD(LOG_FLOAT,agent_x,&agent_pos.x)
LOG_ADD(LOG_FLOAT,agent_y,&agent_pos.y)
LOG_ADD(LOG_FLOAT,swarm_best_x,&swarm_best.x)
LOG_ADD(LOG_FLOAT,swarm_best_y,&swarm_best.y)
LOG_ADD(LOG_FLOAT,gas0_lp,&RS_lp[0])
LOG_ADD(LOG_FLOAT,gas1_lp,&RS_lp[1])
LOG_ADD(LOG_FLOAT,gas2_lp,&RS_lp[2])
LOG_GROUP_STOP(PSO)