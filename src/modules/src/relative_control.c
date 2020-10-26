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
// static float critical_laser = 1.0;
static float desired_velocity = 0.5; // speed in m/s that we aim for

// static int status = 0;
static bool isInit;
static bool onGround = true;
static bool keepFlying = false;
static int num_cycl;

// static bool follow_x = true;
// static bool wall_following = false;
static setpoint_t setpoint;
static float_t relaVarInCtrl[NumUWB][STATE_DIM_rl];
static float_t inputVarInCtrl[NumUWB][STATE_DIM_rl];
static uint8_t selfID;
static float_t height;

float lasers[4];
static float relaCtrl_p = 2.0f;
static float desired_heading = 0.0f;
static float relaCtrl_i = 0.0001f;
static float relaCtrl_d = 0.01f;
static float all_RS[NumUWB];
static float voltage_bias[NumUWB] = {0.0,0.0,0.0};
static float RS_lp[NumUWB] = {0.0,0.0,0.0};
// static float NDI_k = 2.0f;
static char c = 0; // monoCam
float search_range = 10.0; // search range in meters

// PSO-Specific
float r_p, r_g, v_x, v_y;
float omega = -0.166;
float phi_p = -0.466;
float phi_g = 2.665;

float old_vx = 0.0;
float old_vy = 0.0;

static float swarm_avoid_thres = 0.8; // 
static float wp_reached_thres = 0.5; // [m]
static float swarm_avoid_gain = 15.0f;
static float laser_repulse_gain = 5.0f;
static float warning_laser = 1.5; // start correcting if a laser ranger sees smaller than this
static int status = 0;
static int previous_status = 0;

bool fly_repulsion = true;
float laser_repulsion_thresh = 1.5;
float line_max_dist = 0.2;
float line_heading = 0.0;

int upper_idx, lower_idx, following_laser;
int current_ticks, started_wall_avoid_ticks, start_laser, max_reached_laser, start_laser_corrected, local_laser_idx;
int ticks_to_follow = 10000;
float following_heading;
bool search_left = false;
bool left_line = false;

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

struct Line {
  struct Point p0;
  struct Point p1;
  float a;
  float b; 
  float c; 
};

struct Point agent_pos,random_point;
struct Point goal = {.x = 0.0, .y= 0.0};

struct gas_Point agent_best = {.x = 0.0f, .y=0.0f, .gas_conc = 0.0f};
struct gas_Point swarm_best = {.x = 0.0f, .y=0.0f, .gas_conc = 0.0f};

struct Line line_to_goal;

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



void cap_heading(float* heading)
{
  while (*heading > 2.0f*(float)(M_PI))
  {
    *heading = *heading - 2.0f*(float)(M_PI);
  }
  while (*heading < 0.0f)
  {
    *heading = *heading + 2.0f*(float)(M_PI);
  }
}
float get_heading_to_point(struct Point agent, struct Point goal)
{
  float delta_x = goal.x - agent.x;
  float delta_y = goal.y - agent.y;
  float psi = atan2f(delta_y,delta_x);

  cap_heading(&psi);

  return psi;
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
       if (distance < swarm_avoid_thres)
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


float get_distance_points(struct Point p1, struct Point p2)
{
  return sqrtf(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
}



void cap_laser(int* laser)
{
  while (*laser > 3)
  {
    *laser = *laser - 4;
  }
  while (*laser < 0)
  {
    *laser = *laser + 4;
  }
}


void compute_directed_wp(void)
{
  r_g = (rand()/(float)RAND_MAX);
  r_p = (rand()/(float)RAND_MAX);

  v_x = omega*(goal.x-agent_pos.x)+phi_p*r_p*(agent_best.x-agent_pos.x)+phi_g*r_g*(swarm_best.x-agent_pos.x);
  v_y = omega*(goal.y-agent_pos.y)+phi_p*r_p*(agent_best.y-agent_pos.y)+phi_g*r_g*(swarm_best.y-agent_pos.y);
  goal.x = agent_pos.x + v_x;
  goal.y = agent_pos.y + v_y; 
  
}

void compute_random_wp(void)
{
  random_point.x = (rand()/(float)RAND_MAX)*search_range-0.5f*search_range ;
  random_point.y = (rand()/(float)RAND_MAX)*search_range-0.5f*search_range;

  v_x = 2.935f*(random_point.x)+0.457f*(goal.x-agent_pos.x);
  v_y = 2.935f*(random_point.y)+0.457f*(goal.y-agent_pos.y);
  goal.x = agent_pos.x + v_x;
  goal.y = agent_pos.y + v_y; 

  // // DEBUGGING!!!
  // goal = random_point;
  // goal.x = 10.0;
  // goal.y = -10.0;
}

void update_line_params(struct Line* line)
{
  float dx = line->p1.x - line->p0.x;
  float dy = line->p1.y - line->p0.y;
  // p0 and p1 are the same
  if (dx == 0 && dy ==0) 
  {
    line->a = 0;
    line->b = 0;
    line->c = 0;

  }
  // vertical line
  else if (dx == 0)
  {
    line->a = 1;
    line->b = 0;
    line->c = -line->p0.x;
  }
  // horizontal line
  else if (dy == 0)
  {
    line->a = 0;
    line->b = 1;
    line->c = -line->p0.y;
  }
  // neither horizontal nor vertical line
  else
  {
    line->a = 1;
    line->b = -dx/dy;
    line->c = -line->a*line->p0.x - line->b*line->p0.y ;
  }
}

void update_line(void)
{
  line_to_goal.p0 = agent_pos;
  line_to_goal.p1 = goal;
  update_line_params(&line_to_goal);
}

void update_direction(void)
{
  line_heading = get_heading_to_point(agent_pos,goal); // used to follow the line
  cap_heading(&line_heading);

  lower_idx = (int)(line_heading/(float)(M_PI_2));
  upper_idx = lower_idx+1;

  cap_laser(&lower_idx);
  cap_laser(&upper_idx);

  if (abs(line_heading-(lower_idx*(float)(M_PI_2))) > (float)(M_PI_4))
  {
    following_laser = upper_idx;
  }
  else
  {
    following_laser = lower_idx;
  }
}

void repulse_swarm(float* vx, float* vy)
{
  // attraction to source
  desired_heading = atan2f((goal.y-agent_pos.y),(goal.x-agent_pos.x));
  *(vx) = desired_velocity*cosf(desired_heading);
  *(vy) = desired_velocity*sinf(desired_heading);
  // fly_repulsion = false;
  // repulsion from other agents
  for (int i = 0; i<NumUWB; i++)
  {
    if ( i != selfID)
    {
      float distance = sqrtf(powf(relaVarInCtrl[i][STATE_rlX],2) + powf(relaVarInCtrl[i][STATE_rlY],2));
      if (distance < swarm_avoid_thres)
      {
        fly_repulsion = true;
        float heading_to_agent = atan2f(relaVarInCtrl[i][STATE_rlY],relaVarInCtrl[i][STATE_rlX]);
        float repulsion_heading = heading_to_agent + (float)(M_PI);
        *(vx) += swarm_avoid_gain*((swarm_avoid_thres-distance))*cosf(repulsion_heading);
        *(vy) += swarm_avoid_gain*((swarm_avoid_thres-distance))*sinf(repulsion_heading);
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
      *(vx) += cosf(laser_repulse_heading)*laser_repulse_gain*powf((warning_laser-lasers[i]),2);
      *(vy) += sinf(laser_repulse_heading)*laser_repulse_gain*powf((warning_laser-lasers[i]),2);
    }
  }

  float vector_size = sqrtf(powf(*(vx),2) + powf(*(vy),2));
  *(vx) = *(vx)/vector_size*desired_velocity;
  *(vy) = *(vy)/vector_size*desired_velocity;       
          
}


float get_distance_to_line(struct Line line , struct Point p0)
{
  return( fabsf(line.a*p0.x+line.b*p0.y+line.c) / ( sqrtf(powf(line.a,2)+powf(line.b,2)) ) );
}

bool back_in_line(void)
{
  if (get_distance_to_line(line_to_goal,agent_pos) > line_max_dist)
  {
    left_line = true;
  }
  if (left_line == true && get_distance_to_line(line_to_goal,agent_pos) < line_max_dist)
  {
    left_line = false;
    return true;
  }
  else
  {
    return false;
  }

}

void update_follow_laser(void)
{
  if (get_heading_to_point(agent_pos,goal) > line_heading)
  {
    following_laser = upper_idx;
  }
  else
  {
    following_laser = lower_idx;
  }
}

void follow_line( float* v_x, float* v_y)
{
  // check_passed_goal(ID);
  if (get_distance_to_line(line_to_goal,agent_pos) > line_max_dist)
  {
    update_follow_laser();
  }
  following_heading = following_laser*M_PI_2;
  *(v_x) = cosf(following_heading)*desired_velocity;
  *(v_y) = sinf(following_heading)*desired_velocity;
}

bool free_to_goal(void)
{
  if (lasers[upper_idx] > warning_laser && lasers[lower_idx] > warning_laser)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

void wall_follow_init(void)
{
  float temp_line_heading = get_heading_to_point(agent_pos,goal); // used to follow the line
  cap_heading(&temp_line_heading);

  int lower_idx_temp = (int)(temp_line_heading /(float)(M_PI_2));
  int upper_idx_temp = lower_idx_temp+1;
  cap_laser(&lower_idx_temp);
  cap_laser(&upper_idx_temp);

  float lower_heading = lower_idx_temp*(float)(M_PI_2);
  float upper_heading = upper_idx_temp*(float)(M_PI_2);
  cap_heading(&lower_heading);
  cap_heading(&upper_heading);

  if ( fabsf(temp_line_heading-lower_heading) > fabsf(temp_line_heading-upper_heading))
  {
    start_laser = upper_idx_temp;
    search_left = false;
  }
  else
  {
    start_laser = lower_idx_temp;
    search_left = true;
  }
}

void update_start_laser(void)
{
  if (search_left)
  {
    if (max_reached_laser > start_laser)
    {
      start_laser_corrected = max_reached_laser - 1;
    }
    else
    {
      start_laser_corrected = start_laser;
    }
  }
  else
  {
    if (max_reached_laser < start_laser)
    {
      start_laser_corrected = max_reached_laser + 1;
    }
    else
    {
      start_laser_corrected = start_laser;
    }
  }
}
void reset_wall_follower()
{
  if (search_left)
  {
    search_left = false;
  }
  else
  {
    search_left = true;
  }
  max_reached_laser = start_laser;
}

void follow_wall(float* v_x, float* v_y)
{
    // terminalinfo::debug_msg(std::to_string(search_left));
  update_start_laser(); // avoid osscialations

  if (search_left)
  {
    for (int i = start_laser_corrected; i < (start_laser+4); i++)
    {

      local_laser_idx = i;
      cap_laser(&local_laser_idx);

      if (lasers[local_laser_idx] > warning_laser)
      {
        if (i > max_reached_laser)
        {
          max_reached_laser = i;
        }
        break;
      }
    }
  }
  else
  {
    for (int i = start_laser_corrected; i > (start_laser-4); i--)
    {
      local_laser_idx = i;
      if (lasers[local_laser_idx] > warning_laser)
      {
        if (i < max_reached_laser)
        {
          max_reached_laser = i;
        }
        break;
      }
      
    }
  }

  if(lasers[local_laser_idx] < warning_laser)
  {
    reset_wall_follower();
  }

  following_laser = local_laser_idx;
  *(v_x) = cosf(following_laser*(float)(M_PI_2))*desired_velocity;
  *(v_y) = sinf(following_laser*(float)(M_PI_2))*desired_velocity;
}


void update_status(void)
{

current_ticks = xTaskGetTickCount();



  if (check_collision())
  {
    status = 2;
  }
  else if (previous_status ==2)
  {
    status = 0; 
    update_line();
    update_direction();
  }
  // else if (free_to_goal() && previous_status == 1)
  else if (back_in_line() && previous_status == 1)
  {
    status = 0;
    update_line();
    update_direction();
  }
  else if (!free_to_goal() && previous_status == 0)
  {
    status = 1;
    wall_follow_init();
    started_wall_avoid_ticks = xTaskGetTickCount();
  }
  previous_status = status;

}

void relativeControlTask(void* arg)
{
  systemWaitStart();
  // height = (float)selfID*0.1f+0.2f;
  height = 0.5f;

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
        xTaskGetTickCount();
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
        

        if (swarm_best.gas_conc < 0.2f)
        {
          compute_random_wp();
          update_line();
          update_direction();
          num_cycl = 794;
        }
        else
        {
          compute_directed_wp();
          update_line();
          update_direction();
          num_cycl = 111;
        }
        status = 0;

        for (int i = 0; i< num_cycl; i++) //time before time-out
        {
          // update all gas sensors
          get_all_RS(all_RS);
          update_lowpass(all_RS);
          update_wps(); // update to new individual and swarm best

          estimatorKalmanGetEstimatedPos(&state); //read agent state from kalman filter
          relativeInfoRead((float_t *)relaVarInCtrl, (float_t *)inputVarInCtrl); //get relative state from other agents
          getDistances(lasers); // get laser ranger readings, order: front, left, back, right
          // min_laser = get_min(lasers); // get minimum value of laser rangers
          agent_pos.x = state.x;
          agent_pos.y = state.y;
          wp_dist = get_distance_points(agent_pos,goal);
          update_status();
          
          if (wp_dist < wp_reached_thres )
          {
            break;
          }

          switch (status)
          {
          case 0:
            follow_line(&vx,&vy);
            break;
          
          case 1:
            follow_wall(&vx,&vy);
            break;
      
          case 2:
            repulse_swarm(&vx,&vy);
            break;
          }          

        setHoverSetpoint(&setpoint,vx,vy,height,0);
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