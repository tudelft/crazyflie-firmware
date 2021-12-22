#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_localization.h"
#include "num.h"
#include "param.h"
#include "debug.h"
#include "config.h"
#include <stdlib.h> // random
#include "lpsTwrTag.h" // UWBNum
#include "configblock.h"
#include "uart2.h"
#include "log.h"
#include <math.h>
#include "estimator_kalman.h"
#include "estimator_complementary.h"
#include "stabilizer_types.h"
#include "pid.h"

#define RELATIVE_CONTROL_RATE RATE_100_HZ
#define RELATIVE_CONTROL_DT 1.0f/RELATIVE_CONTROL_RATE

static bool isInit;
static bool onGround = true;
static bool keepFlying = false;
static setpoint_t setpoint;
static float relaVarInCtrl[NumUWB][STATE_DIM_rl];
static float inputVarInCtrl[NumUWB][STATE_DIM_rl];
static uint8_t selfID;
static float height;

// PID control
#define PID_RELXY_KP 3.5f
#define PID_RELXY_KI 0.0f
#define PID_RELXY_KD 0.06f
#define PID_RELXY_OUTPUT_LIM 2.0f

#define PID_RELZ_KP 3.5f
#define PID_RELZ_KI 0.001f
#define PID_RELZ_KD 0.06f
#define PID_RELZ_OUTPUT_LIM 2.0f

#define PID_INTEGRAL_LIM 0.5f

static PidObject pid_relX, pid_relY, pid_relZ;
static float desired_relX = 2.0f;
static float desired_relY = 0.0f;
static float desired_relZ = 0.2f;

// Shushuai's Proposed Gains (CF)
// static float relaCtrl_p = 1.5f;
// static float relaCtrl_i = 0.0001f;
// static float relaCtrl_d = 0.01f;
float pid_vx, pid_vy, pid_vz;
// static float NDI_k = 2.0f;

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

static void sendPositionSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw){
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yaw;
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}

static void set3DVel(setpoint_t *setpoint, float vx, float vy, float vz, float yawrate)
{
  setpoint->mode.z = modeVelocity;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity.z = vz;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}

static void flyRandomIn1meter(float vel){
  float randomYaw = (rand() / (float)RAND_MAX) * 6.28f; // 0-2pi rad
  float randomVel = vel*(rand() / (float)RAND_MAX); // 0-1 m/s
  float vxBody = randomVel * cosf(randomYaw);
  float vyBody = randomVel * sinf(randomYaw);
  for (int i=1; i<100; i++) {
    setHoverSetpoint(&setpoint, vxBody, vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
  for (int i=1; i<100; i++) {
    setHoverSetpoint(&setpoint, -vxBody, -vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
}

#define SIGN(a) ((a>=0)?1:-1)
static float targetX;
static float targetY;
static float targetZ;
// static float PreErr_x = 0;
// static float PreErr_y = 0;
// static float PreErr_z = 0;
// static float IntErr_x = 0;
// static float IntErr_y = 0;
// static float IntErr_z = 0;
// static uint32_t PreTime;
// static float pid_dt;

// static void formation0asCenter(float tarX, float tarY, float tarZ){
//   pid_dt = (float) T2M(xTaskGetTickCount()-PreTime)/1000;
//   PreTime = xTaskGetTickCount();
//   if(pid_dt > 1) // skip the first run of the EKF
//     return;
//   // pid control for formation flight
//   float err_x = -(tarX - relaVarInCtrl[0][STATE_rlX]);
//   float err_y = -(tarY - relaVarInCtrl[0][STATE_rlY]);
//   float err_z = -(tarZ - relaVarInCtrl[0][STATE_rlZ]);
//   pid_vx = PID_RELXY_KP * err_x;
//   pid_vy = PID_RELXY_KP * err_y;
//   pid_vz = PID_RELZ_KP * err_z;
//   float dx = (err_x - PreErr_x) / pid_dt;
//   float dy = (err_y - PreErr_y) / pid_dt;
//   float dz = (err_z - PreErr_z) / pid_dt;
//   PreErr_x = err_x;
//   PreErr_y = err_y;
//   PreErr_z = err_z;
//   pid_vx += PID_RELXY_KD * dx;
//   pid_vy += PID_RELXY_KD * dy;
//   pid_vz += PID_RELZ_KD * dz;
//   IntErr_x += err_x * pid_dt;
//   IntErr_y += err_y * pid_dt;
//   IntErr_z += err_z * pid_dt;
//   pid_vx += PID_RELXY_KI * constrain(IntErr_x, -0.5, 0.5);
//   pid_vy += PID_RELXY_KI * constrain(IntErr_y, -0.5, 0.5);
//   pid_vz += PID_RELZ_KI * constrain(IntErr_z, -0.5, 0.5);
//   pid_vx = constrain(pid_vx, -2.5f, 2.5f);
//   pid_vy = constrain(pid_vy, -2.5f, 2.5f);
//   pid_vz = constrain(pid_vz, -1.5f, 1.5f);

//   // float rep_x = 0.0f;
//   // float rep_y = 0.0f;
//   // for(uint8_t i=0; i<NumUWB; i++){
//   //   if(i!=selfID){
//   //     float dist = relaVarInCtrl[i][STATE_rlX]*relaVarInCtrl[i][STATE_rlX] + relaVarInCtrl[i][STATE_rlY]*relaVarInCtrl[i][STATE_rlY];
//   //     dist = sqrtf(dist);
//   //     rep_x += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlX]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlX]);
//   //     rep_y += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlY]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlY]);
//   //   }
//   // }
//   // rep_x = constrain(rep_x, -1.5f, 1.5f);
//   // rep_y = constrain(rep_y, -1.5f, 1.5f);

//   // pid_vx = constrain(pid_vx + rep_x, -1.5f, 1.5f);
//   // pid_vy = constrain(pid_vy + rep_y, -1.5f, 1.5f);

//   set3DVel(&setpoint, pid_vx, pid_vy, pid_vz, 0);
// }



void relativeControlTask(void* arg)
{
  static uint32_t tick;
  static uint32_t takeoff_tick;
  systemWaitStart();
  static logVarId_t logIdStateIsFlying;
  logIdStateIsFlying = logGetVarId("controller", "inFlight");
  // height = (float)selfID*0.1f+0.2f;
  while(1) {
    vTaskDelay(M2T(1));
    tick = xTaskGetTickCount();
    if (RATE_DO_EXECUTE(RELATIVE_CONTROL_RATE, tick)){
      // Leader drone: share if flying
      if(selfID==0){
        keepFlying = logGetUint(logIdStateIsFlying);
        keepFlying = command_share(selfID, keepFlying);       
        continue;
      }

      // Follower: Perform relative control if leader is flying and data is available
      keepFlying = command_share(selfID, keepFlying);
      if(relativeInfoRead((float *)relaVarInCtrl, (float *)inputVarInCtrl) && keepFlying){
        
        // take off if still on ground (includes delays)
        if(onGround){
          estimatorComplementaryInit(); // reseting kalman filter
          vTaskDelay(M2T(2000));
          for (int i=0; i<50; i++) {
            setHoverSetpoint(&setpoint, 0, 0, 1.0f, 0);
            vTaskDelay(M2T(100));
          }
          onGround = false;
          tick = xTaskGetTickCount();
          takeoff_tick = tick;
        }

        uint32_t time_since_takeoff = T2M(tick - takeoff_tick);
        // Initialization procedure to allow relative EKF to converge
        if (false){
          // 2.0/2.0/1.0
          // 2.0/2.0/2.0
          // -2.0/2.0/2.0
          // -2.0/2.0/1.0
          // -2.0/-2.0/1.0
          // -2.0/-2.0/2.0
          // 2.0/-2.0/2.0
          // 2.0/-2.0/1.0
          // 0.0/0.0/1.0
          if ((time_since_takeoff > 2000) && (time_since_takeoff < 4000))
            sendPositionSetpoint(&setpoint, 2.0f, 2.0f, 1.0f, 0.0f);
          else if (time_since_takeoff < 6000)
            sendPositionSetpoint(&setpoint, 2.0f, 2.0f, 2.0f, 0.0f);
          else if (time_since_takeoff < 8000)
            sendPositionSetpoint(&setpoint, -2.0f, 2.0f, 2.0f, 0.0f);
          else if (time_since_takeoff < 10000)
            sendPositionSetpoint(&setpoint, -2.0f, 2.0f, 1.0f, 0.0f);
          else if (time_since_takeoff < 12000)
            sendPositionSetpoint(&setpoint, -2.0f, -2.0f, 1.0f, 0.0f);
          else if (time_since_takeoff < 14000)
            sendPositionSetpoint(&setpoint, -2.0f, -2.0f, 2.0f, 0.0f);
          else if (time_since_takeoff < 16000)
            sendPositionSetpoint(&setpoint, 2.0f, -2.0f, 2.0f, 0.0f);
          else if (time_since_takeoff < 18000)
            sendPositionSetpoint(&setpoint, 2.0f, -2.0f, 1.0f, 0.0f);
          else
            takeoff_tick = tick;
        }

        if( time_since_takeoff < 15000){
          flyRandomIn1meter(2.0f); // random flight within first 15 seconds
          if ((time_since_takeoff > 2000) && (time_since_takeoff < 4000))
              height = 1.3;
          if ((time_since_takeoff > 4000) && (time_since_takeoff < 6000))
              height = 1.5;
          if ((time_since_takeoff > 6000) && (time_since_takeoff < 8000))
              height = 1.7;
          if ((time_since_takeoff > 8000) && (time_since_takeoff < 10000))
              height = 1.4;
          if ((time_since_takeoff > 10000) && (time_since_takeoff < 12000))
              height = 1.8;
          if ((time_since_takeoff > 12000) && (time_since_takeoff < 14000))
              height = 1.3;
          if ((time_since_takeoff > 14000))
              height = 1.7;
        }else{
          // float t_sec = time_since_takeoff/1000.0f;
          // desired_relX = 2.0f*cosf(t_sec);
          // desired_relY = 2.0f*sinf(t_sec);
          // desired_relZ = 0.5f*cosf(t_sec);
          
          targetX = desired_relX; //-cosf(relaVarInCtrl[0][STATE_rlYaw])*relaXof2in1 + sinf(relaVarInCtrl[0][STATE_rlYaw])*relaYof2in1;
          targetY = desired_relY; //-sinf(relaVarInCtrl[0][STATE_rlYaw])*relaXof2in1 - cosf(relaVarInCtrl[0][STATE_rlYaw])*relaYof2in1;
          targetZ = desired_relZ;
          // formation0asCenter(targetX, targetY, targetZ); 

          pidSetDesired(&pid_relX, targetX);
          pidSetDesired(&pid_relY, targetY);
          pidSetDesired(&pid_relZ, targetZ);

          // need to reverse velocity because relative position is in follower frame of reference,
          // i.e. if desired=2, current=1 need to fly backwards (not forward)
          pid_vx = -pidUpdate(&pid_relX, relaVarInCtrl[0][STATE_rlX], true);
          pid_vy = -pidUpdate(&pid_relY, relaVarInCtrl[0][STATE_rlY], true);
          pid_vz = -pidUpdate(&pid_relZ, relaVarInCtrl[0][STATE_rlZ], true);

          set3DVel(&setpoint, pid_vx, pid_vy, pid_vz, 0);
        }

      }else{
        // landing procedure
        if(!onGround){
          for (int i=1; i<5; i++) {
            if(selfID!=0){
            setHoverSetpoint(&setpoint, 0, 0, 0.3f-(float)i*0.05f, 0);
            vTaskDelay(M2T(10));
            }
          }
          onGround = true;
        } 
      }
    }
  }
}

void relativeControlInit(void)
{
  if (isInit)
    return;
  selfID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f) - 5);

  // Initialize pid controllers
  pidInit(&pid_relX, desired_relX, PID_RELXY_KP, PID_RELXY_KI, PID_RELXY_KD, RELATIVE_CONTROL_DT, RELATIVE_CONTROL_RATE, 0.0f, false);
  pidInit(&pid_relY, desired_relY, PID_RELXY_KP, PID_RELXY_KI, PID_RELXY_KD, RELATIVE_CONTROL_DT, RELATIVE_CONTROL_RATE, 0.0f, false);
  pidInit(&pid_relZ, desired_relZ, PID_RELZ_KP, PID_RELZ_KI, PID_RELZ_KD, RELATIVE_CONTROL_DT, RELATIVE_CONTROL_RATE, 0.0f, false);
  
  pidSetIntegralLimit(&pid_relX, PID_INTEGRAL_LIM);
  pidSetIntegralLimit(&pid_relY, PID_INTEGRAL_LIM);
  pidSetIntegralLimit(&pid_relZ, PID_INTEGRAL_LIM);

  pid_relX.outputLimit = PID_RELXY_OUTPUT_LIM;
  pid_relY.outputLimit = PID_RELXY_OUTPUT_LIM;
  pid_relZ.outputLimit = PID_RELZ_OUTPUT_LIM;

  xTaskCreate(relativeControlTask, RELATIVE_CTRL_TASK_NAME, RELATIVE_CTRL_TASK_STACKSIZE, NULL, RELATIVE_CTRL_TASK_PRI,NULL );
  height = 1.5f;
  isInit = true;
}

LOG_GROUP_START(relativeControl)
LOG_ADD(LOG_FLOAT, vx_cmd, &pid_vx)
LOG_ADD(LOG_FLOAT, vy_cmd, &pid_vy)
LOG_ADD(LOG_FLOAT, vz_cmd, &pid_vz)
LOG_GROUP_STOP(relativeControl)

PARAM_GROUP_START(relative_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_GROUP_STOP(relative_ctrl)
