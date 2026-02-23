/*
 * RL Controller App for Flapper Platform
 *
 * This app deploys a trained RL neural network policy on the Flapper drone.
 * - STATE_IDLE (0):       Motors off / landing
 * - STATE_HOVERING (1):   PID hover at target altitude
 * - STATE_RL_CONTROL (2): Neural-network direct motor control
 *
 * Observation vector (20 elements, matching Python flapper_env):
 *   [0-2]   position in gate frame (x, y, z)
 *   [3-5]   velocity in gate frame (vx, vy, vz)
 *   [6-8]   attitude (phi, theta, psi_relative)
 *   [9-11]  angular rates (p, q, r)  — body frame, rad/s
 *   [12-15] previous NN actions (in [-1, 1])
 *   [16-19] next-gate relative to current gate (x, y, z, yaw)
 *
 * Action vector (4 elements in [-1, 1]):
 *   [0]=left flap, [1]=right flap, [2]=dihedral/pitch, [3]=yaw
 *   Converted to PWM via: pwm = (action+1)*0.5 * PWM_MAX
 *
 * Motor mapping (Flapper PCB revD):
 *   m1 = pitch/dihedral servo   (action[2])
 *   m2 = left flapping motor    (action[0])
 *   m3 = yaw servo              (action[3])
 *   m4 = right flapping motor   (action[1])
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "RLCONTROLLER"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "param_logic.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"
#include "neural_net.h"

// ============================================================================
// Constants
// ============================================================================
#define APP_FREQUENCY  100.0f   // Hz — matches RL training dt=0.01s
#define LAND_VZ_MPS    0.4f     // Descent velocity (m/s)
#define CUT_Z_M        0.14f    // Cut controllers below this altitude (m)

#define NUM_GATES      8
#define GATE_R         1.5f     // Figure-8 radius (m)

// Per-motor PWM limits (from training config)
#define PWM_MIN_M1     0       // dihedral/pitch servo
#define PWM_MAX_M1     65535
#define PWM_MIN_M2     0       // left flapping motor
#define PWM_MAX_M2     60000
#define PWM_MIN_M3     9830    // yaw servo
#define PWM_MAX_M3     65535
#define PWM_MIN_M4     0       // right flapping motor
#define PWM_MAX_M4     60000

#ifndef M_PI_F
#define M_PI_F  3.14159265358979323846f
#endif

#define DEG2RAD (M_PI_F / 180.0f)

// ============================================================================
// State machine
// ============================================================================
typedef enum {
  STATE_IDLE       = 0,
  STATE_HOVERING   = 1,
  STATE_RL_CONTROL = 2,
} ControlState;

// ============================================================================
// Runtime-configurable parameters
// ============================================================================
static float   targetAltitudeM = 1.3f;
static uint8_t targetState     = STATE_IDLE;

// Gate origin in world frame (figure-8 centred at this point)
static float gateOriginX  = 0.0f;
static float gateOriginY  = 0.0f;
static float gateAltitude = 1.5f;   // Gate flight altitude in ENU (m) — matches training z=-1.5

// ============================================================================
// State variables
// ============================================================================
static ControlState currentState = STATE_IDLE;
static float hoverAltitudeM = 0.0f;
static float hoverXM  = 0.0f;
static float hoverYM  = 0.0f;
static TickType_t lastWakeTime;
static bool isLanding      = false;
static bool landingFinished = false;

// ============================================================================
// Gate configuration (figure-8 track from Python training)
// ============================================================================
// Base gate positions relative to origin (x, y in world frame)
static const float baseGateX[NUM_GATES] = {
   GATE_R,  0.0f, -GATE_R,  0.0f,
   GATE_R,  0.0f, -GATE_R,  0.0f
};
static const float baseGateY[NUM_GATES] = {
  -GATE_R,  0.0f,  GATE_R,  3.0f,   // 3.0f = 2*GATE_R
   GATE_R,  0.0f, -GATE_R, -3.0f    // -3.0f = -2*GATE_R
};
static const float baseGateYaw[NUM_GATES] = {
   1.57079632679f,  3.14159265359f,   1.57079632679f,  0.0f,    // pi/2, pi, pi/2, 0
  -1.57079632679f, -3.14159265359f,  -1.57079632679f,  0.0f     // -pi/2, -pi, -pi/2, 0
};

// Runtime gate arrays (filled by initGates with origin offset)
static float gateX[NUM_GATES];
static float gateY[NUM_GATES];
static float gateZ[NUM_GATES];       // sim convention: negative = up
static float gateYaw[NUM_GATES];
static float gateCos[NUM_GATES];
static float gateSin[NUM_GATES];

// Precomputed relative gates: gate[i] in gate[i-1]'s frame
static float gateRelX[NUM_GATES];
static float gateRelY[NUM_GATES];
static float gateRelZ[NUM_GATES];
static float gateRelYaw[NUM_GATES];

// ============================================================================
// RL state
// ============================================================================
static int   currentTargetGate = 0;
static float lastActions[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
static float prevSimX = 0.0f;        // Previous position for gate-pass detection
static float prevSimY = 0.0f;

// Observation and action buffers
static float observation[INPUT_DIM];
static float rlActions[OUTPUT_DIM];

// ============================================================================
// Log variable IDs (state estimation + gyro)
// ============================================================================
static logVarId_t idX     = (logVarId_t)0xFFFF;
static logVarId_t idY     = (logVarId_t)0xFFFF;
static logVarId_t idZ     = (logVarId_t)0xFFFF;
static logVarId_t idVx    = (logVarId_t)0xFFFF;
static logVarId_t idVy    = (logVarId_t)0xFFFF;
static logVarId_t idVz    = (logVarId_t)0xFFFF;
static logVarId_t idRoll  = (logVarId_t)0xFFFF;
static logVarId_t idPitch = (logVarId_t)0xFFFF;
static logVarId_t idYawSe = (logVarId_t)0xFFFF;   // stateEstimate yaw
static logVarId_t idGyroX = (logVarId_t)0xFFFF;
static logVarId_t idGyroY = (logVarId_t)0xFFFF;
static logVarId_t idGyroZ = (logVarId_t)0xFFFF;

// Param variable IDs for motor override
static paramVarId_t idMotorEnable;
static paramVarId_t idMotorM1, idMotorM2, idMotorM3, idMotorM4;
static paramVarId_t idTumbleCheck;

// ============================================================================
// Logging variables (visible in cfclient)
// ============================================================================
static float   logNnOut0, logNnOut1, logNnOut2, logNnOut3;
static uint16_t logPwmM1, logPwmM2, logPwmM3, logPwmM4;
static uint8_t  logTargetGate = 0;

// ============================================================================
// Helper: wrap angle to [-pi, pi]
// ============================================================================
static float wrapAngle(float a) {
  while (a >  M_PI_F) a -= 2.0f * M_PI_F;
  while (a < -M_PI_F) a += 2.0f * M_PI_F;
  return a;
}

// ============================================================================
// Safely initialize a log variable ID
// ============================================================================
static void ensureLogId(logVarId_t *id, const char *group, const char *name) {
  if (!logVarIdIsValid(*id)) {
    *id = logGetVarId(group, name);
  }
}

// ============================================================================
// Initialize gate positions (called when entering RL mode)
// ============================================================================
static void initGates(void) {
  for (int i = 0; i < NUM_GATES; i++) {
    gateX[i]   = baseGateX[i]   + gateOriginX;
    gateY[i]   = baseGateY[i]   + gateOriginY;
    gateZ[i]   = -gateAltitude;  // ENU altitude → sim convention (z neg = up)
    gateYaw[i] = baseGateYaw[i];
    gateCos[i] = cosf(gateYaw[i]);
    gateSin[i] = sinf(gateYaw[i]);
  }

  // Precompute relative gates: position of gate[i] in gate[i-1]'s frame
  // (matches Python _calculate_relative_gates)
  for (int i = 0; i < NUM_GATES; i++) {
    int prev = (i == 0) ? (NUM_GATES - 1) : (i - 1);
    float dx = gateX[i] - gateX[prev];
    float dy = gateY[i] - gateY[prev];
    float c  = cosf(gateYaw[prev]);
    float s  = sinf(gateYaw[prev]);

    gateRelX[i]   =  dx * c + dy * s;
    gateRelY[i]   = -dx * s + dy * c;
    gateRelZ[i]   =  gateZ[i] - gateZ[prev];
    gateRelYaw[i] =  wrapAngle(gateYaw[i] - gateYaw[prev]);
  }

  DEBUG_PRINT("Gates initialized: origin=(%.2f,%.2f) alt=%.2fm\n",
              (double)gateOriginX, (double)gateOriginY, (double)gateAltitude);
}

// ============================================================================
// Compute the 20-element observation vector
// (matches Python flapper_env.update_states_gate)
// ============================================================================
static void computeObservation(void) {
  // ---- Read firmware state ------------------------------------------------
  float fw_x     = logGetFloat(idX);
  float fw_y     = logGetFloat(idY);
  float fw_z     = logGetFloat(idZ);
  float fw_vx    = logGetFloat(idVx);
  float fw_vy    = logGetFloat(idVy);
  float fw_vz    = logGetFloat(idVz);
  float fw_roll  = logGetFloat(idRoll);   // degrees
  float fw_pitch = logGetFloat(idPitch);  // degrees
  float fw_yaw   = logGetFloat(idYawSe);  // degrees
  float fw_gx    = logGetFloat(idGyroX);  // deg/s
  float fw_gy    = logGetFloat(idGyroY);  // deg/s
  float fw_gz    = logGetFloat(idGyroZ);  // deg/s

  // ---- Convert to simulation convention -----------------------------------
  // Sim uses z-negative-up (NED-like altitude).  Firmware uses z-positive-up.
  // Horizontal axes (x, y) share the same convention.
  float sim_x  =  fw_x;
  float sim_y  =  -fw_y;
  float sim_z  = -fw_z;                   // flip z
  float sim_vx =  fw_vx;
  float sim_vy =  -fw_vy;
  float sim_vz = -fw_vz;                  // flip vz
  float sim_phi   = fw_roll  * DEG2RAD;
  float sim_theta = fw_pitch * DEG2RAD;
  float sim_psi   = -fw_yaw   * DEG2RAD;
  float sim_p     = fw_gx * DEG2RAD;
  float sim_q     = -fw_gz * DEG2RAD;     // this is weird, but correct?
  float sim_r     = -fw_gy * DEG2RAD;

  // ---- Gate-frame transform -----------------------------------------------
  int gi   = currentTargetGate % NUM_GATES;
  float gc = gateCos[gi];
  float gs = gateSin[gi];
  float dx = sim_x - gateX[gi];
  float dy = sim_y - gateY[gi];

  // Position in gate frame
  observation[0] =  dx * gc + dy * gs;
  observation[1] = -dx * gs + dy * gc;
  observation[2] =  sim_z - gateZ[gi];

  // Velocity in gate frame
  observation[3] =  sim_vx * gc + sim_vy * gs;
  observation[4] = -sim_vx * gs + sim_vy * gc;
  observation[5] =  sim_vz;

  // Attitude (yaw relative to gate yaw)
  observation[6] = sim_phi;
  observation[7] = sim_theta;
  observation[8] = wrapAngle(sim_psi - gateYaw[gi]);

  // Angular rates (body frame)
  observation[9]  = sim_p;
  observation[10] = sim_q;
  observation[11] = sim_r;

  // Previous actions (NN outputs from last step)
  observation[12] = lastActions[0];
  observation[13] = lastActions[1];
  observation[14] = lastActions[2];
  observation[15] = lastActions[3];

  // Next gate relative to current target gate
  int next = (currentTargetGate + 1) % NUM_GATES;
  observation[16] = gateRelX[next];
  observation[17] = gateRelY[next];
  observation[18] = gateRelZ[next];
  observation[19] = gateRelYaw[next];

  // Store position for gate-passing detection (before next step)
  prevSimX = sim_x;
  prevSimY = sim_y;
}

// ============================================================================
// Gate-passing detection (plane-crossing check, matches Python step_wait)
// ============================================================================
static void checkGatePassing(void) {
  // Current sim-frame position
  float sim_x = logGetFloat(idX);        // same as sim_x (no flip needed for x)
  float sim_y = logGetFloat(idY);

  int gi   = currentTargetGate % NUM_GATES;
  float nx = cosf(gateYaw[gi]);          // gate normal direction
  float ny = sinf(gateYaw[gi]);

  float old_proj = (prevSimX - gateX[gi]) * nx + (prevSimY - gateY[gi]) * ny;
  float new_proj = (sim_x   - gateX[gi]) * nx + (sim_y   - gateY[gi]) * ny;

  if (old_proj < 0.0f && new_proj >= 0.0f) {
    // Crossed the gate plane — advance target
    int nextGate = (currentTargetGate + 1) % NUM_GATES;
    DEBUG_PRINT("Gate %d passed → next %d\n", gi, nextGate);
    currentTargetGate = nextGate;
    logTargetGate = (uint8_t)nextGate;
  }
}

// ============================================================================
// Convert NN actions [-1,1] to motor PWM and apply via motorPowerSet override
// ============================================================================
static void applyActions(const float* act) {
  // Store for next observation
  lastActions[0] = act[0];
  lastActions[1] = act[1];
  lastActions[2] = act[2];
  lastActions[3] = act[3];

  // Scale [-1,1] → [0,1]
  // Action mapping: [0]=left flap, [1]=right flap, [2]=dihedral, [3]=yaw
  float s0 = (act[0] + 1.0f) * 0.5f;
  float s1 = (act[1] + 1.0f) * 0.5f;
  float s2 = (act[2] + 1.0f) * 0.5f;
  float s3 = (act[3] + 1.0f) * 0.5f;

  // Clamp to [0,1] for safety (NN clip should already ensure this)
  if (s0 < 0.0f) { s0 = 0.0f; } else if (s0 > 1.0f) { s0 = 1.0f; }
  if (s1 < 0.0f) { s1 = 0.0f; } else if (s1 > 1.0f) { s1 = 1.0f; }
  if (s2 < 0.0f) { s2 = 0.0f; } else if (s2 > 1.0f) { s2 = 1.0f; }
  if (s3 < 0.0f) { s3 = 0.0f; } else if (s3 > 1.0f) { s3 = 1.0f; }

  // Scale [0,1] → [0, pwm_max_mi], then clip to [pwm_min_mi, pwm_max_mi]
  // Motor mapping (Flapper revD): m1=pitch, m2=left, m3=yaw, m4=right
  int32_t raw_m1 = (int32_t)(s2 * PWM_MAX_M1);
  int32_t raw_m2 = (int32_t)(s0 * PWM_MAX_M2);
  int32_t raw_m3 = (int32_t)(s3 * PWM_MAX_M3);
  int32_t raw_m4 = (int32_t)(s1 * PWM_MAX_M4);

  // Clip to per-motor [min, max] (matches _compute_control_states clamp)
  if (raw_m1 < PWM_MIN_M1) { raw_m1 = PWM_MIN_M1; } else if (raw_m1 > PWM_MAX_M1) { raw_m1 = PWM_MAX_M1; }
  if (raw_m2 < PWM_MIN_M2) { raw_m2 = PWM_MIN_M2; } else if (raw_m2 > PWM_MAX_M2) { raw_m2 = PWM_MAX_M2; }
  if (raw_m3 < PWM_MIN_M3) { raw_m3 = PWM_MIN_M3; } else if (raw_m3 > PWM_MAX_M3) { raw_m3 = PWM_MAX_M3; }
  if (raw_m4 < PWM_MIN_M4) { raw_m4 = PWM_MIN_M4; } else if (raw_m4 > PWM_MAX_M4) { raw_m4 = PWM_MAX_M4; }

  uint16_t pwm_m1 = (uint16_t)raw_m1;
  uint16_t pwm_m2 = (uint16_t)raw_m2;
  uint16_t pwm_m3 = (uint16_t)raw_m3;
  uint16_t pwm_m4 = (uint16_t)raw_m4;

  // Log for monitoring
  logPwmM1 = pwm_m1;
  logPwmM2 = pwm_m2;
  logPwmM3 = pwm_m3;
  logPwmM4 = pwm_m4;

  // Write to motorPowerSet params (overrides stabilizer output)
  paramSetInt(idMotorM1, pwm_m1);
  paramSetInt(idMotorM2, pwm_m2);
  paramSetInt(idMotorM3, pwm_m3);
  paramSetInt(idMotorM4, pwm_m4);
}

// ============================================================================
// Send a keep-alive setpoint so the supervisor doesn't trigger a timeout
// ============================================================================
static void sendKeepAliveSetpoint(void) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));
  // Disable all control modes — the motors are driven by motorPowerSet
  sp.mode.x   = modeDisable;
  sp.mode.y   = modeDisable;
  sp.mode.z   = modeDisable;
  sp.mode.yaw = modeDisable;
  sp.thrust   = 0;
  commanderSetSetpoint(&sp, 3);
}

// ============================================================================
// Enable / disable motor power override
// ============================================================================
static void enableMotorOverride(bool enable) {
  if (enable) {
    // Set all PWMs to 0 before enabling override (safety)
    paramSetInt(idMotorM1, 0);
    paramSetInt(idMotorM2, 0);
    paramSetInt(idMotorM3, 0);
    paramSetInt(idMotorM4, 0);
    paramSetInt(idMotorEnable, 1);
    // Disable tumble check — RL may produce aggressive manoeuvres
    paramSetInt(idTumbleCheck, 0);
    DEBUG_PRINT("Motor override ON, tumble check OFF\n");
  } else {
    paramSetInt(idMotorEnable, 0);
    paramSetInt(idTumbleCheck, 1);
    DEBUG_PRINT("Motor override OFF, tumble check ON\n");
  }
}

// ============================================================================
// RL controller — one step (called at 100 Hz)
// ============================================================================
static void rlControllerCompute(void) {
  // Detect gate crossing (uses prev position stored last step)
  checkGatePassing();

  // Build the 20-element observation (gate-frame state + actions + next gate)
  computeObservation();

  // Neural-network forward pass
  forward(observation, rlActions);

  // Log NN outputs
  logNnOut0 = rlActions[0];
  logNnOut1 = rlActions[1];
  logNnOut2 = rlActions[2];
  logNnOut3 = rlActions[3];

  // Convert actions → PWM and write to motors
  applyActions(rlActions);

  // Prevent supervisor watchdog from killing the flight
  sendKeepAliveSetpoint();
}

// ============================================================================
// Helper: send hover command
// ============================================================================
static void sendHoverCommand(float altitudeM, float xM, float yM) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));

  sp.mode.z = modeAbs;
  sp.position.z = altitudeM;

  sp.mode.x = modeAbs;
  sp.mode.y = modeAbs;
  sp.position.x = xM;
  sp.position.y = yM;

  sp.mode.yaw = modeVelocity;
  sp.attitudeRate.yaw = 0.0f;

  commanderSetSetpoint(&sp, 3);
}

// ============================================================================
// Helper: send landing velocity command
// ============================================================================
static void sendLandingCommand(void) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));
  sp.mode.x = modeVelocity;
  sp.velocity.x = 0;
  sp.mode.y = modeVelocity;
  sp.velocity.y = 0;
  sp.mode.z = modeVelocity;
  sp.velocity.z = -LAND_VZ_MPS;
  sp.mode.yaw = modeVelocity;
  sp.attitudeRate.yaw = 0;
  sp.velocity_body = true;

  commanderSetSetpoint(&sp, 3);
}

// ============================================================================
// Main app loop
// ============================================================================
void appMain(void) {
  DEBUG_PRINT("RL Controller App started\n");

  // Wait for all log variable IDs to become available
  bool allFound = false;
  while (!allFound) {
    ensureLogId(&idX,     "stateEstimate", "x");
    ensureLogId(&idY,     "stateEstimate", "y");
    ensureLogId(&idZ,     "stateEstimate", "z");
    ensureLogId(&idVx,    "stateEstimate", "vx");
    ensureLogId(&idVy,    "stateEstimate", "vy");
    ensureLogId(&idVz,    "stateEstimate", "vz");
    ensureLogId(&idRoll,  "stateEstimate", "roll");
    ensureLogId(&idPitch, "stateEstimate", "pitch");
    ensureLogId(&idYawSe, "stateEstimate", "yaw");
    ensureLogId(&idGyroX, "gyro", "x");
    ensureLogId(&idGyroY, "gyro", "y");
    ensureLogId(&idGyroZ, "gyro", "z");

    allFound = logVarIdIsValid(idX)  && logVarIdIsValid(idY)  && logVarIdIsValid(idZ)
            && logVarIdIsValid(idVx) && logVarIdIsValid(idVy) && logVarIdIsValid(idVz)
            && logVarIdIsValid(idRoll) && logVarIdIsValid(idPitch) && logVarIdIsValid(idYawSe)
            && logVarIdIsValid(idGyroX) && logVarIdIsValid(idGyroY) && logVarIdIsValid(idGyroZ);

    if (!allFound) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  // Get motor-override param IDs
  idMotorEnable = paramGetVarId("motorPowerSet", "enable");
  idMotorM1     = paramGetVarId("motorPowerSet", "m1");
  idMotorM2     = paramGetVarId("motorPowerSet", "m2");
  idMotorM3     = paramGetVarId("motorPowerSet", "m3");
  idMotorM4     = paramGetVarId("motorPowerSet", "m4");
  idTumbleCheck = paramGetVarId("supervisor",    "tmblChckEn");

  DEBUG_PRINT("Log + param IDs ready. Target alt: %.2f m\n", (double)targetAltitudeM);

  // Initialize timing
  lastWakeTime = xTaskGetTickCount();

  // ---- Main control loop at 100 Hz ----------------------------------------
  while (1) {
    float currentX = logGetFloat(idX);
    float currentY = logGetFloat(idY);
    float currentZ = logGetFloat(idZ);

    // ---- Handle state transitions ------------------------------------------
    if ((uint8_t)targetState != (uint8_t)currentState) {
      ControlState newState = (ControlState)targetState;

      // Leaving RL mode → disable motor override
      if (currentState == STATE_RL_CONTROL && newState != STATE_RL_CONTROL) {
        enableMotorOverride(false);
      }

      currentState = newState;
      DEBUG_PRINT("State → %u\n", (unsigned)currentState);

      if (currentState == STATE_HOVERING) {
        hoverAltitudeM = targetAltitudeM;
        hoverXM = currentX;
        hoverYM = currentY;
        DEBUG_PRINT("Hover at (%.2f, %.2f, %.2f)\n",
                    (double)hoverXM, (double)hoverYM, (double)hoverAltitudeM);

      } else if (currentState == STATE_RL_CONTROL) {
        // Gates are in world frame — origin/altitude set via params
        // (default: figure-8 centred at (0,0) at 1.5m)
        initGates();

        // Reset RL state
        currentTargetGate = 0;
        logTargetGate     = 0;
        memset(lastActions, 0, sizeof(lastActions));
        prevSimX = currentX;
        prevSimY = currentY;

        // Save hover-return position
        hoverAltitudeM = currentZ;
        hoverXM = currentX;
        hoverYM = currentY;

        enableMotorOverride(true);
        DEBUG_PRINT("RL active at (%.2f, %.2f, %.2f)\n",
                    (double)currentX, (double)currentY, (double)currentZ);

      } else if (currentState == STATE_IDLE) {
        isLanding = false;
        landingFinished = false;
        DEBUG_PRINT("Idle — landing\n");
      }
    }

    // ---- State machine -----------------------------------------------------
    switch (currentState) {
      case STATE_IDLE:
        if (!landingFinished) {
          if (!isLanding) {
            DEBUG_PRINT("Landing from z=%.2f\n", (double)currentZ);
            isLanding = true;
          }
          sendLandingCommand();
          if (currentZ >= 0.0f && currentZ <= CUT_Z_M) {
            DEBUG_PRINT("Landed z=%.4f, cutting power\n", (double)currentZ);
            setpoint_t cut;
            memset(&cut, 0, sizeof(cut));
            cut.mode.x   = modeDisable;
            cut.mode.y   = modeDisable;
            cut.mode.z   = modeDisable;
            cut.mode.yaw = modeDisable;
            cut.thrust   = 0;
            commanderSetSetpoint(&cut, 3);
            landingFinished = true;
          }
        }
        break;

      case STATE_HOVERING:
        sendHoverCommand(hoverAltitudeM, hoverXM, hoverYM);
        break;

      case STATE_RL_CONTROL:
        rlControllerCompute();
        break;

      default:
        currentState = STATE_IDLE;
        break;
    }

    vTaskDelayUntil(&lastWakeTime, F2T(APP_FREQUENCY));
  }
}

// ============================================================================
// Parameters (cfclient / CRTP)
// ============================================================================
PARAM_GROUP_START(rlapp)
  PARAM_ADD(PARAM_FLOAT  | PARAM_PERSISTENT, targetAlt,   &targetAltitudeM)
  PARAM_ADD(PARAM_UINT8,                     targetState,  &targetState)
  PARAM_ADD(PARAM_FLOAT,                     gateOriginX,  &gateOriginX)
  PARAM_ADD(PARAM_FLOAT,                     gateOriginY,  &gateOriginY)
  PARAM_ADD(PARAM_FLOAT,                     gateAlt,      &gateAltitude)
PARAM_GROUP_STOP(rlapp)

// ============================================================================
// Log variables (cfclient / CRTP)
// ============================================================================
LOG_GROUP_START(rlapp)
  LOG_ADD(LOG_UINT8,  state,      &currentState)
  LOG_ADD(LOG_UINT8,  tgtGate,    &logTargetGate)
  LOG_ADD(LOG_FLOAT,  hoverX,     &hoverXM)
  LOG_ADD(LOG_FLOAT,  hoverY,     &hoverYM)
  LOG_ADD(LOG_FLOAT,  hoverAlt,   &hoverAltitudeM)
  LOG_ADD(LOG_FLOAT,  nnOut0,     &logNnOut0)
  LOG_ADD(LOG_FLOAT,  nnOut1,     &logNnOut1)
  LOG_ADD(LOG_FLOAT,  nnOut2,     &logNnOut2)
  LOG_ADD(LOG_FLOAT,  nnOut3,     &logNnOut3)
  LOG_ADD(LOG_UINT16, pwmM1,      &logPwmM1)
  LOG_ADD(LOG_UINT16, pwmM2,      &logPwmM2)
  LOG_ADD(LOG_UINT16, pwmM3,      &logPwmM3)
  LOG_ADD(LOG_UINT16, pwmM4,      &logPwmM4)
LOG_GROUP_STOP(rlapp)
