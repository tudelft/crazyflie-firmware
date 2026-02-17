/*
 * RL Controller App for Flapper Platform
 *
 * This app provides a simple interface for testing RL controllers.
 * - aux0 switch: triggers takeoff to target altitude and hover
 * - aux1 switch: toggles between hover control and RL controller (stand-in)
 *
 * The app listens to these aux channels and manages the flight state accordingly.
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
#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"

// Frequency definitions
#define APP_FREQUENCY 100.0f  // Hz
#define LAND_VZ_MPS 0.4f      // Descent velocity (m/s)
#define CUT_Z_M 0.14f         // Cut controllers below this altitude

// ============================================================================
// State machine
// ============================================================================
typedef enum {
  STATE_IDLE = 0,           // Waiting for takeoff signal
  STATE_HOVERING = 1,       // Hovering at target altitude
  STATE_RL_CONTROL = 2,     // Using RL controller (stub implementation)
} ControlState;

// ============================================================================
// Runtime-configurable parameters
// ============================================================================
static float targetAltitudeM = 1.3f;  // Takeoff altitude in meters
static uint8_t targetState = STATE_IDLE;  // Target state (0=IDLE, 1=HOVERING, 2=RL_CONTROL)

// ============================================================================
// State variables
// ============================================================================
static ControlState currentState = STATE_IDLE;
static float hoverAltitudeM = 0.0f;   // Altitude at which we start hovering
static float hoverXM = 0.0f;          // X position at which we start hovering
static float hoverYM = 0.0f;          // Y position at which we start hovering
static TickType_t lastWakeTime;       // For precise 100 Hz timing
static bool isLanding = false;         // Flag to track if landing sequence has been initiated
static bool landingFinished = false;   // Flag to track if landing is complete

// ============================================================================
// Log variable IDs
// ============================================================================
static logVarId_t idX = (logVarId_t)0xFFFF;           // Current X position
static logVarId_t idY = (logVarId_t)0xFFFF;           // Current Y position
static logVarId_t idZ = (logVarId_t)0xFFFF;           // Current Z position

// ============================================================================
// Helper function to safely get log variable IDs
// ============================================================================
static void ensureLogId(logVarId_t *id, const char *group, const char *name) {
  if (!logVarIdIsValid(*id)) {
    *id = logGetVarId(group, name);
  }
}

// ============================================================================
// Helper function: send hover command
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
// Helper function: send landing velocity command
// ============================================================================
static void sendLandingCommand(void) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));
  sp.mode.x = modeVelocity;
  sp.velocity.x = 0;
  sp.mode.y = modeVelocity;
  sp.velocity.y = 0;
  sp.mode.z = modeVelocity;
  sp.velocity.z = -LAND_VZ_MPS;  // Descend at constant velocity
  sp.mode.yaw = modeVelocity;
  sp.attitudeRate.yaw = 0;
  sp.velocity_body = true;

  commanderSetSetpoint(&sp, 3);
}

// ============================================================================
// RL Controller (stand-in implementation)
// This is a placeholder that will be replaced with the actual RL implementation
// ============================================================================
static void rlControllerCompute(void) {
  setpoint_t sp;
  memset(&sp, 0, sizeof(sp));

  // Stand-in: just maintain hover at saved altitude
  
  sp.mode.z = modeAbs;
  sp.position.z = hoverAltitudeM;
  
  sp.mode.x = modeVelocity;
  sp.mode.y = modeVelocity;
  sp.velocity_body = true;
  sp.velocity.x = 0.2f;
  sp.velocity.y = 0.0f;
  
  sp.mode.yaw = modeVelocity;
  sp.attitudeRate.yaw = 0.0f;

  commanderSetSetpoint(&sp, 3);
}



// ============================================================================
// Main app loop
// ============================================================================
void appMain(void) {
  DEBUG_PRINT("RL Controller App started\n");

  // Wait for log variable IDs to be available
  while (!logVarIdIsValid(idX) || !logVarIdIsValid(idY) || !logVarIdIsValid(idZ)) {
    ensureLogId(&idX, "stateEstimate", "x");
    ensureLogId(&idY, "stateEstimate", "y");
    ensureLogId(&idZ, "stateEstimate", "z");
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  DEBUG_PRINT("Log variables initialized. Waiting for state parameter...\n");
  DEBUG_PRINT("Target altitude: %.2f m\n", (double)targetAltitudeM);

  // Initialize timing for 100 Hz execution
  lastWakeTime = xTaskGetTickCount();

  // Main control loop at 100 Hz
  while (1) {
    float currentX = logGetFloat(idX);
    float currentY = logGetFloat(idY);
    float currentZ = logGetFloat(idZ);

    // Handle state transitions based on targetState parameter
    if ((uint8_t)targetState != currentState) {
      currentState = (ControlState)targetState;
      DEBUG_PRINT("State changed to %u\n", (unsigned int)currentState);
      
      // Initialize state-specific parameters
      if (currentState == STATE_HOVERING) {
        hoverAltitudeM = targetAltitudeM;
        hoverXM = currentX;
        hoverYM = currentY;
        DEBUG_PRINT("Hovering at (%.2f, %.2f, %.2f)\n", (double)hoverXM, (double)hoverYM, (double)hoverAltitudeM);
      } else if (currentState == STATE_RL_CONTROL) {
        hoverAltitudeM = currentZ;  // Save current altitude for return
        hoverXM = currentX;         // Save current X position for return
        hoverYM = currentY;         // Save current Y position for return
        DEBUG_PRINT("RL controller activated. Hover position saved at (%.2f, %.2f, %.2f)\n", (double)hoverXM, (double)hoverYM, (double)hoverAltitudeM);
      } else if (currentState == STATE_IDLE) {
        isLanding = false;
        landingFinished = false;
        DEBUG_PRINT("Idle state - initiating landing sequence\n");
      }
    }

    // State machine
    switch (currentState) {
      case STATE_IDLE:
        // Idle state - perform smooth landing if flying, then cut power
        if (!landingFinished) {
          if (!isLanding) {
            DEBUG_PRINT("Initiating landing with velocity control (z=%.2f)\n", (double)currentZ);
            isLanding = true;
          }
          sendLandingCommand();
          // Check if we've reached landing altitude
          if (currentZ >= 0.0f && currentZ <= CUT_Z_M) {
            DEBUG_PRINT("Landing complete at z=%.4f, cutting power\n", (double)currentZ);
            // Cut all controllers and thrust
            setpoint_t cut;
            memset(&cut, 0, sizeof(cut));
            cut.mode.x = modeDisable;
            cut.mode.y = modeDisable;
            cut.mode.z = modeDisable;
            cut.mode.yaw = modeDisable;
            cut.thrust = 0;
            commanderSetSetpoint(&cut, 3);
            landingFinished = true;
          }
        }
        break;

      case STATE_HOVERING:
        // Hovering at target altitude at (hoverXM, hoverYM) position
        sendHoverCommand(hoverAltitudeM, hoverXM, hoverYM);
        break;

      case STATE_RL_CONTROL:
        // Using RL controller (stand-in)
        rlControllerCompute();
        break;

      default:
        currentState = STATE_IDLE;
        break;
    }

    // Maintain precise 100 Hz execution frequency
    vTaskDelayUntil(&lastWakeTime, F2T(APP_FREQUENCY));
  }
}

// ============================================================================
// Parameter definitions (can be changed from client)
// State values: 0=IDLE, 1=HOVERING, 2=RL_CONTROL
// ============================================================================
PARAM_GROUP_START(rlapp)
  PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, targetAltitude, &targetAltitudeM)
  PARAM_ADD(PARAM_UINT8, targetState, &targetState)
PARAM_GROUP_STOP(rlapp)

// ============================================================================
// Log definitions (for monitoring state)
// State values: 0=IDLE, 1=HOVERING, 2=RL_CONTROL
// ============================================================================
LOG_GROUP_START(rlapp)
  LOG_ADD(LOG_UINT8, state, &currentState)
  LOG_ADD(LOG_FLOAT, hoverX, &hoverXM)
  LOG_ADD(LOG_FLOAT, hoverY, &hoverYM)
  LOG_ADD(LOG_FLOAT, hoverAltitude, &hoverAltitudeM)
LOG_GROUP_STOP(rlapp)
