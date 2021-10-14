// Guido de Croon, TU Delft
// H-file of the optical flow based attitude estimator


#include "stabilizer_types.h"

#define CRAZY_FLIE 0
#define FLAPPER 1
#define DRONE CRAZY_FLIE

//  Ultimately, these could all be parameters to be set from the ground station:
#define CONSTANT_ALT_FILTER 1
#define OF_DRAG 1
// Only for constant alt for now!
#define OF_TWO_DIM 0
// Only for changing alt
#define OF_THRUST_BIAS 0
// Whether to use gyros:
#define OF_USE_GYROS 1

#if CONSTANT_ALT_FILTER == 1
  #if OF_TWO_DIM == 0
    #define OF_V_IND 0
    #define OF_ANGLE_IND 1
    #define OF_Z_IND 2
    // #define OF_ANGLE_DOT_IND 3

    #if OF_USE_GYROS == 1
      // ventral flow and gyros
      #define N_MEAS_OF_KF 2
    #else
      // gyros not used at all
      #define N_MEAS_OF_KF 1
    #endif

    #define OF_THETA_IND -1
    #define OF_VX_IND -1

    #if OF_THRUST_BIAS == 0
      #define N_STATES_OF_KF 3
      #define OF_THRUST_BIAS_IND -1
    #else
      // does this work with thrust bias?
      #define N_STATES_OF_KF 5
      #define OF_THRUST_BIAS_IND 4
    #endif
  #endif
#endif


// TODO: make these parameters in the estimation scheme:
#define OF_TB_Q 0.02
#define OF_TB_P 0.5

// For now no divergence:
#define OF_LAT_FLOW_IND 0
#define OF_RATE_IND 1
#if OF_USE_GYROS == 1
  #define OF_LAT_FLOW_X_IND 2
#else
  #define OF_LAT_FLOW_X_IND 1
#endif


void init_OF_att(void);
void estimator_OF_att(state_t *state, const uint32_t tick);
void reset_OF_att(void);