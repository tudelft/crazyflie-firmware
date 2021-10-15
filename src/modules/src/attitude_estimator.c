// Guido de Croon, TU Delft

#include "attitude_estimator.h"
#include "arm_math.h"
#include "cf_math.h"
#include "math3d.h"

#include "log.h"
#include "param.h"
#include "debug.h"

// struct for state-keeping:
struct InsFlow {

  // vision measurements:
  float optical_flow_x;
  float optical_flow_y;
  float vision_time; // perhaps better to use microseconds (us) instead of float in seconds
  bool new_flow_measurement;

  // RPMs:
  uint16_t RPM[8]; // max an octocopter
  uint8_t RPM_num_act;

  float lp_gyro_pitch;
  float lp_gyro_bias_pitch; // determine the bias before take-off
  float lp_gyro_roll;
  float lp_gyro_bias_roll; // determine the bias before take-off
  float thrust_factor; // determine the additional required scale factor to have unbiased thrust estimates
  float lp_thrust;
  float lp_roll_command;
};
struct InsFlow ins_flow;

// parameters for the filter:

#if CONSTANT_ALT_FILTER
    #if OF_DRAG
        // TODO: put the crazy flie / flapper parameters here:
      float parameters[23] = {0.178636,0.477699,0.096570,0.149979,0.101511,0.000100,0.214033,0.051588,0.156062,0.255246,0.020759,1.884148,0.053983,2.839847,1.069889,0.130275,0.320725,1.077034,0.898386,0.558318,0.073190,0.098345,0.080426};
    #endif
#endif

#define PAR_IX 0
#define PAR_MASS 1
#define PAR_BASE 2
#define PAR_K0 3
#define PAR_K1 4
#define PAR_K2 5
#define PAR_K3 6
#define PAR_R0 7
#define PAR_R1 8
#define PAR_Q0 9
#define PAR_Q1 10
#define PAR_Q2 11
#define PAR_Q3 12
#define PAR_Q4 13
#define PAR_P0 14
#define PAR_P1 15
#define PAR_P2 16
#define PAR_P3 17
#define PAR_P4 18
#define PAR_KD 19
#define PAR_Q_TB 20
#define PAR_P_TB 21

// other parameters
float lp_factor = 0.95;
float lp_factor_strong = 1-1E-3;
uint8_t reset_filter;
uint8_t use_filter;
uint8_t run_filter;
uint32_t counter_of;
float thrust_factor;

// matrices for state, actuation noise, state uncertainty, and measurement noise:

float OF_X[N_STATES_OF_KF] = {0.};
float OF_Q[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
float OF_P[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
float OF_R[N_MEAS_OF_KF][N_MEAS_OF_KF] = {{0.}};
//static __attribute__((aligned(4))) arm_matrix_instance_f32 OF_Xm = { N_STATES_OF_KF, 1, (float *)OF_X};
static __attribute__((aligned(4))) arm_matrix_instance_f32 OF_Qm = { N_STATES_OF_KF, N_STATES_OF_KF, (float *)OF_Q};
static __attribute__((aligned(4))) arm_matrix_instance_f32 OF_Pm = { N_STATES_OF_KF, N_STATES_OF_KF, (float *)OF_P};
static __attribute__((aligned(4))) arm_matrix_instance_f32 OF_Rm = { N_MEAS_OF_KF, N_MEAS_OF_KF, (float *)OF_R};

void init_OF_att() {

  ins_flow.new_flow_measurement = false;
  ins_flow.lp_gyro_pitch = 0.0f;
  ins_flow.lp_gyro_bias_pitch = 0.0f;
  ins_flow.lp_gyro_roll = 0.0f;
  ins_flow.lp_gyro_bias_roll = 0.0f;
  ins_flow.thrust_factor = 1.0f;
  ins_flow.lp_thrust = 0.0f;
  ins_flow.lp_roll_command = 0.0f;

  // Extended Kalman filter:
  // reset the state and P matrix:
  reset_OF_att();

  // R-matrix, measurement noise 
  OF_R[OF_LAT_FLOW_IND][OF_LAT_FLOW_IND] = parameters[PAR_R0];

  // Q-matrix, actuation noise (TODO: make params)
  OF_Q[OF_V_IND][OF_V_IND] = parameters[PAR_Q0];
  OF_Q[OF_ANGLE_IND][OF_ANGLE_IND] = parameters[PAR_Q1];
  OF_Q[OF_Z_IND][OF_Z_IND] = parameters[PAR_Q3];

  // TODO: for a moment and thrust model, fit these factors:
  /*
  RPM_FACTORS[0] = parameters[PAR_K0]*1E-7;
  RPM_FACTORS[1] = parameters[PAR_K1]*1E-7;
  RPM_FACTORS[2] = parameters[PAR_K2]*1E-7;
  RPM_FACTORS[3] = parameters[PAR_K3]*1E-7;
  */
  
  reset_filter = 0;
  use_filter = 0;
  run_filter = 0;

  /* TODO:  how to get time? 
  of_time = get_sys_time_float();
  of_prev_time = get_sys_time_float();
  */
  DEBUG_PRINT("FLOWEST: Attitude Estimator Flow Initialized\n");
}

void reset_OF_att() {

  // (re-)initialize the state:
  for(int i = 0; i < N_STATES_OF_KF; i++) {
      OF_X[i] = 0.0f;
  }
  OF_X[OF_Z_IND] = 1.0; // nonzero z

  // P-matrix:
  for(int i = 0; i < N_STATES_OF_KF; i++) {
      for(int j = 0; j < N_STATES_OF_KF; j++) {
	  OF_P[i][j] = 0.0f;
    }
  }
  if(CONSTANT_ALT_FILTER == 1) {
      OF_P[OF_V_IND][OF_V_IND] = parameters[PAR_P0]; 
      OF_P[OF_ANGLE_IND][OF_ANGLE_IND] = parameters[PAR_P1];
      OF_P[OF_Z_IND][OF_Z_IND] = parameters[PAR_P3];
  }

  // can be used for printing something once in a while
  counter_of = 0;
  DEBUG_PRINT("FLOWEST: Attitude Estimator Flow Reset\n");
}

void estimator_OF_att(float dt)
{
   
  float mass = parameters[PAR_MASS]; // 0.400;
  /*
  float moment = 0.0f; // for now assumed to be 0
  float Ix = parameters[PAR_IX]; // 0.0018244;
  float b = parameters[PAR_BASE];
  */
  float g = 9.81; // TODO: get a more accurate definition from pprz
  float kd = parameters[PAR_KD]; // 0.5
  float drag = 0.0f;


  if(reset_filter==1) {
      reset_OF_att();
      reset_filter = 0;
  }

  if (run_filter==0) {
    return;
  }

  counter_of++;

  // assuming that the typical case is no rotation, we can estimate the (initial) bias of the gyro:
  ins_flow.lp_gyro_bias_roll = lp_factor_strong * ins_flow.lp_gyro_bias_roll + (1-lp_factor_strong) * ins_flow.lp_gyro_roll;
  float gyro_msm = (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll);

  // TODO: is this check still useful?
  //if(dt > 1.0f) {
  //    dt = 0.01f;
  //}

  // propagate the state with Euler integration:
  if(CONSTANT_ALT_FILTER) {
      OF_X[OF_V_IND] += dt * (g * tanf(OF_X[OF_ANGLE_IND]));
      if(OF_DRAG) {
        // quadratic drag acceleration:
        drag = dt * kd * (OF_X[OF_V_IND]*OF_X[OF_V_IND]) / mass;
        // apply it in the right direction:
        if(OF_X[OF_V_IND] > 0) OF_X[OF_V_IND] -= drag;
        else OF_X[OF_V_IND] += drag;
      }

      if(OF_USE_GYROS) {
	    OF_X[OF_ANGLE_IND] += dt * gyro_msm; 
      }
  }

  // ensure that z is not 0 (or lower)
  if(OF_X[OF_Z_IND] < 1e-2f) {
      OF_X[OF_Z_IND] = 1e-2f;
  }

  // prepare the update and correction step:
  // we have to recompute these all the time, as they depend on the state:
  // discrete version of state transition matrix F: (ignoring t^2)
  float F[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
  for(int i = 0; i < N_STATES_OF_KF; i++) {
      F[i][i] = 1.0f;
  }
  if(CONSTANT_ALT_FILTER) {
    F[OF_V_IND][OF_ANGLE_IND] = dt*(g/(cosf(OF_X[OF_ANGLE_IND])*cosf(OF_X[OF_ANGLE_IND])));
  }
  if(OF_DRAG) {
      // In MATLAB: -sign(v)*2*kd*v/m (always minus, whether v is positive or negative):
      F[OF_V_IND][OF_V_IND] -=  dt * 2 * kd * fabsf(OF_X[OF_V_IND]) / mass;
  }

  // G matrix (whatever it may be):
  float G[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
  // TODO: we miss an off-diagonal element here (compare with MATLAB)
  for(int i = 0; i < N_STATES_OF_KF; i++) {
	G[i][i] = dt;
  }

  // Jacobian observation matrix H:
  float H[N_MEAS_OF_KF][N_STATES_OF_KF] = {{0.}};

  if(CONSTANT_ALT_FILTER) {
      // Hx = [-cosf(theta)^2/z, (v*sinf(theta))/ z, (v* cosf(theta)^2)/z^2];
    // lateral flow:
    H[OF_LAT_FLOW_IND][OF_V_IND] = -cosf(OF_X[OF_ANGLE_IND])*cosf(OF_X[OF_ANGLE_IND])/ OF_X[OF_Z_IND];
    H[OF_LAT_FLOW_IND][OF_ANGLE_IND] = OF_X[OF_V_IND]*sinf(2*OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];
    H[OF_LAT_FLOW_IND][OF_Z_IND] = OF_X[OF_V_IND]*cosf(OF_X[OF_ANGLE_IND])*cosf(OF_X[OF_ANGLE_IND])/(OF_X[OF_Z_IND]*OF_X[OF_Z_IND]);
  }
  __attribute__((aligned(4))) arm_matrix_instance_f32 Phim = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) F};
  __attribute__((aligned(4))) arm_matrix_instance_f32 Gammam = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) G};
  __attribute__((aligned(4))) arm_matrix_instance_f32 Jacm = { N_MEAS_OF_KF, N_STATES_OF_KF, (float*) H};

  // Corresponding MATLAB statement:    :O
  // P_k1_k = Phi_k1_k*P*Phi_k1_k' + Gamma_k1_k*Q*Gamma_k1_k';
  float PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  float P_PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  float Phi_P_PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  __attribute__((aligned(4))) arm_matrix_instance_f32 PhiTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) PhiT};
  __attribute__((aligned(4))) arm_matrix_instance_f32 P_PhiTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) P_PhiT};
  __attribute__((aligned(4))) arm_matrix_instance_f32 Phi_P_PhiTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) Phi_P_PhiT};
  mat_trans(&Phim, &PhiTm);
  mat_mult(&OF_Pm, &PhiTm, &P_PhiTm);
  mat_mult(&Phim, &P_PhiTm, &Phi_P_PhiTm);

  float GT[N_STATES_OF_KF][N_STATES_OF_KF];
  float Q_GT[N_STATES_OF_KF][N_STATES_OF_KF];
  float G_Q_GT[N_STATES_OF_KF][N_STATES_OF_KF];
  __attribute__((aligned(4))) arm_matrix_instance_f32 GTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) GT};
  __attribute__((aligned(4))) arm_matrix_instance_f32 Q_GTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) Q_GT};
  __attribute__((aligned(4))) arm_matrix_instance_f32 G_Q_GTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) G_Q_GT};
  mat_trans(&Gammam, &GTm);
  mat_mult(&OF_Qm, &GTm, &Q_GTm);
  mat_mult(&Gammam, &Q_GTm, &G_Q_GTm);

  arm_mat_add_f32(&Phi_P_PhiTm, &G_Q_GTm, &OF_Pm); // in original code, P is used. Is that OF_P?
  
  // correct state when there is a new vision measurement:
  if(ins_flow.new_flow_measurement) {

    // determine Kalman gain:
    // MATLAB statement:
    // S_k = Hx*P_k1_k*Hx' + R;
    float JacT[N_STATES_OF_KF][N_MEAS_OF_KF];
    float P_JacT[N_STATES_OF_KF][N_MEAS_OF_KF];
    float Jac_P_JacT[N_MEAS_OF_KF][N_MEAS_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 JacTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) JacT};
    __attribute__((aligned(4))) arm_matrix_instance_f32 P_JacTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) P_JacT};
    __attribute__((aligned(4))) arm_matrix_instance_f32 Jac_P_JacTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) Jac_P_JacT};
    mat_trans(&Jacm, &JacTm);
    mat_mult(&OF_Pm, &JacTm, &P_JacTm); // TODO: is it intentional that P changes above before this formula is used?
    mat_mult(&Jacm, &P_JacTm, &Jac_P_JacTm);


    float S[N_MEAS_OF_KF][N_MEAS_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 Sm = { N_MEAS_OF_KF, N_MEAS_OF_KF, (float*) S};
    arm_mat_add_f32(&Jac_P_JacTm, &OF_Rm, &Sm);
    
    // MATLAB statement:
    // K_k1 = P_k1_k*Hx' * inv(S_k);
    float K[N_STATES_OF_KF][N_MEAS_OF_KF];
    float INVS[N_MEAS_OF_KF][N_MEAS_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 Km = { N_MEAS_OF_KF, N_MEAS_OF_KF, (float*) K};
    __attribute__((aligned(4))) arm_matrix_instance_f32 INVSm = { N_MEAS_OF_KF, N_MEAS_OF_KF, (float*) INVS};
    // TODO: in the foreseen filter, the matrix has size 1x1... So we can just do 1/s here. 
    mat_inv(&Sm, &INVSm);
    mat_mult(&P_JacTm, &INVSm, &Km);

    // Correct the state:
    // MATLAB:
    // Z_expected = [-v*cosf(theta)*cosf(theta)/z + zd*sinf(2*theta)/(2*z) + thetad;
    //			(-v*sinf(2*theta)/(2*z)) - zd*cosf(theta)*cosf(theta)/z];
    float Z_expected[N_MEAS_OF_KF];

    if(CONSTANT_ALT_FILTER) {
      Z_expected[OF_LAT_FLOW_IND] = -OF_X[OF_V_IND]*cosf(OF_X[OF_ANGLE_IND])*cosf(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]+gyro_msm;
    }

    //  i_k1 = Z - Z_expected;
    float innovation[N_MEAS_OF_KF][1];
    innovation[OF_LAT_FLOW_IND][0] = ins_flow.optical_flow_x - Z_expected[OF_LAT_FLOW_IND];
    __attribute__((aligned(4))) arm_matrix_instance_f32 innovationm = { N_MEAS_OF_KF, 1, (float*) innovation};

    // X_k1_k1 = X_k1_k + K_k1*(i_k1);
    float KI[N_STATES_OF_KF][1];
    __attribute__((aligned(4))) arm_matrix_instance_f32 KIm = { N_STATES_OF_KF, 1, (float*) KI};
    mat_mult(&Km, &innovationm, &KIm);
    for(int i = 0; i < N_STATES_OF_KF; i++) {
	    OF_X[i] += KI[i][0];
    }
    
    // P_k1_k1 = (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)' + K_k1*R*K_k1'; % Joseph form of the covariance update equation
    float K_Jac[N_STATES_OF_KF][N_STATES_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 K_Jacm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) K_Jac};
    mat_mult(&Km, &Jacm, &K_Jacm);

    float eye[N_STATES_OF_KF][N_STATES_OF_KF];
    for(int i = 0; i < N_STATES_OF_KF; i++) {
      eye[i][i] = 1.0f;
    }
    float e_K_Jac[N_STATES_OF_KF][N_STATES_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 eyem = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) eye};
    __attribute__((aligned(4))) arm_matrix_instance_f32 e_K_Jacm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) e_K_Jac};
    arm_mat_sub_f32(&eyem, &K_Jacm, &e_K_Jacm);

    float e_K_JacT[N_STATES_OF_KF][N_STATES_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 e_K_JacTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) e_K_JacT};
    mat_trans(&e_K_Jacm, &e_K_JacTm);
    
    // (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)'
    float P_pre[N_STATES_OF_KF][N_STATES_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 P_prem = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) P_pre};
    mat_mult(&OF_Pm, &e_K_JacTm, &P_prem);
    mat_mult(&e_K_Jacm, &P_prem, &OF_Pm);
    
    // K_k1*R*K_k1'
    // TODO: check all MAKE_MATRIX that they mention the number of ROWS!
    float KT[N_MEAS_OF_KF][N_STATES_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 KTm = { N_MEAS_OF_KF, N_STATES_OF_KF, (float*) KT};
    mat_trans(&Km, &KTm);

    float RKT[N_MEAS_OF_KF][N_STATES_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 R_KTm = { N_MEAS_OF_KF, N_STATES_OF_KF, (float*) RKT};
    mat_mult(&OF_Rm, &KTm, &R_KTm);
    
    float KRKT[N_STATES_OF_KF][N_STATES_OF_KF];
    __attribute__((aligned(4))) arm_matrix_instance_f32 KRKTm = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) KRKT};
    mat_mult(&Km, &R_KTm, &KRKTm);
    // TODO: the first is a const, but it should go well. Perhaps double check the result.
    arm_mat_add_f32(&OF_Pm, &KRKTm, &OF_Pm);

    float trace_P = 0.0f;
    for(int i = 0; i < N_STATES_OF_KF; i++) {
    	trace_P += OF_P[i][i];
    }

    // indicate that the measurement has been used:
    ins_flow.new_flow_measurement = false;
  }

  // update the time:
  // of_prev_time = of_time;
}

void get_quaternion(float *q) {
  float phi = 5.0f / 57.0f; //OF_X[OF_ANGLE_IND];
  float theta = 10.0f / 57.0f;
  float psi = 3.141592654f - 20.0f / 57.0f ;

  struct vec rpy;
  rpy.x = psi;
  rpy.y = theta;  // pitch up positive
  rpy.z = phi;    // roll right positive
  struct quat tempq = rpy2quat(rpy);

  q[0] = tempq.x;
  q[1] = tempq.y;
  q[2] = tempq.z;
  q[3] = tempq.w;      
}

void set_flow_measurement(float flow_msmx) {
  ins_flow.new_flow_measurement = true;
  ins_flow.optical_flow_x = flow_msmx;
}

void set_gyro_measurement(float gyro_x) {
  ins_flow.lp_gyro_roll = lp_factor * ins_flow.lp_gyro_roll + (1-lp_factor) * gyro_x;
}

/**
 * Logging variables of the flowest
 */
LOG_GROUP_START(flowest)
/**
 * @brief Test
 */
LOG_ADD(LOG_UINT32, counter_of, &counter_of)
LOG_ADD(LOG_FLOAT, X0, &OF_X[0])
LOG_ADD(LOG_FLOAT, X1, &OF_X[1])
LOG_ADD(LOG_FLOAT, X2, &OF_X[2])
LOG_GROUP_STOP(flowest)

/**
 * Settings and parameters for handling of the flowdecks
 * measurments
 */
PARAM_GROUP_START(flowest)
/**
 * @brief Reset Filter with a 1
 */
PARAM_ADD(PARAM_UINT8, reset_filter, &reset_filter)
/**
 * @brief Reset Filter with a 1
 */
PARAM_ADD(PARAM_UINT8, use_filter, &use_filter)
/**
 * @brief Reset Filter with a 1
 */
PARAM_ADD(PARAM_UINT8, run_filter, &run_filter)
/**
 * @brief Set standard devivation flow measurement (default: 2.0f)
 */
//PARAM_ADD_CORE(PARAM_FLOAT, flowStdFixed, &flowStdFixed)
PARAM_GROUP_STOP(flowest)

