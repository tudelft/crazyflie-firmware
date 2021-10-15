// Guido de Croon, TU Delft

#include "attitude_estimator.h"
#include "arm_math.h"
#include "cf_math.h"

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
	    float parameters[20] = {8.407886e-03, 4.056093e-01, 1.555919e-01, 1.291584e-01, 2.594766e-01, 1.927331e-01, 9.599609e-02, 1.688265e-01, 5.589618e-02, 1.605665e-01, 1.195912e-01, 1.809532e+00, 4.268251e-02, 3.003060e+00, 1.098473e+00, 1.944433e-01, 2.363352e-01, 1.110390e+00, 1.190994e+00, 6.211962e-01};
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
bool reset_filter;
int use_filter;
bool run_filter;
uint32_t counter;
float thrust_factor;

// matrices for state, actuation noise, state uncertainty, and measurement noise:

float OF_X[N_STATES_OF_KF] = {0.};
float OF_Q[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
float OF_P[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
float OF_R[N_MEAS_OF_KF][N_MEAS_OF_KF] = {{0.}};
static __attribute__((aligned(4))) arm_matrix_instance_f32 OF_Xm = { N_STATES_OF_KF, 1, (float *)OF_X};
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
  
  reset_filter = false;
  use_filter = 0;
  run_filter = false;

  /* TODO:  how to get time? 
  of_time = get_sys_time_float();
  of_prev_time = get_sys_time_float();
  */

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
  counter = 0;
}

void estimator_OF_att(state_t *state, const uint32_t tick)
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

  if(reset_filter) {
      ins_reset_filter();
      reset_filter = false;
  }

  // assuming that the typical case is no rotation, we can estimate the (initial) bias of the gyro:
  ins_flow.lp_gyro_bias_roll = lp_factor_strong * ins_flow.lp_gyro_bias_roll + (1-lp_factor_strong) * ins_flow.lp_gyro_roll;
  float gyro_msm = (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll);

  // TODO: get the new time: Using tick?
  //of_time = get_sys_time_float();
  float dt = 0.01; //of_time - of_prev_time;
  if(dt > 1.0f) {
      dt = 0.01f;
  }

  // propagate the state with Euler integration:
  if(CONSTANT_ALT_FILTER) {
      OF_X[OF_V_IND] += dt * (g * tan(OF_X[OF_ANGLE_IND]));
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
  if(OF_X[OF_Z_IND] < 1e-2) {
      OF_X[OF_Z_IND] = 1e-2;
  }

  // prepare the update and correction step:
  // we have to recompute these all the time, as they depend on the state:
  // discrete version of state transition matrix F: (ignoring t^2)
  float F[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
  for(int i = 0; i < N_STATES_OF_KF; i++) {
      F[i][i] = 1.0f;
  }
  if(CONSTANT_ALT_FILTER) {
    F[OF_V_IND][OF_ANGLE_IND] = dt*(g/(cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])));
  }
  if(OF_DRAG) {
      // In MATLAB: -sign(v)*2*kd*v/m (always minus, whether v is positive or negative):
      F[OF_V_IND][OF_V_IND] -=  dt * 2 * kd * abs(OF_X[OF_V_IND]) / mass;
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
      // Hx = [-cos(theta)^2/z, (v*sin(theta))/ z, (v* cos(theta)^2)/z^2];
    // lateral flow:
    H[OF_LAT_FLOW_IND][OF_V_IND] = -cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/ OF_X[OF_Z_IND];
    H[OF_LAT_FLOW_IND][OF_ANGLE_IND] = OF_X[OF_V_IND]*sin(2*OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];
    H[OF_LAT_FLOW_IND][OF_Z_IND] = OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/(OF_X[OF_Z_IND]*OF_X[OF_Z_IND]);
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
    mat_inv(&Sm, &INVSm);
    mat_mult(&P_JacTm, &INVSm, &Km);

    // Correct the state:
    // MATLAB:
    // Z_expected = [-v*cos(theta)*cos(theta)/z + zd*sin(2*theta)/(2*z) + thetad;
    //			(-v*sin(2*theta)/(2*z)) - zd*cos(theta)*cos(theta)/z];
    float Z_expected[N_MEAS_OF_KF];

    if(CONSTANT_ALT_FILTER) {
      Z_expected[OF_LAT_FLOW_IND] = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]+gyro_msm;
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
    __attribute__((aligned(4))) arm_matrix_instance_f32 K_Jacm = { N_STATES_OF_KF, 1, (float*) K_Jac};
    mat_mult(&Km, &Jacm, &K_Jacm);

    float _eye[N_STATES_OF_KF][N_STATES_OF_KF];
    
    
    MAKE_MATRIX_PTR(eye, _eye, N_STATES_OF_KF);
    float_mat_diagonal_scal(eye, 1.0, N_STATES_OF_KF);
    DEBUG_PRINT("eye:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, eye);

    float _eKJac[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(eKJac, _eKJac, N_STATES_OF_KF);
    float_mat_diff(eKJac, eye, KJac, N_STATES_OF_KF, N_STATES_OF_KF);
    DEBUG_PRINT("eKJac:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, eKJac);

    float _eKJacT[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(eKJacT, _eKJacT, N_STATES_OF_KF);
    float_mat_transpose(eKJacT, eKJac, N_STATES_OF_KF, N_STATES_OF_KF);
    // (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)'
    float _P_pre[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(P_pre, _P_pre, N_STATES_OF_KF);
    float_mat_mul(P_pre, P, eKJacT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
    float_mat_mul(P, eKJac, P_pre, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
    DEBUG_PRINT("eKJac * P *eKJacT:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

    // K_k1*R*K_k1'
    // TODO: check all MAKE_MATRIX that they mention the number of ROWS!
    float _KT[N_MEAS_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KT, _KT, N_MEAS_OF_KF);
    float_mat_transpose(KT, K, N_STATES_OF_KF, N_MEAS_OF_KF);
    float _RKT[N_MEAS_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(RKT, _RKT, N_MEAS_OF_KF);
    float_mat_mul(RKT, R, KT, N_MEAS_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);
    float _KRKT[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KRKT, _KRKT, N_STATES_OF_KF);
    float_mat_mul(KRKT, K, RKT, N_STATES_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);
    DEBUG_PRINT("KRKT:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, KRKT);

    // summing the two parts:
    float_mat_sum(P, P, KRKT, N_STATES_OF_KF, N_STATES_OF_KF);

    DEBUG_PRINT("P corrected:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);
    float trace_P = 0.0f;
    for(int i = 0; i < N_STATES_OF_KF; i++) {
	trace_P += P[i][i];
    }
    DEBUG_PRINT("trace P = %f\n", trace_P);

    // indicate that the measurement has been used:
    ins_flow.new_flow_measurement = false;
  }

  // update the time:
  of_prev_time = of_time;
}