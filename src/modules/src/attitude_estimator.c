// Guido de Croon, TU Delft

#include "attitude_estimator.h"
#include "arm_math.h"

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
	    OF_X[OF_ANGLE_IND] += dt * (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll); 
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

  arm_matrix_instance_f32 Phi = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) F};
  arm_matrix_instance_f32 Gamma = { N_STATES_OF_KF, N_STATES_OF_KF, (float*) G};
  arm_matrix_instance_f32 Jac = { N_MEAS_OF_KF, N_STATES_OF_KF, (float*) H};

  // Corresponding MATLAB statement:    :O
  // P_k1_k = Phi_k1_k*P*Phi_k1_k' + Gamma_k1_k*Q*Gamma_k1_k';



  
  float _PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PhiT, _PhiT, N_STATES_OF_KF);
  float _P_PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PPhiT, _P_PhiT, N_STATES_OF_KF);
  float _Phi_P_PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PhiPPhiT, _Phi_P_PhiT, N_STATES_OF_KF);

  float_mat_transpose(PhiT, Phi, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(PPhiT, P, PhiT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(PhiPPhiT, Phi, PPhiT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);

  DEBUG_PRINT("Phi*P*PhiT:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, PhiPPhiT);

  float _GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(GT, _GT, N_STATES_OF_KF);
  float _Q_GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(QGT, _Q_GT, N_STATES_OF_KF);
  float _G_Q_GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(GQGT, _G_Q_GT, N_STATES_OF_KF);

  float_mat_transpose(GT, Gamma, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(QGT, Q, GT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(GQGT, Gamma, QGT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);

  DEBUG_PRINT("Gamma*Q*GammaT:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, GQGT);

  float_mat_sum(P, PhiPPhiT, GQGT, N_STATES_OF_KF, N_STATES_OF_KF);
  DEBUG_PRINT("P:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

  // correct state when there is a new vision measurement:
  if(ins_flow.new_flow_measurement) {

    DEBUG_PRINT("*********************\n");
    DEBUG_PRINT("   NEW MEASUREMENT   \n");
    DEBUG_PRINT("*********************\n");

    // determine Kalman gain:
    // MATLAB statement:
    // S_k = Hx*P_k1_k*Hx' + R;
    float _JacT[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(JacT, _JacT, N_STATES_OF_KF);
    float _P_JacT[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(PJacT, _P_JacT, N_STATES_OF_KF);
    float _Jac_P_JacT[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(JacPJacT, _Jac_P_JacT, N_MEAS_OF_KF);

    float_mat_transpose(JacT, Jac, N_MEAS_OF_KF, N_STATES_OF_KF);
    float_mat_mul(PJacT, P, JacT, N_STATES_OF_KF, N_STATES_OF_KF, N_MEAS_OF_KF);
    DEBUG_PRINT("P*JacT:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_MEAS_OF_KF, PJacT);

    float_mat_mul(JacPJacT, Jac, PJacT, N_MEAS_OF_KF, N_STATES_OF_KF, N_MEAS_OF_KF);

    DEBUG_PRINT("Jac*P*JacT:\n");
    DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, JacPJacT);

    float _S[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(S, _S, N_MEAS_OF_KF);
    float_mat_sum(S, JacPJacT, R, N_MEAS_OF_KF, N_MEAS_OF_KF);

    DEBUG_PRINT("S:\n");
    DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, S);

    // MATLAB statement:
    // K_k1 = P_k1_k*Hx' * inv(S_k);
    float _K[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(K, _K, N_STATES_OF_KF);
    float _INVS[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(INVS, _INVS, N_MEAS_OF_KF);
    float_mat_invert(INVS, S, N_MEAS_OF_KF);
    if(DEBUG_INS_FLOW) {
	// This should be the identity matrix:
	float _SINVS[N_MEAS_OF_KF][N_MEAS_OF_KF];
	MAKE_MATRIX_PTR(SINVS, _SINVS, N_MEAS_OF_KF);
	float_mat_mul(SINVS, S, INVS, N_MEAS_OF_KF, N_MEAS_OF_KF, N_MEAS_OF_KF);
	DEBUG_PRINT("S*Inv(S):\n");
	DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, SINVS);
    }

    float_mat_mul(K, PJacT, INVS, N_STATES_OF_KF, N_MEAS_OF_KF, N_MEAS_OF_KF);
    DEBUG_PRINT("K:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_MEAS_OF_KF, K);

    // Correct the state:
    // MATLAB:
    // Z_expected = [-v*cos(theta)*cos(theta)/z + zd*sin(2*theta)/(2*z) + thetad;
    //			(-v*sin(2*theta)/(2*z)) - zd*cos(theta)*cos(theta)/z];
    float Z_expected[N_MEAS_OF_KF];

    // TODO: take this var out? It was meant for debugging...
    float Z_expect_GT_angle;

    if(CONSTANT_ALT_FILTER) {
      Z_expected[OF_LAT_FLOW_IND] = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
				+OF_X[OF_ANGLE_DOT_IND]; // TODO: Currently, no p works better than using p here. Analyze!

      /* TODO: remove later, just for debugging:
      float Z_exp_no_rate = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];
      float Z_exp_with_rate = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]+OF_X[OF_ANGLE_DOT_IND];
      printf("Z_exp_no_rate = %f, Z_exp_with_rate = %f, measured = %f, angle dot = %f, p = %f: ", Z_exp_no_rate, Z_exp_with_rate,
	     ins_flow.optical_flow_x, OF_X[OF_ANGLE_DOT_IND], dt * (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f);
      if(fabs(ins_flow.optical_flow_x - Z_exp_no_rate) < fabs(ins_flow.optical_flow_x - Z_exp_with_rate)) {
	  printf("NO RATE WINS!");
      }
      printf("\n");*/

      Z_expected[OF_DIV_FLOW_IND] = -OF_X[OF_V_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND]);

      if(OF_TWO_DIM) {
	  Z_expected[OF_LAT_FLOW_X_IND] = -OF_X[OF_VX_IND]*cos(OF_X[OF_THETA_IND])*cos(OF_X[OF_THETA_IND])/OF_X[OF_Z_IND]; // TODO: no q?
      }

      Z_expect_GT_angle = -OF_X[OF_V_IND]*cos(eulers->phi)*cos(eulers->phi)/OF_X[OF_Z_IND];

      if(OF_USE_GYROS) {
	  Z_expected[OF_RATE_IND] = OF_X[OF_ANGLE_DOT_IND]; // TODO: is this even in the right direction?
      }
    }
    else {
      Z_expected[OF_LAT_FLOW_IND] = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
				     + OF_X[OF_Z_DOT_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND])
				     + OF_X[OF_ANGLE_DOT_IND]; // TODO: We first had this rate term but not for the constant alt filter.
							       // Simulation and data analysis from real flights shows that including it is better. CHECK IN REALITY!

      Z_expected[OF_DIV_FLOW_IND] = -OF_X[OF_V_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND])
				    -OF_X[OF_Z_DOT_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];

      Z_expect_GT_angle = -OF_X[OF_V_IND]*cos(eulers->phi)*cos(eulers->phi)/OF_X[OF_Z_IND]
					     + OF_X[OF_Z_DOT_IND]*sin(2*eulers->phi)/(2*OF_X[OF_Z_IND]);
					     //+ OF_X[OF_ANGLE_DOT_IND];
      if(N_MEAS_OF_KF == 3) {
      	Z_expected[OF_RATE_IND] = OF_X[OF_ANGLE_DOT_IND]; // TODO: is this even in the right direction?
      }


      float Z_exp_no_rate = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
			    + OF_X[OF_Z_DOT_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND]);
      float Z_exp_with_rate = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
			      + OF_X[OF_Z_DOT_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND])
			      + OF_X[OF_ANGLE_DOT_IND];

      /*
      printf("Z_exp_no_rate = %f, Z_exp_with_rate = %f, measured = %f, angle dot = %f, p = %f: ", Z_exp_no_rate, Z_exp_with_rate,
       ins_flow.optical_flow_x, OF_X[OF_ANGLE_DOT_IND], dt * (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f);
      if(fabs(ins_flow.optical_flow_x - Z_exp_no_rate) < fabs(ins_flow.optical_flow_x - Z_exp_with_rate)) {
	  printf("NO RATE WINS!");
      }
      printf("\n");*/
    }

    //  i_k1 = Z - Z_expected;
    float innovation[N_MEAS_OF_KF][1];
    //print_ins_flow_state();
    innovation[OF_LAT_FLOW_IND][0] = ins_flow.optical_flow_x - Z_expected[OF_LAT_FLOW_IND];
    DEBUG_PRINT("Expected flow filter: %f, Expected flow ground truth = %f, Real flow x: %f, Real flow y: %f.\n", Z_expected[OF_LAT_FLOW_IND], Z_expect_GT_angle, ins_flow.optical_flow_x, ins_flow.optical_flow_y);
    innovation[OF_DIV_FLOW_IND][0] = ins_flow.divergence - Z_expected[OF_DIV_FLOW_IND];
    DEBUG_PRINT("Expected div: %f, Real div: %f.\n", Z_expected[OF_DIV_FLOW_IND], ins_flow.divergence);
    if(CONSTANT_ALT_FILTER && OF_TWO_DIM) {
	innovation[OF_LAT_FLOW_X_IND][0] = ins_flow.optical_flow_y - Z_expected[OF_LAT_FLOW_X_IND];
	DEBUG_PRINT("Expected flow in body X direction filter: %f, Real flow in corresponding y direction: %f, gyro = %f, expected velocity = %f, real velocity = %f, expected theta = %f, real theta = %f.\n",
	       Z_expected[OF_LAT_FLOW_X_IND], ins_flow.optical_flow_y, ins_flow.lp_gyro_pitch - ins_flow.lp_gyro_bias_pitch, OF_X[OF_VX_IND], velocities->x, OF_X[OF_THETA_IND], eulers->theta);
    }
    if(OF_USE_GYROS) {
	float gyro_meas_roll;
	gyro_meas_roll = (ins_flow.lp_gyro_roll - ins_flow.lp_gyro_bias_roll) * (M_PI/180.0f) / 74.0f;

	// TODO: You can fake gyros here by estimating them as follows:
	// rate_p_filt_est = -1.8457e-04 * cmd_roll;
	// gyro_meas_roll = -1.8457e-04 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);
	// gyro_meas_roll = -2.0e-03 * (stabilization_cmd[COMMAND_ROLL]-ins_flow.lp_roll_command);

	innovation[OF_RATE_IND][0] = gyro_meas_roll - Z_expected[OF_RATE_IND];
	//innovation[OF_RATE_IND][0] = rates->p - Z_expected[OF_RATE_IND];
	DEBUG_PRINT("Expected rate: %f, Real rate: %f.\n", Z_expected[OF_RATE_IND], ins_flow.lp_gyro_roll);
    }

    MAKE_MATRIX_PTR(I, innovation, N_MEAS_OF_KF);
    DEBUG_PRINT("Innovation:");
    DEBUG_MAT_PRINT(N_MEAS_OF_KF, 1, I);

    // X_k1_k1 = X_k1_k + K_k1*(i_k1);
    float _KI[N_STATES_OF_KF][1];
    MAKE_MATRIX_PTR(KI, _KI, N_STATES_OF_KF);
    float_mat_mul(KI, K, I, N_STATES_OF_KF, N_MEAS_OF_KF, 1);

    DEBUG_PRINT("K*innovation:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, 1, KI);

    DEBUG_PRINT("PRE: v = %f, angle = %f\n", OF_X[OF_V_IND], OF_X[OF_ANGLE_IND]);
    for(int i = 0; i < N_STATES_OF_KF; i++) {
	OF_X[i] += KI[i][0];
    }
    DEBUG_PRINT("POST v: %f, angle = %f\n", OF_X[OF_V_IND], OF_X[OF_ANGLE_IND]);

    DEBUG_PRINT("Angles (deg): ahrs = %f, ekf = %f.\n", (180.0f/M_PI)*eulers->phi, (180.0f/M_PI)*OF_X[OF_ANGLE_IND]);

    DEBUG_PRINT("P before correction:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

    // P_k1_k1 = (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)' + K_k1*R*K_k1'; % Joseph form of the covariance update equation
    float _KJac[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KJac, _KJac, N_STATES_OF_KF);
    float_mat_mul(KJac, K, Jac, N_STATES_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);

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