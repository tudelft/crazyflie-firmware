#ifndef VELOCITY_ESTIMATOR_H_
#define VELOCITY_ESTIMATOR_H_

#include "stabilizer_types.h"


/** 2nd Order Complementary filter in z domain (G(z^-1))
 * Y = G1*U1 + G2*U2
 * 
 *            n[0] + n[1]*z^-1 + n[2]*z^-2
 * G(z^-1) = ------------------------------
 *            d[0] + d[1]*z^-1 + d[2]*z^-2
 */
typedef struct ComplementaryFilter2_s {
  float num1[3]; // Numerator G1
  float num2[3]; // Numerator G2
  float den[3]; // Denominator (Common)
  float u1[2]; // Input1 history
  float u2[2]; // Input2 history
  float y[2]; // Output history
} ComplementaryFilter2_t;


/** Initialize complementary filter for vz estimation from baro and accelerometer
 * Discretization using bilinear transform of Y(s) = G1(s)*U1(s) + G2(s)*U2(s)
 * G1(s) = s/(s^2 + k1*s + k2)
 * G2(s) = s*(k1*s + k2)/(s^2 + k1*s + k2)
 * 
 * @param filter 2nd order complementary filter structure for vz
 * @param k1 parameter
 * @param k2 parameter
 * @param Ts sample time
 */
void init_complementary_filter_vz(ComplementaryFilter2_t *filter, float k1, float k2, float Ts);


/** Update 2nd order complementary filter from inputs
 *
 * @param filter 2nd order complementary filter structure
 * @param u1 input 1
 * @param u2 input 2
 * @return filtered output value
 */
float update_complementary_filter(ComplementaryFilter2_t *filter, float u1, float u2);

#endif // VELOCITY_ESTIMATOR_H_