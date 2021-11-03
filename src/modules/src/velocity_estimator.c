
#include "velocity_estimator.h"

#define DEBUG_MODULE "VELCOMP"
#include "debug.h"

void init_complementary_filter_vz(ComplementaryFilter2_t *filter, float k1, float k2, float Ts){
    // Common Denominator
    float tmp_den0 = 4.0f + 2.0f*k1*Ts + k2*Ts*Ts;
    filter->den[0] = 1.0f;
    filter->den[1] = (-8.0f + 2.0f*k2*Ts*Ts)/tmp_den0;
    filter->den[2] = (4.0f - 2.0f*k1*Ts + k2*Ts*Ts)/tmp_den0;

    // G1(z^-1)
    filter->num1[0] = 2.0f*Ts/tmp_den0;
    filter->num1[1] = 0.0f;
    filter->num1[2] = -2.0f*Ts/tmp_den0;

    // G2(z^-1)
    filter->num2[0] = (4.0f*k1 + 2.0f*k2*Ts)/tmp_den0;
    filter->num2[1] = (-8.0f*k1)/tmp_den0;
    filter->num2[2] = (4.0f*k1 - 2.0f*k2*Ts)/tmp_den0;

    // Memory
    filter->u1[0] = 0.0f;
    filter->u1[1] = 0.0f;
    filter->u2[0] = 0.0f;
    filter->u2[1] = 0.0f;
    filter->y[0] = 0.0f;
    filter->y[1] = 0.0f;
    DEBUG_PRINT("vz numerator1: %f, %f, %f, numerator1: %f, %f, %f, denominator %f, %f, %f", 
        (double) filter->num1[0], (double) filter->num1[1], (double) filter->num1[2], 
        (double) filter->num2[0], (double) filter->num2[1], (double) filter->num2[2],
        (double) filter->den[0], (double) filter->den[1], (double) filter->den[2]);
}



float update_complementary_filter(ComplementaryFilter2_t *filter, float u1, float u2){
    // Calculate output
    float y = filter->num1[0]*u1
            + filter->num1[1] * filter->u1[0] 
            + filter->num1[2] * filter->u1[1]
            + filter->num2[0] * u2
            + filter->num2[1] * filter->u2[0]
            + filter->num2[2] * filter->u2[1]
            - filter->den[1] * filter->y[0]
            - filter->den[2] * filter->y[1];

    // Update filter history
    filter->u1[1] = filter->u1[0];
    filter->u1[0] = u1;
    filter->u2[1] = filter->u2[0];
    filter->u2[0] = u2;
    filter->y[1] = filter->y[0];
    filter->y[0] = y;

    return y;
}