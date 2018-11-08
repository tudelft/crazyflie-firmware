/*
 * lobe_navigation.h
 *
 *  Created on: Nov 7, 2018
 *      Author: knmcguire
 */

#ifndef SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_LOBE_NAVIGATION_H_
#define SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_LOBE_NAVIGATION_H_
#include <stdint.h>
#include <stdbool.h>
float lobe_navigator(float* vel_x, float* vel_y, float* vel_w, float front_range, float side_range, float current_heading, float current_pos_x, float current_pos_y, uint8_t rssi);
void init_lobe_navigator();

#endif /* SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_LOBE_NAVIGATION_H_ */
