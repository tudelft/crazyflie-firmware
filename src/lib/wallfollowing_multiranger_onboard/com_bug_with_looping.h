/*
 * com_bug_with_looping.h
 *
 *  Created on: Nov 8, 2018
 *      Author: knmcguire
 */

#ifndef SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_COM_BUG_WITH_LOOPING_H_
#define SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_COM_BUG_WITH_LOOPING_H_

void init_com_bug_loop_controller(float new_ref_distance_from_wall, float max_speed_ref);
int com_bug_loop_controller(float* vel_x, float* vel_y, float* vel_w, float front_range, float left_range, float right_range, float current_heading, float current_pos_x, float current_pos_y);

#endif /* SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_COM_BUG_WITH_LOOPING_H_ */
