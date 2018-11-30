/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * zranger.h: Z-Ranger deck driver
 */

#ifndef _STEREOBOARD_H_
#define _STEREOBOARD_H_

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"
#include "stabilizer_types.h"
#include "deck_core.h"

#include "deck.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "debug.h"
#include "log.h"
#include "uart1.h"
#include "system.h"
#include "pprzlink/pprz_transport.h"
#include "pprz_datalink.h"
#include "pprzlink/intermcu_msg.h"

/* Main magneto pitot strcuture */
struct stereocam_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                   ///< If we received a message
};

extern uint16_t front_range_UD;
extern uint16_t back_range_UD;
extern uint16_t right_range_UD;
extern uint16_t left_range_UD;

extern bool stereoboard_isinit;

extern void stereoboardDeckInit(DeckInfo *info);
extern void stereoboardTask(void* arg);


#endif /* _ZRANGER_H_ */
