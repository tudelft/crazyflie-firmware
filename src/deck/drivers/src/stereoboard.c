/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * crazycar.c - Deck driver for the Crazyflie 2.0 Crazycar deck
 */
#define DEBUG_MODULE "StereoboardDeck"

#include "stereoboard.h"
#include "zranger.h"
#include "arm_math.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "param.h"

#include "commander.h"

#define COMMANDER_PRIORITY_HOMING 3

uint16_t range_last;

//static char ch = 'a';
static float velx, vely, velz;
static float homingvector_x, homingvector_y;
static float motion_x, motion_y;
static uint8_t make_snapshot = 0;
static uint8_t follow_vector = 0;
uint8_t msg_id_check = 0;
uint8_t use_stereoboard = 1;
/*struct stereocam_t stereocam = {
 .device = 0,
 .msg_available = false
 };*/

//laser range
uint16_t up_range = 0;
uint16_t bottom_range = 0;
uint8_t id_range = 0;



struct UartDataStruct USART1_Data;
uint8_t msg_buf[4 * 64];           // define local data

/* struct pprz_transport pprz;
 struct link_device dev;*/
struct uint8array
{
  uint8_t len;
  uint8_t height;
  uint8_t *data;
  bool data_new;
};

static setpoint_t sp;

void stereoboardTask(void* arg)
{

  systemWaitStart();
  struct uint8array stereocam_data = { .len = 0, .data = msg_buf, .data_new = 0,
      .height = 0 };  // buffer used to contain image without line endings

  while (1) {
    pprz_check_and_parse(&dev, &pprz, stereocam_data.data,
        &stereocam_data.data_new);

    uint8_t msg_id = stereocam_data.data[1];

    //DEBUG_PRINT("msgID is %d\n",msg_id);

    msg_id_check = msg_id;

    motion_x = (float) msg_id;
    switch (msg_id) {
      case 90:
        homingvector_x = DL_STEREOCAM_VISUALHOMING_X(stereocam_data.data);
        homingvector_y = DL_STEREOCAM_VISUALHOMING_Y(stereocam_data.data);
        pprz_msg_send_STEREOCAM_VISUALHOMING_COMMAND(&(pprz.trans_tx), &dev, 0,
            &make_snapshot);

        if(follow_vector) {
          memset(&sp, 0, sizeof(sp));
          sp.mode.x = modeVelocity;
          sp.mode.y = modeVelocity;
          sp.mode.z = modeVelocity;
          sp.velocity.x = homingvector_x;
          sp.velocity.y = homingvector_y;
          sp.velocity.z = 0.0;
          sp.mode.yaw = modeVelocity;
          sp.attitudeRate.yaw = 0.0;
          commanderSetSetpoint(&sp, COMMANDER_PRIORITY_HOMING);
        }

        break;
      case 15:

    	  id_range= DL_IMCU_REMOTE_GROUND_id(stereocam_data.data);
    	  if (id_range == 0)
    		  up_range = DL_IMCU_REMOTE_GROUND_range(stereocam_data.data);
    	  else if(id_range == 2)
    		  bottom_range = DL_IMCU_REMOTE_GROUND_range(stereocam_data.data);

    	  break;
      default:
        break;
    }
    //DEBUG_PRINT("%d, check!\n",vel);

    vTaskDelay(10);
  }
}

/* Initialize the deck driver */
void stereoboardDeckInit(DeckInfo *info)
{
  //uart1Init(115200);
  uart1Init(38400);

  datalink_init(&USART1_Data);

  DEBUG_PRINT("Test StereoboardDeck!\n");

  xTaskCreate(stereoboardTask, UART_RX_TASK_NAME, UART_RX_TASK_STACKSIZE, NULL,
      3, NULL);

}

static const DeckDriver stereoboard_deck = {
    .vid = 0xBC,
    .pid = 0x00,
    .name = "bcStereoboard",
    .usedPeriph = DECK_USING_UART1,
    .usedGpio = DECK_USING_TX1 | DECK_USING_RX1,

    .init = stereoboardDeckInit
};

DECK_DRIVER(stereoboard_deck);

LOG_GROUP_START(updown_laser)
LOG_ADD(LOG_UINT16, up_range, &up_range)
LOG_ADD(LOG_UINT16, bottom_range, &bottom_range)
LOG_GROUP_STOP(updown_laser)

/*LOG_GROUP_START(stereoboard)
LOG_ADD(LOG_FLOAT, velocity x, &velx)
LOG_ADD(LOG_FLOAT, velocity y, &vely)
LOG_ADD(LOG_FLOAT, velocity z, &velz)
LOG_ADD(LOG_FLOAT, motion_x, &motion_x)
LOG_ADD(LOG_FLOAT, motion_y, &motion_y)
LOG_GROUP_STOP(stereoboard)

LOG_GROUP_START(monocam)
LOG_ADD(LOG_FLOAT, homingvector_x, &homingvector_x)
LOG_ADD(LOG_FLOAT, homingvector_y, &homingvector_y)
LOG_ADD(LOG_UINT8, make_snapshot, &make_snapshot)
LOG_ADD(LOG_UINT8, msg_id_check, &msg_id_check)

LOG_GROUP_STOP(monocam)

PARAM_GROUP_START(visualhoming)
PARAM_ADD(PARAM_UINT8, make_snapshot, &make_snapshot)
PARAM_ADD(PARAM_UINT8, follow_vector, &follow_vector)
PARAM_GROUP_STOP(visualhoming)*/
