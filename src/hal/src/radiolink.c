/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * radiolink.c - Radio link layer
 */

#include <string.h>
#include <stdint.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"
#include "radiolink.h"
#include "syslink.h"
#include "crtp.h"
#include "configblock.h"
#include "log.h"
#include "led.h"
#include "ledseq.h"
#include "queuemonitor.h"



uint8_t movingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}
//static uint8_t rssi_beacon_filtered;
	  int pos_avg = 0;
	  long sum = 0;
	  int arrNumbers[10] = {35};
	  int len = sizeof(arrNumbers) / sizeof(int);

#define RADIOLINK_TX_QUEUE_SIZE (1)

static xQueueHandle  txQueue;
static xQueueHandle crtpPacketDelivery;

static bool isInit;

static int radiolinkSendCRTPPacket(CRTPPacket *p);
static int radiolinkSetEnable(bool enable);
static int radiolinkReceiveCRTPPacket(CRTPPacket *p);

//Local RSSI variable used to enable logging of RSSI values from Radio
static uint8_t rssi=130;
uint8_t rssi_ext = 130;
static uint8_t rssi_inter = 140;
uint8_t rssi_inter_ext = 140;
static uint8_t id_inter = 99;
uint8_t id_inter_ext = 99;
uint8_t own_id = 0;
static uint8_t rssi_beacon_inter = 140;
uint8_t rssi_beacon_inter_ext = 140;

static float rssi_angle_inter = 0.0f;
 float rssi_angle_inter_ext = 0.0f;


static struct crtpLinkOperations radiolinkOp =
{
  .setEnable         = radiolinkSetEnable,
  .sendPacket        = radiolinkSendCRTPPacket,
  .receivePacket     = radiolinkReceiveCRTPPacket,
};

void radiolinkInit(void)
{
  if (isInit)
    return;

  txQueue = xQueueCreate(RADIOLINK_TX_QUEUE_SIZE, sizeof(SyslinkPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(txQueue);
  crtpPacketDelivery = xQueueCreate(5, sizeof(CRTPPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);


  ASSERT(crtpPacketDelivery);

  syslinkInit();

  radiolinkSetChannel(configblockGetRadioChannel());
  radiolinkSetDatarate(configblockGetRadioSpeed());
  radiolinkSetAddress(configblockGetRadioAddress());

  uint64_t address = configblockGetRadioAddress();
  own_id = (uint8_t)((address) & 0x00000000ff);

  isInit = true;
}

bool radiolinkTest(void)
{
  return syslinkTest();
}

void radiolinkSetChannel(uint8_t channel)
{
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_CHANNEL;
  slp.length = 1;
  slp.data[0] = channel;
  syslinkSendPacket(&slp);
}

void radiolinkSetDatarate(uint8_t datarate)
{
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_DATARATE;
  slp.length = 1;
  slp.data[0] = datarate;
  syslinkSendPacket(&slp);
}

void radiolinkSetAddress(uint64_t address)
{
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_ADDRESS;
  slp.length = 5;
  memcpy(&slp.data[0], &address, 5);
  syslinkSendPacket(&slp);
}

void radiolinkSetPowerDbm(int8_t powerDbm)
{
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_POWER;
  slp.length = 1;
  slp.data[0] = powerDbm;
  syslinkSendPacket(&slp);
}

void radiolinkSendInfoGradientBug(int8_t state, float angle_rssi )
{
	SyslinkPacket slp;

	slp.type = SYSLINK_GRADIENT_BUG;
	slp.length = 5;
	slp.data[0] = state;
	memcpy(&slp.data[1], &angle_rssi, sizeof(float));

	syslinkSendPacket(&slp);
}


void radiolinkSyslinkDispatch(SyslinkPacket *slp)
{
  static SyslinkPacket txPacket;
  if (slp->type == SYSLINK_RADIO_RAW)
  {
    slp->length--; // Decrease to get CRTP size.
    xQueueSend(crtpPacketDelivery, &slp->length, 0);
    ledseqRun(LINK_LED, seq_linkup);
    // If a radio packet is received, one can be sent
    if (xQueueReceive(txQueue, &txPacket, 0) == pdTRUE)
    {
      ledseqRun(LINK_DOWN_LED, seq_linkup);
      syslinkSendPacket(&txPacket);
    }
  } else if (slp->type == SYSLINK_RADIO_RAW_BROADCAST)
  {
    slp->length--; // Decrease to get CRTP size.
    xQueueSend(crtpPacketDelivery, &slp->length, 0);
    ledseqRun(LINK_LED, seq_linkup);
    // no ack for broadcasts
  } else if (slp->type == SYSLINK_RADIO_RSSI)
	{
		//Extract RSSI sample sent from radio
		memcpy(&rssi, slp->data, sizeof(uint8_t));
		//rssi_ext = rssi;


		rssi_ext = (uint8_t)movingAvg(arrNumbers, &sum, pos_avg, len, (int)rssi);
		pos_avg++;
	    if (pos_avg >= len){
	    	pos_avg = 0;
	    }

	}else if (slp->type == SYSLINK_RADIO_RSSI_INTER)
	{
		//Extract RSSI sample sent from radio
		memcpy(&rssi_inter, slp->data, sizeof(uint8_t));
		rssi_inter_ext = rssi_inter;
		memcpy(&id_inter, slp->data+1, sizeof(uint8_t));
		id_inter_ext = id_inter;
		memcpy(&rssi_beacon_inter, slp->data+2, sizeof(uint8_t));
		rssi_beacon_inter_ext = rssi_beacon_inter;
		memcpy(&rssi_angle_inter, slp->data+3, sizeof(float));
		rssi_angle_inter_ext = rssi_angle_inter;
	}
}

static int radiolinkReceiveCRTPPacket(CRTPPacket *p)
{
  if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE)
  {
    return 0;
  }

  return -1;
}

static int radiolinkSendCRTPPacket(CRTPPacket *p)
{
  static SyslinkPacket slp;

  ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  slp.type = SYSLINK_RADIO_RAW;
  slp.length = p->size + 1;
  memcpy(slp.data, &p->header, p->size + 1);

  if (xQueueSend(txQueue, &slp, M2T(100)) == pdTRUE)
  {
    return true;
  }

  return false;
}

struct crtpLinkOperations * radiolinkGetLink()
{
  return &radiolinkOp;
}

static int radiolinkSetEnable(bool enable)
{
  return 0;
}

LOG_GROUP_START(radio)
LOG_ADD(LOG_UINT8, rssi, &rssi)
LOG_ADD(LOG_UINT8, rssi_inter, &rssi_inter)
LOG_ADD(LOG_UINT8, id_inter, &id_inter)
//LOG_ADD(LOG_UINT8, rssi_b_inter, &rssi_beacon_inter)
LOG_GROUP_STOP(radio)
