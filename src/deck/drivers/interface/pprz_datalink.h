/*
 * datalink.h
 *
 *  Created on: 22 Aug 2017
 *      Author: kirk
 */

#ifndef STEREOBOARD_PPRZ_DATALINK_H_
#define STEREOBOARD_PPRZ_DATALINK_H_

#include "uart1.h"
#include "pprzlink/pprz_transport.h"


/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "queue.h"

extern struct pprz_transport pprz;
extern struct link_device dev;


extern void datalink_init(struct UartDataStruct* periph);

#endif /* STEREOBOARD_PPRZ_DATALINK_H_ */
