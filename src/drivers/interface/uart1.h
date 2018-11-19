/**
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
 * uart1.h - uart1 driver for deck port
 */
#ifndef UART1_H_
#define UART1_H_

#include <stdbool.h>
#include "eprintf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "list.h"

#define UART1_BAUDRATE           9600
#define UART1_DATA_TIMEOUT_MS    1000
#define UART1_DATA_TIMEOUT_TICKS (UART1_DATA_TIMEOUT_MS / portTICK_RATE_MS)

#define UART1_TYPE             USART3
#define UART1_PERIF            RCC_APB1Periph_USART3
#define ENABLE_UART1_RCC       RCC_APB1PeriphClockCmd
#define UART1_IRQ              USART3_IRQn

#define UART1_DMA_IRQ          DMA1_Channel2_IRQn
#define UART1_DMA_IT_TC        DMA1_IT_TC2
#define UART1_DMA_CH           DMA1_Channel2

#define UART1_GPIO_PERIF       RCC_AHB1Periph_GPIOC
#define UART1_GPIO_PORT        GPIOC
#define UART1_GPIO_TX_PIN      GPIO_Pin_10
#define UART1_GPIO_RX_PIN      GPIO_Pin_11
#define UART1_GPIO_AF_TX_PIN   GPIO_PinSource10
#define UART1_GPIO_AF_RX_PIN   GPIO_PinSource11
#define UART1_GPIO_AF_TX       GPIO_AF_USART3
#define UART1_GPIO_AF_RX       GPIO_AF_USART3


#define TXBUFFERSIZE    (64*64) // 4 KByte
#define RXBUFFERSIZE    (64*64) // 4 KByte


struct UartDataStruct {
int *device;
};

/*
 * Definition of the queue used by the scheduler.
 * Items are queued by copy, not reference.  See the following link for the
 * rationale: http://www.freertos.org/Embedded-RTOS-Queues.html
 */
typedef struct QueueDefinition
{
  int8_t *pcHead;         /*< Points to the beginning of the queue storage area. */
  int8_t *pcTail;         /*< Points to the byte at the end of the queue storage area.  Once more byte is allocated than necessary to store the queue items, this is used as a marker. */
  int8_t *pcWriteTo;        /*< Points to the free next place in the storage area. */

  union             /* Use of a union is an exception to the coding standard to ensure two mutually exclusive structure members don't appear simultaneously (wasting RAM). */
  {
    int8_t *pcReadFrom;     /*< Points to the last place that a queued item was read from when the structure is used as a queue. */
    UBaseType_t uxRecursiveCallCount;/*< Maintains a count of the number of times a recursive mutex has been recursively 'taken' when the structure is used as a mutex. */
  } u;

  List_t xTasksWaitingToSend;   /*< List of tasks that are blocked waiting to post onto this queue.  Stored in priority order. */
  List_t xTasksWaitingToReceive;  /*< List of tasks that are blocked waiting to read from this queue.  Stored in priority order. */

  volatile UBaseType_t uxMessagesWaiting;/*< The number of items currently in the queue. */
  UBaseType_t uxLength;     /*< The length of the queue defined as the number of items it will hold, not the number of bytes. */
  UBaseType_t uxItemSize;     /*< The size of each items that the queue will hold. */

  volatile BaseType_t xRxLock;  /*< Stores the number of items received from the queue (removed from the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */
  volatile BaseType_t xTxLock;  /*< Stores the number of items transmitted to the queue (added to the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */

  #if ( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t uxQueueNumber;
    uint8_t ucQueueType;
  #endif

  #if ( configUSE_QUEUE_SETS == 1 )
    struct QueueDefinition *pxQueueSetContainer;
  #endif

} xQUEUE;

/**
 * Initialize the UART.
 */
void uart1Init(const uint32_t baudrate);

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool uart1Test(void);

/**
 * Read a byte of data from incoming queue with a timeout defined by UART1_DATA_TIMEOUT_MS
 * @param[out] c  Read byte
 * @return true if data, false if timeout was reached.
 */
bool uart1GetDataWithTimout(uint8_t *c);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uart1SendData(uint32_t size, uint8_t* data);

/**
 * Send a single character to the serial port using the uartSendData function.
 * @param[in] ch Character to print. Only the 8 LSB are used.
 *
 * @return Character printed
 */
int uart1Putchar(int ch);

void uart1Getchar(char * ch);

bool uart1CharAvailable(void);


/**
 * Uart printf macro that uses eprintf
 * @param[in] FMT String format
 * @param[in] ... Parameters to print
 *
 * @note If UART Crtp link is activated this function does nothing
 */
#define uart1Printf(FMT, ...) eprintf(uart1Putchar, FMT, ## __VA_ARGS__)

#endif /* UART1_H_ */
