/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * mtf02deck.c: MTF-02 optical flow and range sensor deck driver
 *
 * The MTF-02 outputs data in MSPv2 format. Observed message patterns:
 *   - Rangefinder: $X< 00 01 1F 05 00 [5 bytes payload] [crc]
 *   - Optical Flow: $X< 00 02 1F 09 00 [9 bytes payload] [crc]
 */

#define DEBUG_MODULE "MTF02"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"

#include "uart1.h"
#include "mtf02deck.h"

#include "stabilizer_types.h"
#include "estimator.h"

#include "cf_math.h"
#include "usec_time.h"

#include <string.h>
#include <stdlib.h>

// Measurement noise model for range sensor
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f;
static const float expPointB = 4.0f;
static const float expStdB = 0.2f;
static float expCoeff;

// Range outlier limit (mm)
#define RANGE_OUTLIER_LIMIT 5000

// Flow outlier limit
#define FLOW_OUTLIER_LIMIT 100

// UART baudrate
#define MTF02_BAUDRATE 115200

// Minimum flow quality threshold
#define MIN_FLOW_QUALITY 30

static bool isInit = false;

// Logged values
static uint32_t lastDistance = 0;
static int32_t lastFlowX = 0;
static int32_t lastFlowY = 0;
static uint8_t lastFlowQuality = 0;
static uint8_t lastRangeQuality = 0;

// Flow values sent to EKF (after axis transformation)
static float lastDpixelX = 0.0f;
static float lastDpixelY = 0.0f;
static float lastFlowDt = 0.0f;

// Debug counters
static uint32_t msgCountRange = 0;
static uint32_t msgCountFlow = 0;
static uint32_t validRangeCount = 0;
static uint32_t validFlowCount = 0;

// Settings
static bool useFlowDisabled = false;
static bool useRangeDisabled = false;
static float flowStdFixed = 1.5f;
static float flowScale = 1.0f;  // Scaling factor for flow measurements (tune if drift occurs)

/**
 * CRC8 DVB-S2 calculation for MSPv2
 */
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

/**
 * Parse MSPv2 message from byte stream
 * Returns true when a complete valid message has been received
 */
static bool mspParseChar(msp_msg_t* msg, uint8_t data)
{
    switch (msg->state) {
        case MSP_STATE_IDLE:
            if (data == MSP_HEADER_DOLLAR) {
                msg->state = MSP_STATE_HEADER_X;
            }
            break;

        case MSP_STATE_HEADER_X:
            if (data == MSP_HEADER_X) {
                msg->state = MSP_STATE_HEADER_DIR;
            } else {
                msg->state = MSP_STATE_IDLE;
            }
            break;

        case MSP_STATE_HEADER_DIR:
            if (data == MSP_HEADER_DIR_FROM) {
                msg->state = MSP_STATE_FLAG;
                msg->crc_calc = 0;  // Start CRC calculation
            } else {
                msg->state = MSP_STATE_IDLE;
            }
            break;

        case MSP_STATE_FLAG:
            msg->flag = data;
            msg->crc_calc = crc8_dvb_s2(msg->crc_calc, data);
            msg->state = MSP_STATE_FUNC_LO;
            break;

        case MSP_STATE_FUNC_LO:
            msg->function = data;
            msg->crc_calc = crc8_dvb_s2(msg->crc_calc, data);
            msg->state = MSP_STATE_FUNC_HI;
            break;

        case MSP_STATE_FUNC_HI:
            msg->function |= (uint16_t)data << 8;
            msg->crc_calc = crc8_dvb_s2(msg->crc_calc, data);
            msg->state = MSP_STATE_LEN_LO;
            break;

        case MSP_STATE_LEN_LO:
            msg->payload_len = data;
            msg->crc_calc = crc8_dvb_s2(msg->crc_calc, data);
            msg->state = MSP_STATE_LEN_HI;
            break;

        case MSP_STATE_LEN_HI:
            msg->payload_len |= (uint16_t)data << 8;
            msg->crc_calc = crc8_dvb_s2(msg->crc_calc, data);
            msg->payload_idx = 0;
            if (msg->payload_len == 0) {
                msg->state = MSP_STATE_CRC;
            } else if (msg->payload_len > MSP_MAX_PAYLOAD_LEN) {
                msg->state = MSP_STATE_IDLE;
            } else {
                msg->state = MSP_STATE_PAYLOAD;
            }
            break;

        case MSP_STATE_PAYLOAD:
            msg->payload[msg->payload_idx++] = data;
            msg->crc_calc = crc8_dvb_s2(msg->crc_calc, data);
            if (msg->payload_idx >= msg->payload_len) {
                msg->state = MSP_STATE_CRC;
            }
            break;

        case MSP_STATE_CRC:
            msg->crc = data;
            msg->state = MSP_STATE_IDLE;
            if (msg->crc_calc == msg->crc) {
                return true;  // Valid message!
            }
            break;

        default:
            msg->state = MSP_STATE_IDLE;
            break;
    }

    return false;
}

/**
 * Process rangefinder message
 * Payload format (5 bytes): quality(1) + distance(4)
 */
static void processRangefinderMessage(msp_msg_t* msg)
{
    if (msg->payload_len < 5) {
        return;
    }

    msp_rangefinder_payload_t payload;
    memcpy(&payload, msg->payload, sizeof(payload));

    lastRangeQuality = payload.quality;
    lastDistance = payload.distance_mm;

    msgCountRange++;

    // Process range measurement if valid
    if (!useRangeDisabled && payload.quality > 0 && payload.distance_mm > 0 && 
        payload.distance_mm < RANGE_OUTLIER_LIMIT) {
        float distance = (float)payload.distance_mm * 0.001f;  // mm to m
        float stdDev = expStdA * (1.0f + expf(expCoeff * (distance - expPointA)));

        rangeSet(rangeDown, distance);
        rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
        validRangeCount++;
    }
}

/**
 * Process optical flow message
 * Payload format (9 bytes): quality(1) + motion_x(4) + motion_y(4)
 */
static void processOpticalFlowMessage(msp_msg_t* msg, uint64_t* lastTime)
{
    if (msg->payload_len < 9) {
        return;
    }

    msp_opflow_payload_t payload;
    memcpy(&payload, msg->payload, sizeof(payload));

    lastFlowQuality = payload.quality;
    lastFlowX = payload.motion_x;
    lastFlowY = payload.motion_y;

    msgCountFlow++;

    // Process flow measurement if valid
    if (!useFlowDisabled && payload.quality >= MIN_FLOW_QUALITY) {
        if (abs(payload.motion_x) < FLOW_OUTLIER_LIMIT && 
            abs(payload.motion_y) < FLOW_OUTLIER_LIMIT) {
            
            flowMeasurement_t flowData;
            flowData.stdDevX = flowStdFixed;
            flowData.stdDevY = flowStdFixed * 2;
            flowData.dt = (float)(usecTimestamp() - *lastTime) / 1000000.0f;
            *lastTime = usecTimestamp();

            // Flip motion information to comply with sensor mounting
            // Same transformation as flowdeck: dpixelx = -deltaY, dpixely = -deltaX
            // Apply scaling factor for tuning
            flowData.dpixelx = flowScale * (float)(-payload.motion_y);
            flowData.dpixely = flowScale * (float)(-payload.motion_x);

            // Store for logging
            lastDpixelX = flowData.dpixelx;
            lastDpixelY = flowData.dpixely;
            lastFlowDt = flowData.dt;

            estimatorEnqueueFlow(&flowData);
            validFlowCount++;
        }
    }
}

void mtf02Task(void* arg)
{
    systemWaitStart();

    static msp_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    uint64_t lastTime = usecTimestamp();

    while (1) {
        uint8_t data;
        if (uart1GetDataWithTimeout(&data, M2T(10))) {
            if (mspParseChar(&msg, data)) {
                // Valid message received
                switch (msg.function) {
                    case MSP_FUNC_RANGEFINDER:
                        processRangefinderMessage(&msg);
                        break;

                    case MSP_FUNC_OPFLOW:
                        processOpticalFlowMessage(&msg, &lastTime);
                        break;

                    default:
                        break;
                }
            }
        }
    }
}

void mtf02Init(DeckInfo* info)
{
    if (isInit) {
        return;
    }

    uart1Init(MTF02_BAUDRATE);

    // Pre-compute constant for the measurement noise model
    expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

    xTaskCreate(mtf02Task, MTF02_TASK_NAME, MTF02_TASK_STACKSIZE, NULL,
                MTF02_TASK_PRI, NULL);

    DEBUG_PRINT("MTF-02 deck initialized (UART1 @ %d baud)\n", MTF02_BAUDRATE);
    isInit = true;
}

bool mtf02Test(void)
{
    return isInit;
}

static const DeckDriver mtf02Deck = {
    .vid = 0,
    .pid = 0,
    .name = "bcMTF02",
    .requiredEstimator = StateEstimatorTypeKalman,

    .usedGpio = 0,
    .usedPeriph = DECK_USING_UART1,

    .init = mtf02Init,
    .test = mtf02Test,
};

DECK_DRIVER(mtf02Deck);

/**
 * Logging variables
 */
LOG_GROUP_START(mtf02)
  // Raw sensor values
  LOG_ADD(LOG_UINT32, distance, &lastDistance)
  LOG_ADD(LOG_INT32, flowX, &lastFlowX)
  LOG_ADD(LOG_INT32, flowY, &lastFlowY)
  LOG_ADD(LOG_UINT8, flowQual, &lastFlowQuality)
  LOG_ADD(LOG_UINT8, rangeQual, &lastRangeQuality)
  // Values sent to EKF (after axis transform)
  LOG_ADD(LOG_FLOAT, dpixelX, &lastDpixelX)
  LOG_ADD(LOG_FLOAT, dpixelY, &lastDpixelY)
  LOG_ADD(LOG_FLOAT, flowDt, &lastFlowDt)
  // Counters
  LOG_ADD(LOG_UINT32, rangeCnt, &msgCountRange)
  LOG_ADD(LOG_UINT32, flowCnt, &msgCountFlow)
  LOG_ADD(LOG_UINT32, validRange, &validRangeCount)
  LOG_ADD(LOG_UINT32, validFlow, &validFlowCount)
LOG_GROUP_STOP(mtf02)

/**
 * Parameter variables
 */
PARAM_GROUP_START(mtf02)
  PARAM_ADD(PARAM_UINT8, flowDisable, &useFlowDisabled)
  PARAM_ADD(PARAM_UINT8, rangeDisable, &useRangeDisabled)
  PARAM_ADD(PARAM_FLOAT, flowStd, &flowStdFixed)
  PARAM_ADD(PARAM_FLOAT, flowScale, &flowScale)  // Scaling factor for flow (default 1.0)
PARAM_GROUP_STOP(mtf02)
