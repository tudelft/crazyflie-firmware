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
 * mtf02deck.h: MTF-02 optical flow and range sensor deck driver
 *
 * The MTF-02 outputs data in MSPv2 (Multiwii Serial Protocol v2) format.
 * It sends two message types:
 *   - Rangefinder (function 0x011F): 5-byte payload with distance
 *   - Optical Flow (function 0x021F): 9-byte payload with flow data
 */

#ifndef _MTF02_DECK_H_
#define _MTF02_DECK_H_

#include <stdint.h>
#include <stdbool.h>
#include "deck_core.h"

// MSPv2 Protocol definitions
// Header: $X< (0x24 0x58 0x3C) for sensor-to-FC direction
#define MSP_HEADER_DOLLAR        0x24  // '$'
#define MSP_HEADER_X             0x58  // 'X' (MSPv2)
#define MSP_HEADER_DIR_FROM      0x3C  // '<' (from peripheral to FC)
#define MSP_MAX_PAYLOAD_LEN      64

// MSP Function codes for MTF-02
// Observed from debug output: 0x1F01 (rangefinder) and 0x1F02 (optical flow)
// Wire format: 01 1F and 02 1F (little-endian) -> 0x1F01 and 0x1F02
#define MSP_FUNC_RANGEFINDER     0x1F01  // Rangefinder/ToF data (5-byte payload)
#define MSP_FUNC_OPFLOW          0x1F02  // Optical flow data (9-byte payload)

// MSPv2 parser states
typedef enum {
    MSP_STATE_IDLE = 0,
    MSP_STATE_HEADER_X,
    MSP_STATE_HEADER_DIR,
    MSP_STATE_FLAG,
    MSP_STATE_FUNC_LO,
    MSP_STATE_FUNC_HI,
    MSP_STATE_LEN_LO,
    MSP_STATE_LEN_HI,
    MSP_STATE_PAYLOAD,
    MSP_STATE_CRC
} msp_parser_state_t;

// MSPv2 message structure
typedef struct {
    uint8_t flag;
    uint16_t function;
    uint16_t payload_len;
    uint8_t payload[MSP_MAX_PAYLOAD_LEN];
    uint8_t crc;
    
    // Parser state
    msp_parser_state_t state;
    uint16_t payload_idx;
    uint8_t crc_calc;  // Running CRC calculation
} msp_msg_t;

// Rangefinder payload structure (5 bytes for function 0x011F)
// Based on observed: quality + 4-byte distance
#pragma pack(push, 1)
typedef struct {
    uint8_t quality;        // Signal quality (0xFF = valid)
    uint32_t distance_mm;   // Distance in mm
} msp_rangefinder_payload_t;
#pragma pack(pop)

// Optical flow payload structure (9 bytes for function 0x021F)
// Based on observed: quality + 4-byte flow_x + 4-byte flow_y (or similar)
#pragma pack(push, 1)
typedef struct {
    uint8_t quality;        // Flow quality
    int32_t motion_x;       // Motion/flow in X
    int32_t motion_y;       // Motion/flow in Y
} msp_opflow_payload_t;
#pragma pack(pop)

// Public function declarations
void mtf02Init(DeckInfo* info);
bool mtf02Test(void);
void mtf02Task(void* arg);

#endif /* _MTF02_DECK_H_ */