#ifndef __LPS_TWR_TAG_H__
#define __LPS_TWR_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"
#include "mac.h"

#define LPS_TWR_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWR_ANSWER 0x02
#define LPS_TWR_FINAL 0x03
#define LPS_TWR_REPORT 0x04 // Report contains all measurement from the anchor

#define LPS_TWR_TYPE 0
#define LPS_TWR_SEQ 1

extern uwbAlgorithm_t uwbTwrTagAlgorithm;

// Update the report payload - reduce distToPeers array
typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];
  uint16_t reciprocalDistance;
  float selfX;
  float selfY;
  float selfGz;
  float selfh;
  float selfVx;
  float selfVy;
  float selfVz;
  bool keep_flying;
  uint8_t auxMask;
  uint16_t distToPeers[3];  // Back to [3] - simpler to just index by peer ID
} __attribute__((packed)) lpsTwrTagReportPayload_t;

bool twrGetSwarmInfo(int robNum, uint16_t* range, float* x, float* y, float* gyroZ, float* height, float* vx, float* vy, float* vz);
bool command_share(int RobIDfromControl, bool keep_flying);

typedef struct {
  const uint64_t antennaDelay;
  const int rangingFailedThreshold;

  locoAddress_t tagAddress;
  const locoAddress_t anchorAddress[LOCODECK_NR_OF_TWR_ANCHORS];

   // TWR data
  point_t anchorPosition[LOCODECK_NR_OF_TWR_ANCHORS];
  bool combinedAnchorPositionOk;

  // TWR-TDMA options
  bool useTdma;
  int tdmaSlot;
} lpsTwrAlgoOptions_t;

#define TWR_RECEIVE_TIMEOUT 1000

uint16_t twrGetIndirectDist(uint8_t peerJ, uint8_t peerK);

// Only define if not already defined elsewhere
#ifndef LOCODECK_NR_OF_TWR_ANCHORS
#define LOCODECK_NR_OF_TWR_ANCHORS 3
#endif

#endif // __LPS_TWR_TAG_H__