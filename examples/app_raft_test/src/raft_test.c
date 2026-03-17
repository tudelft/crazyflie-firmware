#include <string.h>
#include "FreeRTOS.h"
#include "debug.h"
#include "task.h"
#include "app.h"
#include "raft.h"

static TaskHandle_t raftTaskHandle;
static Raft_Node_t *raftNode;

static void testProposeEmptyLog() {
  bool success = true;
  uint16_t requestId = 0;
  while (1) {
    if (success) {
      requestId = raftProposeNew(RAFT_LOG_COMMAND_RESERVED, NULL, 0);
      success = false;
    } else {
      raftProposeRetry(requestId, RAFT_LOG_COMMAND_RESERVED, NULL, 0);
    }
    success = raftProposeCheck(requestId, 2000);
    DEBUG_PRINT("raftTask: proposed requestId = %u, success = %d.\n",
                requestId,
                success
    );
    if (success) {
      break;
    }
  }
}

static void testProposeMemeberAdd(UWB_Address_t newMember) {
  bool success = true;
  uint16_t requestId = 0;
  while (1) {
    if (success) {
      requestId = raftProposeNew(RAFT_LOG_COMMAND_CONFIG_ADD, (uint8_t * ) & newMember, 2);
      success = false;
    } else {
      raftProposeRetry(requestId, RAFT_LOG_COMMAND_CONFIG_ADD, (uint8_t * ) & newMember, 2);
    }
    success = raftProposeCheck(requestId, 2000);
    DEBUG_PRINT("testProposeMemeberAdd: proposed requestId = %u, success = %d.\n",
                requestId,
                success
    );
    printRaftConfig(raftNode->config);
    if (success) {
      break;
    }
  }
}

static void testProposeMemeberRemove(UWB_Address_t newMember) {
  bool success = true;
  uint16_t requestId = 0;
  while (1) {
    if (success) {
      requestId = raftProposeNew(RAFT_LOG_COMMAND_CONFIG_REMOVE, (uint8_t * ) & newMember, 2);
      success = false;
    } else {
      raftProposeRetry(requestId, RAFT_LOG_COMMAND_CONFIG_REMOVE, (uint8_t * ) & newMember, 2);
    }
    success = raftProposeCheck(requestId, 2000);
    DEBUG_PRINT("testProposeMemeberRemove: proposed requestId = %u, success = %d.\n",
                requestId,
                success
    );
    printRaftConfig(raftNode->config);
    if (success) {
      break;
    }
  }
}

static void raftTestTask() {
  testProposeEmptyLog();
  testProposeMemeberRemove(4);
  testProposeMemeberAdd(4);
  while (1) {
    DEBUG_PRINT("raftTestTask: Done!\n");
    vTaskDelay(M2T(3000));
  }
}

void appMain() {
  raftNode = getGlobalRaftNode();
  xTaskCreate(raftTestTask, "RAFT_TEST", UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &raftTaskHandle);
}
