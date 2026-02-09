#include "switch.h"
#define DEBUG_MODULE "AUX_SWITCH"
#include "debug.h"

#include <string.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "system.h"

#include "supervisor.h"


bool isInit = false;

logVarId_t idAux0, idAux1, idAux2, idAux3;
uint16_t aux_0, aux_1, aux_2, aux_3;


void switchInit(void)
{
  if (isInit)
    return;

  idAux0 = logGetVarId("cppm", "aux0");
  idAux1 = logGetVarId("cppm", "aux1");
  idAux2 = logGetVarId("cppm", "aux2");
  idAux3 = logGetVarId("cppm", "aux3");

  xTaskCreate(auxSwitchTask, SWITCH_TASK_NAME, SWITCH_TASK_STACKSIZE, NULL, SWITCH_TASK_PRI, NULL);
  isInit = true;
  DEBUG_PRINT("AUX switch task created\n");
}

void auxSwitchTask(void *arg)
{
  systemWaitStart();

  static uint32_t tick;
  while (1)
  {
    vTaskDelay(1);
    tick = xTaskGetTickCount();
    if (RATE_DO_EXECUTE(RATE_25_HZ, tick))
    {
      // Read aux channels
      aux_0 = logGetUint(idAux0);
      aux_1 = logGetUint(idAux1);
      aux_2 = logGetUint(idAux2);
      aux_3 = logGetUint(idAux3);

      if (auxConnected())
      {
      //   check if we need to arm/disarm
        if (auxState(3))
        {
          if (!supervisorIsArmed() && supervisorCanArm())
          {
            supervisorRequestArming(true);
            DEBUG_PRINT("Arming with AUX3\n");
          }
        }
        else if (supervisorIsArmed())
        {
          supervisorRequestArming(false);
          DEBUG_PRINT("Disarming with AUX3\n");
        }
      }
    }
  }
}

bool auxConnected(void)
{
  return aux_0 > 0 || aux_1 > 0 ||  aux_2 > 0 || aux_3 > 0;
}

bool auxState(uint8_t auxNumber)
{
  switch (auxNumber)
  {
  case 0:
    return (aux_0 > 0) && (aux_0 < 1400);
  case 1:
    return (aux_1 > 0) && (aux_1 < 1400);
  case 2:
    return (aux_2 > 0) && (aux_2 < 1400);
  case 3:
    return (aux_3 > 0) && (aux_3 < 1400);
  default:
    return false;
  }
}

void armWithAux(void)
{

}