// Lightweight storage for swarm telemetry values
#include "swarm_info.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile swarm_info_t g_swarm_info = {0};

void swarmInfoUpdate(float x, float y, float gz, float h, float vx, float vy) {
  taskENTER_CRITICAL();
  g_swarm_info.x = x;
  g_swarm_info.y = y;
  g_swarm_info.gz = gz;
  g_swarm_info.h  = h;
  g_swarm_info.vx = vx;
  g_swarm_info.vy = vy;
  taskEXIT_CRITICAL();
}

void swarmInfoGet(float* x, float* y, float* gz, float* h, float* vx, float* vy) {
  taskENTER_CRITICAL();
  if (x)  *x  = g_swarm_info.x;
  if (y)  *y  = g_swarm_info.y;
  if (gz) *gz = g_swarm_info.gz;
  if (h)  *h  = g_swarm_info.h;
  if (vx) *vx = g_swarm_info.vx;
  if (vy) *vy = g_swarm_info.vy;
  taskEXIT_CRITICAL();
}
