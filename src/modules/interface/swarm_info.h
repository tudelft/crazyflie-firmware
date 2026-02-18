/**
 * Simple pub/sub for latest swarm telemetry values to be sent over UWB.
 * Produced by stabilizer (single writer), consumed by UWB modules (readers).
 */
#pragma once

#include <stdint.h>

typedef struct {
  float x;  // m (position x)
  float y;  // m (position y)
  float gz; // deg/s (yaw rate)
  float h;  // m (height / z position)
  float vx; // m/s (velocity x)
  float vy; // m/s (velocity y)
  float vz; // m/s (velocity z)
} swarm_info_t;

// Called from the stabilizer loop (single writer)
void swarmInfoUpdate(float x, float y, float gz, float h, float vx, float vy, float vz);

// Called from other modules (read latest snapshot)
void swarmInfoGet(float* x, float* y, float* gz, float* h, float* vx, float* vy, float* vz);
