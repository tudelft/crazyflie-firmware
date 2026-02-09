/**
 * Simple pub/sub for latest swarm telemetry values to be sent over UWB.
 * Produced by stabilizer (single writer), consumed by UWB modules (readers).
 */
#pragma once

#include <stdint.h>

typedef struct {
  float x; // m/s
  float y; // m/s
  float gz; // deg/s (yaw rate)
  float h;  // m (height / z position)
} swarm_info_t;

// Called from the stabilizer loop (single writer)
void swarmInfoUpdate(float x, float y, float gz, float h);

// Called from other modules (read latest snapshot)
void swarmInfoGet(float* x, float* y, float* gz, float* h);
