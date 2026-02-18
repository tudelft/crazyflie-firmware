#ifndef __RELATIVE_LOCALIZATION_H__
#define __RELATIVE_LOCALIZATION_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * Relative Localization EKF Module
 * 
 * Estimates relative position of peers in the swarm using UWB ranging
 * and shared velocity/gyro data.
 * 
 * Convention (from perspective of THIS drone):
 *   - State[j] = [x, y, z, psi] = position and heading of peer j in SELF's body frame
 *   - x: forward (positive = peer is in front)
 *   - y: left (positive = peer is to the left)
 *   - z: up (positive = peer is above)
 *   - psi: relative heading (positive = peer facing left relative to self)
 * 
 * Usage example in an app:
 * 
 *   void appMain() {
 *     relativeLocalizationInit();
 *     relativeLocalizationSetNumPeers(2);
 *     relativeLocalizationSetUseIndirect(true);  // Enable indirect measurements
 *     
 *     while(1) {
 *       vTaskDelay(M2T(10));  // 100 Hz
 *       
 *       relativeLocalizationUpdate(0.01f, true);  // dt=10ms, use indirect
 *       
 *       float state[4];
 *       if (relativeLocalizationGetState(0, state)) {
 *         // state[0..2] = relative position to peer 0
 *         // state[3] = relative heading to peer 0
 *       }
 *     }
 *   }
 */

// ============ INITIALIZATION ============

/**
 * Initialize the relative localization module
 * Call once at startup
 */
void relativeLocalizationInit(void);

/**
 * Reset all EKF states
 */
void relativeLocalizationReset(void);

/**
 * Reset EKF state for a specific peer
 */
void relativeLocalizationResetPeer(uint8_t peerId);

// ============ MAIN UPDATE ============

/**
 * Main update function - call this periodically (e.g., 100Hz)
 * 
 * This function:
 *   1. Gets self state from swarm_info
 *   2. Gets peer data from twrGetSwarmInfo()
 *   3. Runs EKF predict + update for each peer
 *   4. Optionally processes indirect measurements
 * 
 * @param dt Time step in seconds since last call
 * @param useIndirect If true, also process indirect measurements between peers
 */
void relativeLocalizationUpdate(float dt, bool useIndirect);

// ============ STATE ACCESS ============

/**
 * Get relative state of a peer
 * 
 * @param peerId Peer index (0 to numPeers-1)
 * @param state Output array of size 4: [x, y, z, psi]
 *              x, y, z in meters (position in self's body frame)
 *              psi in radians (relative heading)
 * @return true if peer is initialized and connected
 */
bool relativeLocalizationGetState(uint8_t peerId, float *state);

/**
 * Get covariance diagonal for a peer
 * 
 * @param peerId Peer index
 * @param cov Output array of size 4: [Pxx, Pyy, Pzz, Ppsi]
 * @return true if peer is initialized
 */
bool relativeLocalizationGetCovariance(uint8_t peerId, float *cov);

// ============ CONNECTION STATUS ============

/**
 * Check if any peer is connected
 * @return true if at least one peer is active
 */
bool relativeLocalizationIsConnected(void);

/**
 * Get bitmask of connected peers
 * @return Bitmask where bit n is set if peer n is connected
 */
uint8_t relativeLocalizationGetConnectedMask(void);

/**
 * Get number of currently connected peers
 * @return Count of connected peers
 */
uint8_t relativeLocalizationGetNumConnected(void);

// ============ CONFIGURATION ============

/**
 * Set number of peers to track (max 9)
 */
void relativeLocalizationSetNumPeers(uint8_t n);

/**
 * Enable/disable indirect measurements
 * When enabled, uses ranges between other peers (heard via UWB packets)
 * to improve localization accuracy
 */
void relativeLocalizationSetUseIndirect(bool enable);

#endif // __RELATIVE_LOCALIZATION_H__