## Config for Flapper Drone Nimble+
###################################################

## Setup for Bolt
POWER_DISTRIBUTION = nimble_FD_PCB

CFLAGS += -DMOTOR_SETUP_NIMBLE
CFLAGS += -DENABLE_PWM_EXTENDED
CFLAGS += -DPID_FILTER_ALL

CFLAGS += -DIMU_PHI=0.0f
CFLAGS += -DIMU_THETA=90.0f
CFLAGS += -DIMU_PSI=180.0f

# CFLAGS += -DDECK_FORCE=bcUSD:bcCPPM

# CFLAGS += -DDECK_FORCE=bcUSD # force the SD card deck
CFLAGS += -DDECK_FORCE=bcCurrentDeck

# CFLAGS += -DIMPROVED_BARO_Z_HOLD
# CFLAGS += -DNIMBLE_MAX_THRUST 53000.0f

# CFLAGS += -DDECK_FORCE=bcCPPM # force the CCPM deck
# CFLAGS += -DCPPM_USE_PB8 # use alternative pins for CPPM: PB4, PB5 (not yet working) or PB8
# CFLAGS += -DEXTRX_ALT_HOLD # enable altitude hold with external Rx
# CFLAGS += -DEXTRX_TAER # TAER channel mapping (default is AETR - Aileron/Roll, Elevator/Pitch, Thrust, Rudder/Yaw)

# CFLAGS += -DDECK_FORCE=bcLedRing
# CFLAGS += -DTURN_OFF_LED
# CFLAGS += -DLEDRING_DEFAULT_EFFECT=19
# CFLAGS += -DLED_RING_NBR_LEDS=20

# CFLAGS += -DDECK_FORCE=bcUSD:bcLedRing # force the SD card deck and the LED Ring deck

###############################################
