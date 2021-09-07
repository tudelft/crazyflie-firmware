## Config for Flapper Drone Nimble+
###################################################

POWER_DISTRIBUTION = nimble_FD_PCB

# CFLAGS += -DNIMBLE_USE_CF2 % only when using CF2 as flight controller

CFLAGS += -DMOTOR_SETUP_NIMBLE
CFLAGS += -DENABLE_PWM_EXTENDED
CFLAGS += -DPID_FILTER_ALL

CFLAGS += -DIMU_PHI=0.0f
CFLAGS += -DIMU_THETA=90.0f
CFLAGS += -DIMU_PSI=180.0f

# CFLAGS += -DDECK_FORCE=bcUSD:bcCPPM:bcLedRing:bcCurrentDeck

# CFLAGS += -DDECK_FORCE=bcUSD # force the SD card deck
# CFLAGS += -DDECK_FORCE=bcCurrentDeck

# CFLAGS += -DIMPROVED_BARO_Z_HOLD
# CFLAGS += -DNIMBLE_MAX_THRUST 53000.0f


# CFLAGS += -DDECK_FORCE=bcCPPM # force the CCPM deck
# CFLAGS += -DCPPM_USE_PA2 # use alternative pins for CPPM: PA2(TX2), PA3(RX2), PB4(IO_3), PB5(IO_2) or PB8(IO_1)
# CFLAGS += -DEXTRX_ALT_HOLD # enable altitude hold with external Rx
# CFLAGS += -DEXTRX_ARMING # enable arming with external Rx (setup via "Brushless handling" compile flags)
# CFLAGS += -DEXTRX_TAER # use TAER channel mapping instead of the default AETR - Aileron(Roll), Elevator(Pitch), Thrust, Rudder(Yaw)

# CFLAGS += -DDECK_FORCE=bcLedRing
# CFLAGS += -DTURN_OFF_LED
# CFLAGS += -DLEDRING_DEFAULT_EFFECT=19
# CFLAGS += -DLED_RING_NBR_LEDS=20

# CFLAGS += -DDECK_FORCE=bcUSD:bcLedRing # force the SD card deck and the LED Ring deck

###############################################
