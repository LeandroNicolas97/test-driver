/***************************************************************************
 *   file                 : defaults.h                                     *
 *   begin                : Oct 10, 2011                                   *
 *   copyright            : (C) 2011 by Innovex Tecnologias Ltda.          *
 *   email                : pablo@innovex.cl                               *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2011. Innovex.                                          *
 ***************************************************************************/

/*
 * Hardware configuration for the END_DEVICES
 */

#ifndef _DEFAULTS_H
#define _DEFAULTS_H

#define MAX_N_SENSORS 10 /* Maximum 10 sensors attached to this device */

/* Network parameters */
#define DEFAULT_PAN_ID      0XCAFE /* Default PAN-ID */
#define COORDINATOR_ADDRESS 0x0000 /* Short address of the pan-coordinator */

/** Default values for sampling and transmission */
#define DEFAULT_SAMPLING_INTERVAL 30 /* Seconds */
#define DEFAULT_TX_SLOT           0
#define DEFAULT_LOG_INTERVAL      300 /* 5 minutes */

/** Time to have the calibration button pressed to start a calibration */
#define TIME_TO_START_CALIBRATION 5

/** Default values for alarm limits */
#define DEFAULT_LOW_OXYGEN_ALARM_LEVEL             5.0   /* Below this level, ring low oxygen alarm */
#define DEFAULT_HIGH_OXYGEN_ALARM_LEVEL            12.0  /* Above this level, ring high oxygen alarm */
#define DEFAULT_OXYGEN_OPEN_LEVEL                  6.0   /* Below this level, open the oxygen flow */
#define DEFAULT_OXYGEN_CLOSE_LEVEL                 9.0   /* Above this level, close the oxygen flow */
#define DEFAULT_LOW_OXYGEN_SATURATION_ALARM_LEVEL  80.0  /* Below this level, ring low oxygen alarm */
#define DEFAULT_HIGH_OXYGEN_SATURATION_ALARM_LEVEL 120.0 /* Above this level, ring high oxygen alarm */
#define DEFAULT_OXYGEN_SATURATION_OPEN_LEVEL       90.0  /* Below this level, open the oxygen flow */
#define DEFAULT_OXYGEN_SATURATION_CLOSE_LEVEL      95.0  /* Above this level, close the oxygen flow */

/** Default salinity */
#define DEFAULT_SALINITY          0.0
/** Default temperature to use if temperature measurement is invalid */
#define DEFAULT_FIXED_TEMPERATURE 10.0

/**
 * Plausibility limits, if some of the measurements falls outside the ranges specified here,
 * they are marked as not plausible.
 */
#define PLAUSIBLE_MIN_TEMPERATURE 2.0
#define PLAUSIBLE_MAX_TEMPERATURE 32.0
#define PLAUSIBLE_MIN_OXYGEN      0.0
#define PLAUSIBLE_MAX_OXYGEN      20.0
#define PLAUSIBLE_MIN_DEPTH       0.0
#define PLAUSIBLE_MAX_DEPTH       60.0

/**
 * Sensors powering up time in ms
 * TODO This should be in the driver of every sensor.
 */
#define PONSEL_SENSORS_POWERUP_TIME             1000
#define INNOVEX_FLOW_SENSORS_POWERUP_TIME       50
#define INNOVEX_WAVE_SENSORS_POWERUP_TIME       15000
#define INNOVEX_WATER_FLOW_SENSORS_POWERUP_TIME 1000
#define YOSEMITECH_POWERUP_TIME                 22000 /* time to finish cleaning */
#define INNOVEX_SENSORS_POWERUP_TIME            DEFAULT_SENSOR_POWER_UP_TIME
#define AQUAS_SENSORS_POWERUP_TIME              3000
#define LUFFT_POWERUP_TIME                      60000
#define VAISALA_POWERUP_TIME                    10000
#define HUIZHONG_POWERUP_TIME                   6000
#define ANBSENSORS_POWERUP_TIME                 75000
#define XM126_POWERUP_TIME                      1000
#define CHEMINS_POWERUP_TIME                    4000
#define ST_VL53L1X_POWER_TIME                   1000

#define FRESHWATER 0
#define SEAWATER   1

#endif /* _DEFAULTS_H */
