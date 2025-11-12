
#ifndef _ACTUAL_CONDITIONS_H
#define _ACTUAL_CONDITIONS_H

#include "measurement.h"
#include "bsp-config.h"

#define WAKE  1
#define SLEEP 0
/**
 * The actual state of the device.
 * All state information should be packed here.
 */
struct oxycontroller_state {
    uint32_t warm_starts;          /* Number of warm starts */
    uint32_t total_scans;          /* Number of scans that we have performed */
    uint8_t calib_pressed;         /* How many seconds has the calibration button been pressed */
    uint16_t quick_response;       /* Number of times to respond quickly */
    uint8_t link_quality;          /* The current link quality */
    uint8_t n_of_sensors_detected; /* Number of sensors detected during start up */
    uint8_t display_on;            /* True to keep the display ON */
    uint8_t has_solenoid_control;  /* True if the solenoid control (i2C chip) is connected */
    uint8_t start_testing;         /* XXX */
    uint32_t uptime_at_last_ping;
    uint8_t can_sleep; /* Indicate the device can sleep immediately */
    uint8_t coordinator_found;
    uint32_t missed_conection;     /* Number of NAKs since last confirmed reception */
    uint8_t using_external_memory; /* True if we are using an external memory */
};

/**
 * Actual measurements.
 * This is a list of all the actual measurements of the attached sensors.
 */
extern struct measurement actual_measurements[MAX_EXTERNAL_SENSORS + 1 + MAX_N_VALVES];

extern struct oxycontroller_state actual_state;

#endif /* end of include guard: _ACTUAL_CONDITIONS_H */
