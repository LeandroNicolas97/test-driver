/**
 *  \file oxygen_contol.c
 *  \brief Control algorithm for the oxygen injection
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 21 december 2018
 *
 *  Copyright 2018 Innovex Tecnologias Ltda. All rights reserved.
 */
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "microio.h"
#include "defaults.h"
#include "led.h"
#include "display_fb.h"
#include "hardware.h"
#include "adc.h"
#include "solenoid.h"
#include "debug.h"
#include "oxygen_saturation.h"
#include "measurement.h"
#include "errorcodes.h"
#include "oxygen_control.h"
#include "watchdog.h"

/*
 * Local defines and constants
 */
#define MAX_SOLENOID_FINISH_VOLTAGE 7000 /* If the solenoid finishes higher than this, disconnected */
#define MIN_SOLENOID_FINISH_VOLTAGE 4000 /* If the solenoid finishes lower than this, short-circuit */

const struct valve_configuration default_valve_configuration = {
    .associated_sensor = 1,
    .injection_open_level = DEFAULT_OXYGEN_OPEN_LEVEL,
    .injection_close_level = DEFAULT_OXYGEN_CLOSE_LEVEL,
    .valve_type = VALVE_BISTABLE,
    .valve_number_of_pulses = 3,
    .injection_mode = INJECTION_DISABLED,
    .solenoid_pulse_length = 100000,
    .is_active = 0,
};

/**
 * The actual valve state
 */
struct valve_state {
    uint8_t valve_is_open;       /* Flag to notify that the valve is open */
    uint8_t valve_retries;       /* Number of times remaining to pulse the valve again for security */
    enum solenoid_status status; /* The status of the valve */
};

/*
 * Local prototypes
 */
static enum solenoid_status check_valve_finish_voltage(void);

/*
 * Valve configuration.
 * The configuration is in another place, we just keep a pointer to that place here.
 */
static struct valve_configuration *configuration_;

/*
 * Device state
 */
static struct valve_state actual_state[MAX_N_VALVES];

/**
 * Initialize the oxygen control sub-system.
 * @param configuration A Pointer to the structure with the valves' configuration
 */
void oxygen_control_init(struct valve_configuration *configuration)
{
    configuration_ = configuration;
}

/**
 * Get the configuration of the specified valve.
 */
static struct valve_configuration *cfg(int valve_nr)
{
    return &(configuration_[valve_nr]);
}

/**
 * Open the flow of oxygen
 */
enum solenoid_status open_oxygen_flow(int valve_nr)
{
    enum solenoid_status status = SOLENOID_OK;

    if (valve_nr < 0 || valve_nr >= MAX_N_VALVES) {
        status = SOLENOID_INVALID_VALUE;
        goto finish;
    }
    if (cfg(valve_nr)->valve_type == VALVE_NORMALLY_OPEN) {
        status = solenoid_release();
    } else if (cfg(valve_nr)->valve_type == VALVE_NORMALLY_CLOSE) {
        status = solenoid_activate_forward(valve_nr);
    } else if (cfg(valve_nr)->valve_type == VALVE_BISTABLE || cfg(valve_nr)->valve_type == VALVE_BISTABLE_INVERSE) {
        if (!actual_state[valve_nr].valve_is_open) {
            actual_state[valve_nr].valve_is_open = 1;
            actual_state[valve_nr].valve_retries = cfg(valve_nr)->valve_number_of_pulses;
        }
        if (actual_state[valve_nr].valve_retries > 0) {
            actual_state[valve_nr].valve_retries--;
            /* Send a pulse in the open direction */
            DEBUG("Opening valve\n");
            watchdog_disable();
            status = solenoid_prepare(); /* Here we get the status of the solenoid power supply */
            if (cfg(valve_nr)->valve_type == VALVE_BISTABLE) {
                solenoid_activate_forward(valve_nr);
            } else {
                solenoid_activate_reverse(valve_nr);
            }
            sleep_microseconds(cfg(valve_nr)->solenoid_pulse_length);
            /* Read the solenoid voltage just before release, only if it was already ok */
            if (status == SOLENOID_OK) {
                status = check_valve_finish_voltage();
            }
            solenoid_release();
            watchdog_init();
        }
    }
finish:
    return status;
}

/**
 * Close the flow of oxygen
 */
enum solenoid_status close_oxygen_flow(int valve_nr)
{
    watchdog_reset();
    enum solenoid_status status = SOLENOID_OK;

    if (valve_nr < 0 || valve_nr >= MAX_N_VALVES) {
        status = SOLENOID_INVALID_VALUE;
        goto finish;
    }
    if (cfg(valve_nr)->valve_type == VALVE_NORMALLY_OPEN) {
        status = solenoid_activate_forward(valve_nr);
    } else if (cfg(valve_nr)->valve_type == VALVE_NORMALLY_CLOSE) {
        status = solenoid_release();
    } else if (cfg(valve_nr)->valve_type == VALVE_BISTABLE || cfg(valve_nr)->valve_type == VALVE_BISTABLE_INVERSE) {
        if (actual_state[valve_nr].valve_is_open) {
            actual_state[valve_nr].valve_is_open = 0;
            actual_state[valve_nr].valve_retries = cfg(valve_nr)->valve_number_of_pulses;
        }
        if (actual_state[valve_nr].valve_retries > 0) {
            actual_state[valve_nr].valve_retries--;
            /* Send a pulse in the close direction */
            DEBUG("Closing valve\n");
            watchdog_disable();
            status = solenoid_prepare();
            if (cfg(valve_nr)->valve_type == VALVE_BISTABLE) {
                solenoid_activate_reverse(valve_nr);
            } else {
                solenoid_activate_forward(valve_nr);
            }
            sleep_microseconds(cfg(valve_nr)->solenoid_pulse_length);
            /* Read the solenoid voltage just before release, only if it was already ok */
            if (status == SOLENOID_OK) {
                status = check_valve_finish_voltage();
            }
            solenoid_release();
            watchdog_init();
        }
    }
finish:
    return status;
}

/**
 * Check if the oxygen levels for a valve are right and open or close the valve accordingly
 * Just consider the sensors which have a valve associated.
 */
void check_oxygen_levels(int valve_nr, int use_saturation, struct measurement *measurement)
{
    float actual_oxygen_level;

    if (valve_nr >= MAX_N_VALVES) {
        return;
    }

    if (cfg(valve_nr)->injection_mode == INJECTION_AUTO) {
        DEBUG("Injection mode: AUTO, checking oxygen levels..\n");
        if (measurement->sensor_status == SENSOR_OK) {
            if (use_saturation) {
                actual_oxygen_level = measurement->oxygen.saturation;
            } else {
                actual_oxygen_level = measurement->oxygen.concentration;
            }
            /* Check the injection levels */
            DEBUG("Measurement ok: %.2fmg/l\n", (double)actual_oxygen_level);
            if (actual_oxygen_level < cfg(valve_nr)->injection_open_level) {
                open_oxygen_flow(valve_nr);
            }
            if (actual_oxygen_level > cfg(valve_nr)->injection_close_level) {
                close_oxygen_flow(valve_nr);
            }
        } else {
            /* Problems with the sensor, show RED too */
            /* xxx  led_on(LED_RED);        */
            /* xxx   led_off(LED_GREEN);    */
        }
    } else if (cfg(valve_nr)->injection_mode == INJECTION_DISABLED) {
        DEBUG("Injection mode: DISABLED, ignoring oxygen levels..\n");
        if (actual_state[valve_nr].valve_is_open) {
            close_oxygen_flow(valve_nr);
        }
    } else if (cfg(valve_nr)->injection_mode == INJECTION_FORCE_ON) {
        DEBUG("Injection mode: FORCED_ON, ignoring oxygen levels..\n");
        if (!actual_state[valve_nr].valve_is_open) {
            open_oxygen_flow(valve_nr);
        }
    }
}

/**
 * Check if the oxygen level for every valve
 * @param use_saturation A Flag to tell if the check is using the saturation value
 * @param measurements A list with the actual measurements of all sensors.
 */
void check_oxygen_levels_all_valves(int use_saturation, struct measurement *measurements)
{
    struct measurement *sensor_measurement;

    for (int valve = 0; valve < MAX_N_VALVES; ++valve) {
        if (cfg(valve)->is_active) {
            sensor_measurement =
                &(measurements[cfg(valve)->associated_sensor]); /* TODO perhaps is better to use the sensor name */
            if (sensor_measurement->type == OXYGEN_SENSOR) {
                check_oxygen_levels(valve, use_saturation, sensor_measurement);
            }
        }
    }
}

/**
 * Set the configuration for a valve
 */
void valve_set_configuration(int valve_nr, struct valve_configuration *valve_configuration)
{
    *cfg(valve_nr) = *valve_configuration;
}

/**
 * Get the configuration for a valve
 */
void valve_get_configuration(int valve_nr, struct valve_configuration *valve_configuration)
{
    *valve_configuration = *cfg(valve_nr);
}

/**
 * Set the injection mode for all valves
 */
void valve_set_all_injection_mode(enum injection_modes injection_mode)
{
    for (int valve_nr = 0; valve_nr < MAX_N_VALVES; ++valve_nr) {
        valve_set_injection_mode(valve_nr, injection_mode);
    }
}

/**
 * Set the injection mode of this device
 */
void valve_set_injection_mode(int valve_nr, enum injection_modes injection_mode)
{
    cfg(valve_nr)->injection_mode = injection_mode;
}

/**
 * Get the injection mode of this device
 */
enum injection_modes get_injection_mode(int valve_nr)
{
    return cfg(valve_nr)->injection_mode;
}

/**
 * Set default values for all the configuration parameters
 * and close all the valves.
 * TODO This does not work correctly for normally closed valves.
 */
void valves_set_default_configuration(void)
{
    for (int valve_nr = 0; valve_nr < MAX_N_VALVES; ++valve_nr) {
        *cfg(valve_nr) = default_valve_configuration;
    }
}

/**
 * Detect all the bi-stable valves or relays and set the status of the corresponding valves
 * Set them all to close.
 * @return the number of valves detected.
 */
enum solenoid_status valve_detect_and_close(int valve_nr)
{
    enum solenoid_status status;

    actual_state[valve_nr].valve_is_open = 1; /* Assume valve may be open */
    actual_state[valve_nr].valve_retries = 0;
    do {
        status = close_oxygen_flow(valve_nr);
    } while (actual_state[valve_nr].valve_retries > 0);
    if (status == SOLENOID_DISCONNECTED) {
        status = SOLENOID_NOT_DETECTED;
    }
    actual_state[valve_nr].status = status;
    return status;
}

/**
 * Get the state of all the valves.
 * It is a bit mask. One bit per valve.
 * @return a bit mask indicating which valves are open
 */
int valves_in_open_state(void)
{
    int result = 0;

    for (int valve_nr = 0; valve_nr < MAX_N_VALVES; ++valve_nr) {
        if (actual_state[valve_nr].valve_is_open) {
            result |= (1 << valve_nr);
        }
    }
    return result;
}

/**
 * Get the state of the specific valve.
 * @return 1 if open, 0 if close.
 */
int valve_in_open_state(int valve_nr)
{
    return actual_state[valve_nr].valve_is_open;
}

static enum solenoid_status check_valve_finish_voltage(void)
{
    enum solenoid_status status = SOLENOID_OK;

    int solenoid_mv = adc_read_solenoid_supply();

    if (solenoid_mv > MAX_SOLENOID_FINISH_VOLTAGE) {
        status = SOLENOID_DISCONNECTED;
    } else if (solenoid_mv < MIN_SOLENOID_FINISH_VOLTAGE) {
        status = SOLENOID_SHORT_CIRCUIT;
    }
    return status;
}

/**
 * Detect a bi-stable valve and get the start and end voltage.
 * @param The valve number
 * @param vm_charged A Pointer to get the voltage of the solenoid supply when fully charged
 * @param vm_discharged A Pointer to get the voltage of the solenoid supply after activation
 * @return the status of the solenoid
 */
enum solenoid_status detect_valve(int valve_nr, int *mv_charged, int *mv_discharged)
{
    enum solenoid_status status;

    watchdog_reset();
    solenoid_prepare();
    *mv_charged = adc_read_solenoid_supply();
    solenoid_activate_forward(valve_nr);
    sleep_microseconds(cfg(valve_nr)->solenoid_pulse_length);
    *mv_discharged = adc_read_solenoid_supply();
    status = solenoid_release();
    watchdog_reset();
    solenoid_prepare();
    status = solenoid_activate_reverse(valve_nr);
    sleep_microseconds(cfg(valve_nr)->solenoid_pulse_length);
    solenoid_release();
    valve_detect_and_close(valve_nr); /* close all valves */
    return status;
}
