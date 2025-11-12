/**
 *  \file oxygen_contol.h
 *  \brief Control algorithm for the oxygen injection
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 21 december 2018
 *
 *  Copyright 2018 Innovex Tecnologias Ltda. All rights reserved.
 */

#ifndef OXYGEN_CONTROL_H
#define OXYGEN_CONTROL_H

#include "solenoid.h"
#include "measurement.h"

/**
 * The injection operation mode of a valve.
 * TODO is this used???
 */
enum injection_mode {
    INJECTION_MODE_OFF,
    INJECTION_MODE_ON,
    INJECTION_MODE_AUTO
};

/**
 * Oxygen control runtime configuration parameters.
 */
struct valve_configuration {
    int associated_sensor;               /* The sensor number associated with this valve */
    float injection_open_level;          /* If oxygen goes below this level, start the oxygen injection. */
    float injection_close_level;         /* If the oxygen concentration is above this level, stop injection. */
    enum valve_type valve_type;          /* Type of valve we have */
    uint8_t valve_number_of_pulses;      /* Number of pulses to give to a bi-stable valve to be sure it is activated */
    enum injection_modes injection_mode; /* Oxygen injection operation mode */
    uint32_t solenoid_pulse_length;      /* Length of the solenoid pulse in milliseconds */
    uint8_t is_active;                   /* True if the valve is active */
};

/** The default valve configuration */
extern const struct valve_configuration default_valve_configuration;

/**
 * Initialize the oxygen control sub-system.
 * @param configuration A Pointer to the structure with the valves' configuration
 */
void oxygen_control_init(struct valve_configuration *configuration);

/**
 * Open the flow of oxygen
 */
enum solenoid_status open_oxygen_flow(int valve_nr);

/**
 * Close the flow of oxygen
 */
enum solenoid_status close_oxygen_flow(int valve_nr);

/**
 * Check if the oxygen level for every valve
 * @param use_saturation A Flag to tell if the check is using the saturation value
 * @param measurements A list with the actual measurements of all sensors.
 */
void check_oxygen_levels_all_valves(int use_saturation, struct measurement *measurements);

/**
 * Set the configuration for a valve
 */
void valve_set_configuration(int valve_nr, struct valve_configuration *valve_configuration);

/**
 * Get the configuration for a valve.
 */
void valve_get_configuration(int valve_nr, struct valve_configuration *valve_configuration);

/**
 * Set the injection mode for all valves
 */
void valve_set_all_injection_mode(enum injection_modes injection_mode);

/**
 * Set the injection mode of this device
 */
void valve_set_injection_mode(int valve_nr, enum injection_modes injection_mode);

/**
 * Get the injection mode of this device
 */
enum injection_modes get_injection_mode(int valve_nr);

/**
 * Set default values for all the configuration parameters.
 */
void valves_set_default_configuration(void);

/**
 * Get the state of all the valves.
 * It is a bit mask. One bit per valve.
 * @return a bit mask indicating which valves are open
 */
int valves_in_open_state(void);

/**
 * Get the state of a specific valve.
 * @return 1 if open, 0 if close.
 */
int valve_in_open_state(int valve_nr);

/**
 * Detect all the bi-stable valves or relays and set the status of the corresponding valves
 * Set them all to close.
 * @return the number of valves detected.
 */
enum solenoid_status valve_detect_and_close(int valve_nr);

/**
 * Detect a bi-stable valve and get the start and end voltage.
 * @param The valve number
 * @param vm_charged A Pointer to get the voltage of the solenoid supply when fully charged
 * @param vm_discharged A Pointer to get the voltage of the solenoid supply after activation
 * @return the status of the solenoid
 */
enum solenoid_status detect_valve(int valve_nr, int *mv_charged, int *mv_discharged);

/**
 * Detect all the valves connected to this device
 */
void detect_all_valves(void);

#endif /* OXYGEN_CONTROL_H */
