/**
 *  \file userinterface.h
 *  \brief User interface
 *
 *  \author Pablo Santamarina Cuneo
 *
 *  Copyright 2016 Innovex Tecnologias Ltda. All rights reserved.
 */

#ifndef USERINTERFACE_H_
#define USERINTERFACE_H_

#include "measurement.h"

/**
 * A generic measurement to be displayed. The variables are classified in
 * levels.
 */
struct visual_measurement {
    float value;                    /* The value of the measurement */
    enum measurement_status status; /* The status of the measurement */
    char *units;                    /* The units as a string */
    char *format;                   /* The format to print the value */
};

/**
 * initialize the lcd, then clears it.
 */
void init_and_clear_lcd(void);

/**
 * Display a welcome message with the firmware version strings
 */
void display_welcome_message(void);

/* XXX What is this ??? */
void display_source_motes(uint8_t source_array[6]);

/**
 * Display all the measurements of all the active sensors
 * @param n_of_sensors_active The number of sensors active
 * @measurement An array with the measurements of all active sensors.
 */
void display_all_measurements(int n_of_sensors_active, struct measurement *measurement, uint8_t use_sat);

/**
 * Display a detailed information of a measurement.
 */
void display_measurement_detail(int sensor_number, char *name, struct measurement *measurement);

/**
 * Display the header of an associated node, showing channel, missed connections and link quality.
 */
void display_associated_header(int channel, int missed_connections, int link_quality, float bat_lvl);

/**
 * Display the header of an non-associated node.
 */
void display_not_associated_header(void);

/**
 * Display the footer with the node name and the pan-id.
 */
void display_footer(char *name, uint16_t pan_id);

/**
 * Display a "going to sleep" message indicating in how many seconds it will
 * go to sleep.
 */
void display_going_to_sleep(int seconds);

void display_end_device_status(float battery_voltage);

/**
 * Show the user a message and wait for the user to press the button. If the user
 * presses the button before the timeout, returns true, otherwise false.
 * @param message The message to display
 * @param seconds The time to wait for the user response
 * @return true if the user pressed the button before timeout
 */
int wait_for_user_response(const char *message, int seconds);

#endif /* USERINTERFACE_H_ */
