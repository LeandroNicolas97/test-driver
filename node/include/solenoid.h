/***************************************************************************
 *   file                 : solenoid.h                                     *
 *   begin                : 10 feb. 2016                                   *
 *   copyright            : (C) 2016 by Innovex Tecnologias Ltda.          *
 *   email                : pablo@innovex.cl                               *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2016. Innovex.                                          *
 ***************************************************************************/

#ifndef SOLENOID_H
#define SOLENOID_H

/**
 * Status of the solenoid operations
 */
enum solenoid_status {
    SOLENOID_OK,
    SOLENOID_NOT_DETECTED,
    SOLENOID_POWER_NOT_DETECTED,
    SOLENOID_INVALID_VALUE,
    SOLENOID_POWER_LOW,
    SOLENOID_DISCONNECTED,
    SOLENOID_SHORT_CIRCUIT
};

/**
 * Initialize the solenoid controller
 */
enum solenoid_status solenoid_init(void);

/**
 * Activate the solenoid in the forward direction. For a normal (non bi-stable) solenoid
 * this means powering the coil.
 */
enum solenoid_status solenoid_activate_forward(int solenoid_nr);

/**
 * Activate the specified solenoid in the reverse position
 * This is only valid for bi-stable solenoids
 */
enum solenoid_status solenoid_activate_reverse(int solenoid_nr);

/**
 * Release the solenoid. Disconnect all power from the specified solenoid.
 */
enum solenoid_status solenoid_release(void);

/**
 * TODO Maybe this is not needed.
 * Enable the boost power supply for the solenoid.
 * After powering the supply, one has to wait until the capacitor has been charged.
 */
enum solenoid_status solenoid_power_on(void);

/**
 * Disable the boost power supply for the solenoid.
 */
enum solenoid_status solenoid_power_off(void);

/**
 * Prepare the solenoid. This is important for bi-stable solenoids
 * which are activated with the charge of a capacitor.
 */
enum solenoid_status solenoid_prepare(void);

/**
 * Read the supply voltage of the solenoid in mV.
 */
int solenoid_read_supply_mv(void);

#endif /* SOLENOID_H */