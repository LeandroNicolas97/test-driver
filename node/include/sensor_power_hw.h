/**
 *  \file sensor_power_hw.h
 *  \brief External sensors' power control.
 *
 *  \author Pablo Santamarina Cuneo
 *
 *  Copyright 2021 Innovex Tecnologias Ltda. All rights reserved.
 */

#ifndef SENSOR_POWER_HW_H
#define SENSOR_POWER_HW_H

/**
 * Initialize the sensors' power control
 */
int sensor_power_init(void);

/**
 * Turn the power of the external sensors ON.
 */
void sensor_power_on(int external_voltage);

/**
 * Turn the power of the external sensors OFF.
 */
void sensor_power_off(int external_voltage);

#endif /* SENSOR_POWER_HW_H */
