#pragma once
#include "adc.h"
#include "bsp-config.h"
#include <stdint.h>

/**
 * Initialize the analog to digital converter
 */
void adc_init(void);

/**
 * Read the voltage of the battery in mV.
 */
int adc_read_battery(void);

/**
 * Read the supply voltage of the sensor in mV.
 * The Vcc of the external sensor is passed trough a 1/2 divider first and this voltage
 * is passed to the ADC.
 */
int adc_read_sensor_supply(void);

/**
 * Read the supply voltage of the solenoid in mV.
 * The solenoid supply is passed trough a 5.6M/560K voltage divider first,
 * dividing by 11, this goes to a follower and then to the ADC2.
 * We have to multiply by 11 to have mV.
 */
int adc_read_solenoid_supply(void);
