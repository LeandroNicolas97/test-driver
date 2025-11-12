/**
 *  \file measurement_operations.h
 *  \brief Operations with measurements
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 08.02.2019
 *
 *  Copyright 2019 Innovex Tecnologias Ltda. All rights reserved.
 */

#ifndef MEASUREMENT_OPERATIONS_H
#define MEASUREMENT_OPERATIONS_H

#include "measurement.h"

/**
 * Get a pointer to the measurements list
 */
struct measurement *actual_measurement(void);

/**
 * Check if two depths are almost the same.
 */
int almost_same_depth(float depth1, float depth2);

/**
 * Join the salinity measurement from a conductivity sensor at about the same depth
 * of an oxygen sensor.
 * @param n_of_measurements The number of measurements in the array to check
 * @param measurement An array with all the measurements to check.
 * @return the number of oxygen sensors with joined salinity measurements
 */
int measurements_join_oxygen_with_salinity(int n_of_measurements, struct measurement *measurement);

/**
 * Calculate the oxygen concentration for a measurement using the saturation
 * obtained from the optical measurement, the salinity and the temperature
 * The concentration can be calculated only if we have valid oxygen and temperature measurements.
 * @param measurement An array with all the measurements to check.
 * @return 1 if two sensors can be joined, 0 otherwise.
 */
void measurement_calculate_oxygen_concentration(struct measurement *measurement);

/**
 * Calculate the oxygen concentration for a list of measurement using the saturation
 * obtained from the optical measurement, the salinity and the temperature
 * The concentration can be calculated only if we have valid oxygen and temperature measurements.
 * @param n_of_measurements The number of measurements in the array to process
 * @param measurement An array with all the measurements to process.
 */
void measurements_list_calculate_oxygen_concentration(int n_of_measurements, struct measurement *measurement);

/*
 * Join two level measurements.
 * @param n_of_measurements The number of measurements in the array to check
 * @param measurement An array with all the measurements to check.
 * @return 1 if two sensors can be joined.
 */
int measurements_join_two_levels(int n_of_measurements, struct measurement *measurement);

/*
 * Join three current AC measurements.
 * @param n_of_measurements The number of measurements in the array to check
 * @param measurement An array with all the measurements to check.
 * @return 1 if two sensors can be joined.
 */
int measurements_join_current_ac(int n_of_measurements, struct measurement *measurement);

int gets_totalized_flow_measurement(int n_of_measurements, struct measurement *measurement);

/*
 * Average oil level
 */
int average_oil_level(int n_of_measurements, struct measurement *measurement);

#endif /* MEASUREMENT_OPERATIONS_H  */
