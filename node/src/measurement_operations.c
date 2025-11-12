/**
 *  \file measurement_operations.c
 *  \brief Operations with measurements
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 08.02.2019
 *
 *  Copyright 2019 Innovex Tecnologias Ltda. All rights reserved.
 */

#include <stdlib.h>
#include <math.h>
#include "debug.h"
#include "measurement.h"
#include "temperature.h"
#include "oxygen_saturation.h"
#include "measurement_operations.h"
#include "smart_sensor.h"
#include "configuration.h"
#include "watchdog.h"
#include "hardware.h"

/**
 * Range to decide that two sensors are at the same depth.
 * Two sensor are at the same depth if they are in +- this range
 */
#define DEPTH_RANGE_EQUAL 0.5f /* Range of +- 0.5m */

/**
 * Check if the depth is almost the same.
 */
int almost_same_depth(float depth1, float depth2)
{
    if (fabsf(depth1 - depth2) <= DEPTH_RANGE_EQUAL) {
        return 1;
    } else {
        return 0;
    }
}

/*
 * Join the salinity measurement from a conductivity sensor at about the same depth
 * of an oxygen sensor.
 * @param n_of_measurements The number of measurements in the array to check
 * @param measurement An array with all the measurements to check.
 * @return 1 if two sensors can be joined, 0 otherwise.
 */
int measurements_join_oxygen_with_salinity(int n_of_measurements, struct measurement *measurement)
{
    int sensors_at_same_depth = 0;
    /* Check if there are oxygen sensors */
    for (int i = 0; i < n_of_measurements; i++) {
        if (measurement[i].type == OXYGEN_SENSOR) {
            struct oxygen_measurement *oxygen = &(measurement[i].oxygen);

            DEBUG("Oxygen sensor (S%i), with depth: %.1f\n", i + 1, (double)oxygen->depth);
            for (int j = 0; j < n_of_measurements; j++) {
                if (measurement[j].type == CONDUCTIVITY_SENSOR) {
                    struct conductivity_measurement *conductivity = &(measurement[j].conductivity);

                    if (almost_same_depth(conductivity->depth, oxygen->depth)) {
                        /* Found two sensor at similar depth. Set the salinity of the oxygen measurement */
                        /* to the value measured by the conductivity sensor */
                        DEBUG("Salinity sensor (S%i), with depth: %.1f salinity %.2f\n",
                              j + 1,
                              (double)conductivity->depth,
                              (double)conductivity->salinity);
                        oxygen->salinity = conductivity->salinity;
                        oxygen->salinity_status = MEASUREMENT_OK;
                        sensors_at_same_depth = 1;
                        break;
                    }
                }
            }
        }
    }
    return sensors_at_same_depth;
}

/**
 * Calculate the oxygen concentration for a measurement using the saturation
 * obtained from the optical measurement, the salinity and the temperature
 * The concentration can be calculated only if we have a valid oxygen and temperature measurements.
 */
void measurement_calculate_oxygen_concentration(struct measurement *measurement)
{
    struct oxygen_measurement *oxygen = &(measurement->oxygen);

    DEBUG("Actual concentration %.2f\n", (double)oxygen->concentration);
    if (measurement->sensor_status == SENSOR_OK && oxygen->saturation_status == MEASUREMENT_OK &&
        oxygen->temperature_status == MEASUREMENT_OK) {
        DEBUG("Concentration calculated from saturation %.1f, salinity %.1f and temperature %.1f\n",
              (double)oxygen->saturation,
              (double)oxygen->salinity,
              (double)oxygen->temperature);
        oxygen->concentration = oxygen_concentration(oxygen->saturation, oxygen->salinity, kelvin(oxygen->temperature));
        DEBUG("New concentration %.2f\n", (double)oxygen->concentration);
    }
}

/**
 * Calculate the oxygen concentration for a list of measurement using the saturation
 * obtained from the optical measurement, the salinity and the temperature
 * The concentration can be calculated only if we have valid oxygen and temperature measurements.
 * @param n_of_measurements The number of measurements in the array to process
 * @param measurement An array with all the measurements to process.
 */
void measurements_list_calculate_oxygen_concentration(int n_of_measurements, struct measurement *measurement)
{
    /* Check if there are oxygen sensors */
    for (int i = 0; i < n_of_measurements; i++) {
        const struct smart_sensor *sensor = smart_sensor_get(i);

        if (sensor->manufacturer == INNOVEX) {
            if (measurement[i].type == OXYGEN_SENSOR) {
                measurement_calculate_oxygen_concentration(&(measurement[i]));
            }
        }
    }
}

/*
 * Join two level measurements.
 * @param n_of_measurements The number of measurements in the array to check
 * @param measurement An array with all the measurements to check.
 * @return 1 if two sensors can be joined.
 */
int measurements_join_two_levels(int n_of_measurements, struct measurement *measurement)
{
    int two_levels = 0;
    /* Check if there are oxygen sensors */
    for (int i = 0; i < n_of_measurements; i++) {
        if (measurement[i].type == LEVEL_SENSOR) {
            struct level_measurement *level_1 = &(measurement[i].level);

            DEBUG("Level 1 sensor (S%i): %.1f\n", i + 1, (double)level_1->level_1);
            for (int j = i + 1; j < n_of_measurements; j++) {
                if (measurement[j].type == LEVEL_SENSOR) {
                    struct level_measurement *level_2 = &(measurement[j].level);

                    DEBUG("Level 2 sensor (S%i): %.1f\n", j + 1, (double)level_2->level_1);
                    level_1->level_2 = level_2->level_1;
                    level_1->level_2_status = MEASUREMENT_OK;
                    two_levels = 1;
                    break;
                }
            }
        }
    }
    return two_levels;
}

int measurements_join_current_ac(int n_of_measurements, struct measurement *measurement)
{
    int phases = 0;

    for (int i = 0; i < n_of_measurements; i++) {
        struct current_ac_measurement *current = &(measurement[i].current_ac);

        for (int j = i + 1; j < n_of_measurements; j++) {
            if ((measurement[j].type == CURRENT_AC_SENSOR) && (phases == 0)) {
                struct current_ac_measurement *current_2 = &(measurement[j].current_ac);

                DEBUG("Current 2 (S: %i): %.1f\n", i + 1, (double)current_2->phase_1);
                current->phase_2 = current_2->phase_1;
                current->phase_2_status = MEASUREMENT_OK;
                phases++;
            }
            if ((measurement[j].type == CURRENT_AC_SENSOR) && (phases > 0)) {
                struct current_ac_measurement *current_3 = &(measurement[j].current_ac);

                DEBUG("Current 3 (S: %i): %.1f\n", i + 1, (double)current_3->phase_1);
                current->phase_3 = current_3->phase_1;
                current->phase_3_status = MEASUREMENT_OK;
                phases++;
            }
        }
        if (phases >= 2) {
            return 0;
        }
    }
    return 0;
}

int gets_totalized_flow_measurement(int n_of_measurements, struct measurement *measurement)
{
    float flow = 0;

    for (int i = 0; i < n_of_measurements; i++) {
        if (measurement[i].type == FLOW_WATER_SENSOR) {
            struct flow_water_measurement *flow_water = &(measurement[i].flow_water);

            if (flow_water->flow_water > flow) {
                flow = flow_water->flow_water;
            }
        }
        cfg.totalized_flow += (uint32_t)round(flow * cfg.sampling_interval * 0.001f);
        for (int i = 0; i < n_of_measurements; i++) {
            if (measurement[i].type == FLOW_WATER_SENSOR) {
                struct flow_water_measurement *flow_water = &(measurement[i].flow_water);

                flow_water->accumulated = cfg.totalized_flow;
                flow_water->flow_water = flow;
                flow_water->accumulated_status = MEASUREMENT_OK;
                flow_water->flow_water_status = MEASUREMENT_OK;
                int ret;

                ret = write_configuration();
                if (ret < 0) {
                    printk("Error guardando totalizador!\n");
                    return -1;
                }
                printk("Totalizador %i\n", cfg.totalized_flow);
            }
        }
    }
    return 0;
}

int average_oil_level(int n_of_measurements, struct measurement *measurement)
{
    uint16_t n_samples[2] = {0};
    uint32_t init_time = k_uptime_get();
    float accumulated_volume[2] = {0.0};
    float total_current_volume = 0;

    /* Takes measurements for 10 seconds. */
    while ((k_uptime_get() - init_time) <= 10000) {
        watchdog_reset();
        for (int i = 0; i < n_of_measurements; i++) {
            if (measurement[i].type == VOLUME_SENSOR && measurement[i].sensor_status == SENSOR_OK) {
                sleep_microseconds(10000);
                smart_sensors_aquire_all(n_of_measurements, 5, measurement);
                struct volume_measurement *volume = &(measurement[i].volume);

                n_samples[i]++;
                accumulated_volume[i] += volume->volume;
            }
        }
    }
    /* Gets average. */
    for (int i = 0; i < n_of_measurements; i++) {
        if (measurement[i].type == VOLUME_SENSOR && measurement[i].sensor_status == SENSOR_OK) {
            struct volume_measurement *volume = &(measurement[i].volume);

            volume->volume = (accumulated_volume[i] / n_samples[i]);
            total_current_volume += volume->volume;
        }
    }
    /* Gets porcentages. */
    for (int i = 0; i < n_of_measurements; i++) {
        if (measurement[i].type == VOLUME_SENSOR && measurement[i].sensor_status == SENSOR_OK) {
            struct volume_measurement *volume = &(measurement[i].volume);

            volume->porcentage = (total_current_volume / cfg.total_volume) * 100;
            volume->porcentage_status = MEASUREMENT_OK;
        }
    }
    return 0;
}
