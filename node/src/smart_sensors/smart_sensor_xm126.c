/***************************************************************************
 *   file                 : smart_sensor_xm126.c                           *
 *   begin                : Dec, 2024                                      *
 *   copyright            : (C) 2011 by Innovex Tecnologias Ltda.          *
 *   Author               : Pablo Alvarez                                  *
 *   email: pablo.alvarez@innovex.cl                                       *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2011. Innovex.                                          *
 ***************************************************************************/

/*
 * Communication with the smart sensors
 */

#include <math.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <sys/_stdint.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include "bsp-config.h"
#include "measurement.h"
#include "smart_sensor.h"
#include "serial.h"
#include "timeutils.h"
#include "debug.h"
#include "watchdog.h"

/* Driver configuration */
#define XM126_RESPONSE_TIMEOUT 5000 /* ms */
#define DETECTION_TRIES        3
#define MAX_SENSORS            2
#define MEASUREMENT_TIMEOUT    12000
#define MAX_RESPONSE_SIZE      60

/* XM126 command definitions as enum */
typedef enum {
    XM126_CMD_STOP_MEASURE = 0,
    XM126_CMD_START_MEASURE = 1,
    XM126_CMD_SENSOR_TYPE = 2
} xm126_cmd_t;

/**
 * Driver function prototypes
 */
static int max_sensors(void);
static const char *name(void);
static int init_driver(void);
static int finish_driver(void);
static int detect(int sensor_number, struct smart_sensor *sensor);
static int prepare(struct smart_sensor *sensor);
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m);

/* Local prototypes */
static int xm126_gets_with_timeout(int sensor_number, char *response, int size, uint32_t timeout);
static bool check_sensor_type(int sensor_number, struct smart_sensor *sensor, char *response, size_t response_size);
static int parse_distances(const char *response, float *distances, int *measurement_index, int max_size);
static int parse_velocity_and_distance(const char *response, float *velocities, float *distances,
                                       int *measurement_index, int max_size);
static void calculate_distances_statistics(const float *distances, int count, struct smart_sensor *sensor,
                                           struct measurement *measurement);
static void calculate_velocity_distance_statistics(const float *velocities, const float *distances, int count,
                                                   struct measurement *measurement, struct smart_sensor *sensor);
static int request_distance_measurement(struct smart_sensor *sensor, struct measurement *measurement);
static int request_velocity_distance_measurement(struct smart_sensor *sensor, struct measurement *measurement);
static void xm126_send_command(int sensor_number, uint8_t cmd);
static int request_measurement_by_type(struct smart_sensor *sensor, struct measurement *measurement);
static int needs_external_voltage(void);

/*
 * Smart sensor driver structure for the Acconeer XM126 radar
 */
const struct smart_sensor_driver smart_sensor_driver_xm126 = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .acquire = acquire,
    .name = name,
    .pass_command = NULL,
    .finish = NULL,
    .calibrate_zero = NULL,
    .calibrate_full = NULL,
    .needs_external_voltage = needs_external_voltage,
};

static int request_measurement_by_type(struct smart_sensor *sensor, struct measurement *measurement)
{
    if (sensor->type == RFLOW_SENSOR) {
        xm126_send_command(sensor->number, XM126_CMD_START_MEASURE);
        if (request_velocity_distance_measurement(sensor, measurement)) {
            return 1;
        }
    } else if (sensor->type == DISTANCE_SENSOR) {
        xm126_send_command(sensor->number, XM126_CMD_START_MEASURE);
        if (request_distance_measurement(sensor, measurement)) {
            return 1;
        }
    }
    return 0;
}
/**
 * Get a string from the sensor's UART with timeout
 */

static int xm126_gets_with_timeout(int sensor_number, char *response, int size, uint32_t timeout)
{
    uint8_t n = 0;
    char c;
    int ret;
    k_timepoint_t end = sys_timepoint_calc(K_MSEC(timeout));

    while (1) {
        watchdog_reset();
        if (sensor_number == 0) {
            ret = serial_getchar(UART_SMART_SENSOR);

        } else {
            ret = serial_getchar(RS232_PORT);
        }

        if (ret >= 0) {
            c = (char)ret; /* Only assign c when we get valid character */
            if (c == '\r') {
                break;
            }
            if (c == '\n') {
                continue; /* Skip newlines */
            }
            *response++ = c;
            n++;
            if (n >= (size - 2)) {
                break;
            }
        }
        if (sys_timepoint_expired(end)) {
            break;
        }
    }

    *response = '\0';
    return n;
}
/**
 * Send commands and receive responses from the XM126
 */
static void xm126_send_command(int sensor_number, uint8_t cmd)
{
    DEBUG("Sending command to XM126 on %s: %u\n", sensor_number == 0 ? "UART_SMART_SENSOR" : "iridium_port", cmd);

    /* Send the uint8_t command directly */
    watchdog_reset();
    if (sensor_number == 0) {
        serial_putchar(UART_SMART_SENSOR, cmd);
    } else {
        serial_putchar(RS232_PORT, cmd);
    }
}

/**
 * Get the maximum number of sensors this driver can handle
 */
static int max_sensors(void)
{
    return MAX_SENSORS;
}

/**
 * Get the name of the driver
 */
static const char *name(void)
{
    return "XM126";
}

/**
 * Initialize the driver
 */
static int init_driver(void)
{
    /* initialize iridium port */
    serial_set_baudrate(RS232_PORT, 115200);
    /* Configure UART_SMART_SENSOR */
    serial_set_baudrate(UART_SMART_SENSOR, 115200);

    return 0;
}

/**
 * Finish the driver
 */
static int finish_driver(void)
{
    serial_disable(UART_SMART_SENSOR);
    serial_disable(RS232_PORT);
    return 0;
}

/**
 * Prepare the sensor.
 */
static int prepare(struct smart_sensor *sensor)
{
    return 0;
}

static bool check_sensor_type(int sensor_number, struct smart_sensor *sensor, char *response, size_t response_size)
{
    xm126_send_command(sensor_number, XM126_CMD_SENSOR_TYPE);

    if (xm126_gets_with_timeout(sensor_number, response, response_size, XM126_RESPONSE_TIMEOUT) > 0) {
        sensor->number = sensor_number;
        DEBUG("XM126 DATA: %s\n", response);

        /* Determine sensor type based on response string */
        if (strstr(response, "Velocity") != NULL) {
            sensor->type = RFLOW_SENSOR;
            return true;
        } else if (strstr(response, "Distance") != NULL) {
            sensor->type = DISTANCE_SENSOR;
            return true;
        } else {
            DEBUG("Unknown sensor type response\n");
            return false;
        }
    }
    return false;
}
/**
 * Parse distances from the XM126.
 */
static int parse_distances(const char *response, float *distances, int *measurement_index, int max_size)
{
    int detected_count;
    float distance;

    /* First check if we have a valid "detected distances" message */
    if (sscanf(response, "%d detected distances", &detected_count) == 1) {
        if (detected_count == 0) {
            /* Handle case of "0 detected distances" */
            if (*measurement_index < max_size) {
                distances[*measurement_index] = 0.0f;
                (*measurement_index)++;
                return 1;
            }
        } else {
            /* Handle case of "X detected distances: X.XXXXXX m" */
            const char *distance_start = strstr(response, ": ");

            if (distance_start != NULL) {
                distance_start += 2; /* Skip ": " */
                if (sscanf(distance_start, "%f", &distance) == 1) {
                    if (*measurement_index < max_size) {
                        distances[*measurement_index] = distance;
                        (*measurement_index)++;
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}

/**
 * Parse velocity and distance from the XM126 response.
 */
static int parse_velocity_and_distance(const char *response, float *velocities, float *distances,
                                       int *measurement_index, int max_size)
{
    float velocity, distance;

    watchdog_reset();

    if (sscanf(response, "Velocity: %f m/s, distance: %f m", &velocity, &distance) == 2) {
        if (*measurement_index < max_size) {
            velocities[*measurement_index] = velocity;
            distances[*measurement_index] = distance;
            (*measurement_index)++;
            return 1;
        } else {
            printk("Max size reached (%d)\n", max_size);
        }
    }
    return 0;
}

/**
 * Calculate statistics from the data taken.
 */
static void calculate_distances_statistics(const float *distances, int count, struct smart_sensor *sensor,
                                           struct measurement *measurement)
{
    struct distance_measurement *m = &(measurement->distance);

    /* Initialize with default values indicating failure */
    m->min_distance = 0.0f;
    m->max_distance = 0.0f;
    m->mean_distance = 0.0f;
    m->min_distance_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->max_distance_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->mean_distance_status = MEASUREMENT_ACQUISITION_FAILURE;

    if (count == 0) {
        return;
    }

    float min = distances[0];
    float max = distances[0];
    float sum = 0;
    int valid_count = 0;

    for (int i = 0; i < count; i++) {
        if (distances[i] > 0) {
            if (valid_count == 0) {
                min = max = distances[i];
            } else {
                if (distances[i] < min) {
                    min = distances[i];
                }
                if (distances[i] > max) {
                    max = distances[i];
                }
            }
            sum += distances[i];
            valid_count++;
        }
    }

    /* Only update measurement values and status if we have valid measurements */
    if (valid_count > 0) {
        m->min_distance = min * 100;
        m->max_distance = max * 100;
        m->mean_distance = sum / valid_count * 100;
        m->min_distance_status = MEASUREMENT_OK;
        m->max_distance_status = MEASUREMENT_OK;
        m->mean_distance_status = MEASUREMENT_OK;
    }

    printk("Acconeer XM126 distances:\n");
    printk("Min distance: %.6f cm\n", (double)m->min_distance);
    printk("Max distance: %.6f cm\n", (double)m->max_distance);
    printk("Mean distance: %.6f cm\n", (double)m->mean_distance);
}

/**
 * Calculate statistics for both velocity and distance measurements.
 */
static void calculate_velocity_distance_statistics(const float *velocities, const float *distances, int count,
                                                   struct measurement *measurement, struct smart_sensor *sensor)
{
    struct radar_flow_measurement *flow = &(measurement->radar_flow);

    /* Initialize with default values indicating failure */
    flow->min_velocity = 0.0f;
    flow->max_velocity = 0.0f;
    flow->mean_velocity = 0.0f;
    flow->distance = 0.0f;
    flow->min_velocity_status = MEASUREMENT_ACQUISITION_FAILURE;
    flow->max_velocity_status = MEASUREMENT_ACQUISITION_FAILURE;
    flow->mean_velocity_status = MEASUREMENT_ACQUISITION_FAILURE;
    flow->distance_status = MEASUREMENT_ACQUISITION_FAILURE;

    if (count == 0) {
        printk("No measurements received\n");
        return;
    }

    /* Check for valid velocity values */
    int valid_vel_count = 0;
    float min_velocity = 0.0f;
    float max_velocity = 0.0f;
    float sum_vel = 0.0f;

    /* Find first valid velocity to initialize min/max */
    for (int i = 0; i < count; i++) {
        if (isfinite(velocities[i])) {
            valid_vel_count++;
            if (valid_vel_count == 1) {
                min_velocity = max_velocity = velocities[i];
                sum_vel = velocities[i];
            } else {
                if (velocities[i] < min_velocity) {
                    min_velocity = velocities[i];
                }
                if (velocities[i] > max_velocity) {
                    max_velocity = velocities[i];
                }
                sum_vel += velocities[i];
            }
        }
    }

    /* Check for valid distance values */
    int valid_dist_count = 0;
    float sum_dist = 0.0f;

    for (int i = 0; i < count; i++) {
        if (isfinite(distances[i]) && distances[i] > 0) {
            sum_dist += distances[i];
            valid_dist_count++;
        }
    }

    /* Update velocity measurements if valid data exists */
    if (valid_vel_count > 0) {
        flow->min_velocity = min_velocity * 100;
        flow->max_velocity = max_velocity * 100;
        flow->mean_velocity = (sum_vel / valid_vel_count) * 100;
        flow->min_velocity_status = MEASUREMENT_OK;
        flow->max_velocity_status = MEASUREMENT_OK;
        flow->mean_velocity_status = MEASUREMENT_OK;
    }

    /* Update distance measurement if valid data exists */
    if (valid_dist_count > 0) {
        flow->distance = (sum_dist / valid_dist_count) * 100;
        flow->distance_status = MEASUREMENT_OK;
    }

    /* Debug output */
    printk("\nVelocity Statistics:\n");
    printk("Max velocity: %.6f cm/s\n", (double)flow->max_velocity);
    printk("Min velocity: %.6f cm/s\n", (double)flow->min_velocity);
    printk("Mean velocity: %.6f cm/s\n", (double)flow->mean_velocity);

    printk("\nDistance Statistics:\n");
    printk("Mean distance: %.6f cm\n", (double)flow->distance);
}
/**
 * Request measurement from the XM126.
 */
static int request_distance_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    char response[MAX_RESPONSE_SIZE];
    float distances[MAX_RESPONSE_SIZE];
    int64_t start_time = get_uptime_ms();
    const int timeout_ms = 8000;
    int measurements_received = 0;

    serial_flush(sensor->number);

    /* Run for the full timeout duration */
    while ((get_uptime_ms() - start_time) <= timeout_ms) {
        watchdog_reset();

        if (xm126_gets_with_timeout(sensor->number, response, sizeof(response), 150) > 0) {
            if (strstr(response, "detected distances") != NULL) {
                /* Note: Modified to pass MAX_RESPONSE_SIZE directly since we're not tracking count */
                parse_distances(response, distances, &measurements_received, MAX_RESPONSE_SIZE);
            }
        } else {
            k_sleep(K_MSEC(10));
        }
    }

    /* Always send stop command */
    xm126_send_command(sensor->number, XM126_CMD_STOP_MEASURE);
    k_sleep(K_MSEC(100)); /* Give some time for the stop command to be processed */

    if (measurements_received > 0) {
        calculate_distances_statistics(distances, measurements_received, sensor, measurement);
        measurement->type = DISTANCE_SENSOR;
        measurement->sensor_status = SENSOR_OK;
        return 1;
    }

    printk("XM126 measurement timeout\n");
    return 0;
}

static int request_velocity_distance_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    char response[MAX_RESPONSE_SIZE];
    float velocities[MAX_RESPONSE_SIZE] = {0};
    float distances[MAX_RESPONSE_SIZE] = {0};
    int measurements_received = 0;
    int64_t start_time = get_uptime_ms();
    const int timeout_ms = 15000;

    serial_flush(sensor->number);

    watchdog_reset();
    k_sleep(K_MSEC(500));

    while ((get_uptime_ms() - start_time) <= timeout_ms) {
        watchdog_reset();

        int received = xm126_gets_with_timeout(sensor->number, response, sizeof(response), 200);

        if (received > 0) {
            if (parse_velocity_and_distance(
                    response, velocities, distances, &measurements_received, MAX_RESPONSE_SIZE)) {
                watchdog_reset();
                k_sleep(K_MSEC(50));
            }
        }

        k_sleep(K_MSEC(10));
    }

    /* Always send stop command, regardless of measurement success */
    xm126_send_command(sensor->number, XM126_CMD_STOP_MEASURE);
    k_sleep(K_MSEC(100)); /* Give some time for the stop command to be processed */

    if (measurements_received > 0) {
        calculate_velocity_distance_statistics(velocities, distances, measurements_received, measurement, sensor);
        measurement->type = RFLOW_SENSOR;
        measurement->sensor_status = SENSOR_OK;
        return 1;
    }

    return 0;
}

/**
 * Detect if an Acconeer sensor is present.
 * @param sensor_number The number of the sensor to check for
 * @param sensor A pointer to store the sensor information
 * @return 0 if OK, 1 on error.
 */
static int detect(int sensor_number, struct smart_sensor *sensor)
{
    char response[MAX_RESPONSE_SIZE];

    DEBUG("Checking XM126 sensor\n");

    watchdog_reset();
    k_sleep(K_MSEC(500));

    for (int tries = 0; tries < 10; tries++) {
        watchdog_reset();
        if (check_sensor_type(sensor_number, sensor, response, sizeof(response))) {

            /* Set common sensor properties */
            sensor->manufacturer = ACCONEER;
            sensor->power_up_time = 5000;
            sensor->channel = sensor_number;

            DEBUG("OK - Detected %s sensor\n", sensor->type == RFLOW_SENSOR ? "Velocity" : "Distance");
            return 1;
        }
    }

    DEBUG("NO - XM126 not detected\n");
    return 0;
}

/**
 * Acquire measurements from the sensor
 * @param tries The number of retries if there are problems with the sensor
 * @param sensor A pointer to store the sensor information
 * @param measurement A Pointer to store the result measurement
 * @return 1 if measurement was successfully acquired, 0 if all retries failed
 */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *measurement)
{
    while (tries > 0) {
        /* Flush ports before starting measurement */
        serial_flush(sensor->number);

        if (request_measurement_by_type(sensor, measurement)) {
            return 1;
        }

        tries--;
        if (tries > 0) {
            serial_flush(sensor->number);
            k_sleep(K_MSEC(700));
        }
    }
    return 0;
}

static int needs_external_voltage(void)
{
    return 0;
}
