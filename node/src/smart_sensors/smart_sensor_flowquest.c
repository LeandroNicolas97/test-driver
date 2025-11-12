/**
 *  \file smart_sensor_flowquest.c
 *  \brief Driver for connecting with Flowquest
 *
 *  \author Raúl Baigorri Morales
 *  \author Sebastián Angulo Oyarzún
 *
 *  \date 09.04.2025
 *
 *  Copyright 2025 Innovex Tecnologias Ltda. All rights reserved.
 */

#include "bsp-config.h"
#include "debug.h"
#include "flowquest.h"
#include "hardware.h"
#include "measurement.h"
#include "serial.h"
#include "smart_sensor.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "watchdog.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/state.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys_clock.h>
#include <zephyr/logging/log.h>
#include "adcp.h"

#define DETECTION_TRIES   4
#define MAX_N_SENSORS     1
#define MAX_RESPONSE_SIZE 2048
#define UART_BUFFER_SIZE  2048
#define MAX_LINE_LENGTH   2048
#define BUFFER_SIZE       2048

/**
 * Driver function prototypes
 */
static int max_sensors(void);                                      /* Maximum number of sensors for this driver */
static const char *name(void);                                     /* Name of the driver */
static int init_driver(void);                                      /* Initialize the driver */
static int finish_driver(void);                                    /* Finish the operations with the driver */
static int detect(int sensor_number, struct smart_sensor *sensor); /* Detect a sensor */
static int prepare(struct smart_sensor *sensor);                   /* Prepare the sensor */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m);
static int needs_external_voltage(void);
static int filter_invalid_chars; /*Global variable to enable or disable filtering*/

/**
 * Smart sensor driver structure for the FLOWQUEST sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_flowquest = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .finish = NULL,
    .calibrate_zero = NULL, /* This sensor don't requier calibration */
    .calibrate_full = NULL,
    .acquire = acquire,
    .pass_command = NULL,
    .name = name,
    .needs_external_voltage = needs_external_voltage,
};

/* Local prototypes */
static int send_command(const char *command, uint8_t *response, size_t size, uint32_t timeout);
static void transmit_command(const char *cmd, size_t size);
static int start_deployment(struct smart_sensor *sensor);
static int detect_config(struct smart_sensor *sensor);
static int gets_with_timeout(uint8_t *response, int size, uint32_t timeout);
const uint8_t *find_pattern(const uint8_t *haystack, size_t haystack_len, const uint8_t *needle, size_t needle_len);

const uint8_t *find_pattern(const uint8_t *haystack, size_t haystack_len, const uint8_t *needle, size_t needle_len)
{
    if (needle_len == 0 || haystack_len < needle_len) {
        return NULL;
    }

    for (size_t i = 0; i <= haystack_len - needle_len; i++) {
        size_t j = 0;

        while (j < needle_len && haystack[i + j] == needle[j]) {
            j++;
        }
        if (j == needle_len) {
            return &haystack[i];
        }
    }

    return NULL;
}

/*
 * Get the maximum number of sensors of this type this driver can handle
 */
static int max_sensors(void)
{
    return MAX_N_SENSORS;
}

/**
 * Get the name of the driver.
 */
static const char *name(void)
{
    return "Flowquest";
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    return 0;
}

/*
 * Prepare the driver
 */
static int prepare(struct smart_sensor *sensor)
{
    return 0;
}

/*
 * Finish the operation with the smart sensors
 */
static int finish_driver(void)
{
    return 0;
}

/**
 * Check for a single sensor with the specified number on the RS-485 bus
 * @param sensor_number The number of the sensor to check for.
 * @param measurement A pointer to return a measurement from the sensor
 * @param sensor A pointer to store the sensor information
 * @return True if a sensor was detected
 */
static int detect(int sensor_number, struct smart_sensor *sensor)
{
    int tries;

    DEBUG("Checking FLOWQUEST %i...\n", sensor_number);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        if (detect_config(sensor) == 1) {
            sensor->manufacturer = FLOWQUEST;
            sensor->type = CURRENT_PROFILER_SENSOR;
            sensor->channel = 0;
            strcpy(sensor->name, "FLOWQUEST");
            DEBUG("OK\n");
            return 1;
        } else {
            DEBUG("NO\n");
        }
    }
    return 0;
}

/**
 * Request and parse ADCP current profiler data from a smart sensor.
 * It performs a two-step acquisition: a short initial response followed by the main data frame.
 *
 * @param adcp Pointer to the structure where the parsed data will be stored
 * @param sensor_number Identifier of the sensor to be queried
 * @return 1 if data was successfully acquired and parsed, 0 on error
 */
static int request_current_profiler(struct adcp_raw_data_flowquest *adcp, int sensor_number)
{
    int count_receive = 0;
    uint8_t *p_response;
    uint8_t response_trama[BUFFER_SIZE] = {0};

    count_receive = gets_with_timeout(response_trama, BUFFER_SIZE, 50000);
    if (count_receive > 0) {
        DEBUG("Trama recibida con exito!\n");
        p_response = response_trama;

        if (parse_flowquest_data_frame(count_receive, p_response, adcp) != 0) {
            DEBUG("Error: No se pudo analizar la trama correctamente.\n");
            return 0;
        }

        return 1;
    } else {
        DEBUG("No se recibió la trama correctamente.\n");
        return 0;
    }
}

/**
 * Acquire one smart sensor attached to this device.
 * @param tries The number of retries if there are problems with the sensor
 * @param sensor The sensor to read
 * @param measurements A Pointer to store the result measurement
 * @return 1 if OK, 0 on error // TODO Better return the error code
 */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *measurement)
{
    struct adcp_raw_data_flowquest flowquest;

    while (tries > 0) {
        watchdog_reset();
        DEBUG("Trying start_deployment()...\n");
        if (start_deployment(sensor) == 1) {
            DEBUG("Start Deployment successful!\n");
            watchdog_reset();
            if (request_current_profiler(&flowquest, sensor->number)) {
                watchdog_reset();
                process_flowquest_raw_data(&flowquest, &adcp_processed_data);

                /* Limitar el número de celdas a 120 */
                if (adcp_processed_data.cells > 120) {
                    adcp_processed_data.cells = 120;
                    DEBUG("Limitando a 120 celdas de datos ADCP\n");
                }

                DEBUG("Procesando %d celdas de datos ADCP\n", adcp_processed_data.cells);
                measurement->type = CURRENT_PROFILER_SENSOR;
                measurement->current_profiler_signature.Heading = adcp_processed_data.heading;
                measurement->current_profiler_signature.Pitch = adcp_processed_data.pitch;
                measurement->current_profiler_signature.Roll = adcp_processed_data.roll;
                measurement->current_profiler_signature.Temperature = adcp_processed_data.temperature;
                measurement->current_profiler_signature.speed = adcp_processed_data.vel[5];
                measurement->current_profiler_signature.direction = adcp_processed_data.dir[5];
                measurement->sensor_status = SENSOR_OK;
                return 1;
            } else {
                DEBUG("Error reading Current Profiler sensor\n");
                tries--;
                watchdog_reset();
            }
        } else {
            DEBUG("Start deployment failed.\n");
            tries--;
            watchdog_reset();
        }
    }
    watchdog_reset();
    return 0;
}
/**
 * Transmits a command string to the sensor over serial (RS-232)
 * with debug logging and automatic command termination.
 *
 * @param cmd Command string to send
 * @param size Length of command string
 */
static void transmit_command(const char *cmd, size_t size)
{

    DEBUG("Sending ASCII: ");
    for (size_t i = 0; i < size; i++) {
        if (cmd[i] >= 32 && cmd[i] <= 126) {
            DEBUG("%c", cmd[i]);
        } else {
            DEBUG(".");
        }
    }
    DEBUG("\\r\n");

    for (size_t i = 0; i < size; i++) {
        serial_putchar(UART_SMART_SENSOR, cmd[i]);
    }
    serial_putchar(UART_SMART_SENSOR, '\r');
}

/**
 * Send a command to the smart sensor and wait for its response.
 *
 * Handles command transmission, optional flushing, and response reception with a timeout.
 *
 * @param command The command string to send
 * @param response Pointer to the buffer where the response will be stored
 * @param size The maximum size of the response buffer
 * @param timeout Maximum time to wait for a response, in microseconds
 * @return Number of bytes received in the response, or 0 on timeout or error
 */
static int send_command(const char *command, uint8_t *response, size_t size, uint32_t timeout)
{
    size_t command_size = strlen(command);

    transmit_command(command, command_size);
    if (strcmp(command, " #&!LQFQ.COMD0505\r\n") == 0) {
        serial_flush(UART_SMART_SENSOR);

        DEBUG("sending 0505\n");
    }
    serial_flush(UART_SMART_SENSOR);

    return gets_with_timeout(response, size, timeout);
}

/**
 * Initiate deployment mode on the FlowQuest smart sensor.
 *
 * Sends initialization and activation commands to the sensor, waits for confirmation
 * messages ("DL_AUTO" and "DSP_AUTO") to verify successful startup.
 *
 * @return 1 if deployment started successfully, 0 on failure
 */
static int start_deployment(struct smart_sensor *sensor)
{
    uint8_t response[BUFFER_SIZE] = {0};
    int n = 0;

    if (send_command(" !FQDL~&CMD%.8787", response, BUFFER_SIZE, 6000)) {
        if (strstr((const char *)response, "DL_AUTO")) {
            goto start_dsp;
        }
    }

    DEBUG("Initial .8787 failed. Trying .8282...\n");
    memset(response, 0, BUFFER_SIZE);
    if (send_command(" !FQDL~&CMD%.8282", response, BUFFER_SIZE, 6000) > 0) {
        if (strstr((const char *)response, "DL_CONF")) {
            DEBUG("Sensor in DL_CONF mode. Sending .9595...\n");
            watchdog_reset();
            n = send_command(" !FQDL~&CMD%.9595", response, BUFFER_SIZE, 8000);
            if (n != 0) {
                if (find_pattern(response, n, (const uint8_t *)"DSP_CONF", strlen("DSP_CONF"))) {
                    DEBUG("Sensor now in DSP_CONF mode. Retrying .8787...\n");
                    memset(response, 0, BUFFER_SIZE);
                    if (send_command(" !FQDL~&CMD%.8787", response, BUFFER_SIZE, 6000)) {
                        if (strstr((const char *)response, "DL_AUTO")) {
                            goto start_dsp;
                        }
                    }
                }
            }
        }
    }

    DEBUG("FlowQuest sensor Start Deployment failed.\n");
    return 0;

start_dsp:
    DEBUG("FlowQuest in DATA LOGGER mode!\n");
    sleep_microseconds(3000000);
    n = send_command("#&!LQFQ.COMD0505\r\n", response, BUFFER_SIZE, 6000);
    if (n != 0) {
        if (find_pattern(response, n, (const uint8_t *)"DSP_AUTO", strlen("DSP_AUTO"))) {
            DEBUG("FlowQuest START DSP!\n");
            return 1;
        }
    }

    DEBUG("FlowQuest failed to enter DSP mode.\n");
    return 0;
}

/**
 * Detect and configure the FlowQuest smart sensor.
 *
 * Attempts to communicate with the sensor to determine its current mode.
 * If in "DL_CONF" mode, proceeds to activate "DSP_CONF" mode.
 * If sensor responds as "FlowQuest Ready", reattempts configuration sequence.
 *
 * @param sensor Pointer to the smart_sensor structure (currently unused)
 * @return
 *    1 if sensor successfully configured into DSP_CONF mode,
 *    0 if communication occurred but configuration failed,
 */
static int detect_config(struct smart_sensor *sensor)
{

    uint8_t response[BUFFER_SIZE] = {0};
    const char *ready_str = "FlowQuest Ready";
    uint32_t timeout = 8000;
    int n = 0;

    DEBUG("Checking FlowQuest sensor...\n");
    if (send_command(" !FQDL~&CMD%.8282", response, BUFFER_SIZE, timeout) > 0) {
        if (strstr((const char *)response, "DL_CONF")) {
            DEBUG("Sensor in DL_CONF mode. Sending .9595...\n");
            watchdog_reset();
            n = send_command(" !FQDL~&CMD%.9595", response, BUFFER_SIZE, timeout);
            if (n != 0) {
                if (find_pattern(response, n, (const uint8_t *)"DSP_CONF", strlen("DSP_CONF"))) {
                    DEBUG("Sensor now in DSP_CONF mode.\n");
                    return 1;
                }
            }
            return 0;
        }

        if (strstr((const char *)response, ready_str)) {
            DEBUG("Sensor responded with 'FlowQuest Ready'. Sending .8282 again...\n");

            sleep_microseconds(3000);
            filter_invalid_chars = 1;

            if (send_command(" !FQDL~&CMD%.8282", response, BUFFER_SIZE, timeout)) {
                if (strstr((const char *)response, "DL_CONF")) {
                    DEBUG("Now in DL_CONF mode. Sending .9595...\n");
                    sleep_microseconds(3000);
                }
                n = send_command(" !FQDL~&CMD%.9595", response, BUFFER_SIZE, timeout);
                if (n != 0) {
                    if (find_pattern(response, n, (const uint8_t *)"DSP_CONF", strlen("DSP_CONF"))) {
                        DEBUG("Sensor now in DSP_CONF mode.\n");
                        return 1;
                    }
                }
            }

            filter_invalid_chars = 0;
            return 0;
        }

        DEBUG("Unexpected response: '%s'\n", response);
        return 0;
    }

    DEBUG("No response from sensor.\n");
    return -1;
}

/**
 * Receive a line of data from the smart sensor with a timeout.
 *
 * Switches to RS485 reception mode and listens for incoming bytes on UART.
 * Stops reading when either a newline character is received, the buffer is full,
 * or the timeout expires. Adds a null terminator at the end of the received data.
 *
 * @param response Pointer to the buffer where the received data will be stored
 * @param size Maximum number of bytes to store (including null terminator)
 * @param timeout Timeout in microseconds for waiting on incoming data
 * @return Number of bytes received (excluding null terminator), or 0 if timeout occurred
 */
static int gets_with_timeout(uint8_t *response, int size, uint32_t timeout)
{
    int n = 0;
    int c;

    rs485_receive(UART_SMART_SENSOR);
    DEBUG("get with timeout\n");
    /* Calculate the absolute end time for the total timeout */
    k_timepoint_t end_time = sys_timepoint_calc(K_MSEC(timeout));
    /* Set up idle timeout parameters */
    int idle_timeout_ms = 100;
    k_timepoint_t idle_end_time;
    /* Watchdog reset control */
    int loop_counter = 0;
    int watchdog_reset_frequency = 1000; /* Reset every 1000 iterations of the loop */

    while (1) {
        watchdog_reset();
        loop_counter++;
        if (loop_counter % watchdog_reset_frequency == 0) {
            watchdog_reset();
        }
        c = serial_getchar(UART_SMART_SENSOR);
        if (c < 0) {
            /* Check for total timeout */
            if (sys_timepoint_expired(end_time)) {
                DEBUG("Total timeout reached\n");
                break;
            }
            /* Check for idle timeout - only if we've received at least one character */
            if (n > 0) {
                if (sys_timepoint_expired(idle_end_time)) {
                    DEBUG("Idle timeout after receiving data\n");
                    break;
                }
            }
        } else {
            /* Valid character received */
            response[n++] = (uint8_t)c;
            idle_end_time = sys_timepoint_calc(K_MSEC(idle_timeout_ms));
            /* Reset watchdog every certain amount of data received */
            if (n % 50 == 0) { /* Every 50 characters */
                watchdog_reset();
            }
            /* Break if buffer is almost full (leave space for null terminator) */
            if (n >= (size - 1)) {
                break;
            }
        }
    }
    /* Final watchdog reset */
    watchdog_reset();
    /* Add null terminator */
    response[n] = '\0';
    return n;
}
static int needs_external_voltage(void)
{
    return 1;
}
