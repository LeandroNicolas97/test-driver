/***************************************************************************
 *   file                 : smart_sensor_signature_nortek.c                *
 *   begin                : Ene, 2025                                   *
 *   copyright            : (C) 2025 by Innovex Tecnologias Ltda.          *
 *   Author               : Fabian Olmos                                   *
 *   Email                : edgar.osorio@innovex.cl                        *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2011. Innovex.                                          *
 ***************************************************************************/

/*
 * Communication with the smart sensors ADCP AQUAPRO NORTEK
 */

#include <string.h>
#include <stdlib.h>
#include "bsp-config.h"
#include "measurement.h"
#include "smart_sensor.h"
#include "serial.h"
#include "hardware.h"
#include "debug.h"
#include "nortek_signature.h"
#include "watchdog.h"
#include "zephyr/sys_clock.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define UART_DEVICE_NODE DT_ALIAS(uart_smart_sensor)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* How many times we try to detect a sensor */
#define DETECTION_TRIES   3
#define MAX_SENSORS       1
#define MAX_RESPONSE_SIZE 256 /* Maximum size for a response from the sensors */

#define EAST_ENU  0
#define NORTH_ENU 1

#define BUFFER_RECEP_ADCP 1500

#define DEPRECATED 0

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

/**
 * Smart sensor driver structure for the Maxbotix range sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_aquadopp_nortek = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .finish = NULL,
    .calibrate_zero = NULL, /* This sensors don't require calibration */
    .calibrate_full = NULL,
    .acquire = acquire,
    .pass_command = NULL,
    .name = name,
    .needs_external_voltage = needs_external_voltage,
};

/*
 * Local prototypes
 */
int aquadopp_sensor_gets_with_timeout(char *response, int size, uint32_t timeout);
/* static int adcp_sensor_gets(char *response, int size); */

/*
 * Get the maximum number of sensors of this type this driver can handle
 */
static int max_sensors(void)
{
    return MAX_SENSORS;
}

/**
 * Get the name of the driver.
 */
static const char *name(void)
{
    return "Aquadopp";
}

/*
 * Send command to Seabird sensors
 */
static void smart_sensor_send_command(unsigned char *data, size_t size)
{
    for (int i = 0; i < size; i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

#if DEPRECATED
/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout of 1 second. The newline is not included in the buffer.
 * @param s A pointer to a buffer to store the received data
 * @param size Size of the rec"eive buffer
 * @return The number of bytes stored in the buffer
 */
static int adcp_sensor_gets(char *s, int size)
{
    return aquadopp_sensor_gets_with_timeout(s, size, 1000);
}
#endif

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @param timeout After this time, signal timeout (microseconds)
 * @return The number of bytes stored in the buffer
 */
int aquadopp_sensor_gets_with_timeout(char *response, int size, uint32_t timeout)
{
    int n = 0;
    char c;
    int enable;

    rs485_receive(UART_SMART_SENSOR);
    enable = 0;
    k_timepoint_t end = sys_timepoint_calc(K_MSEC(timeout));

    while (1) {
        watchdog_reset();
        if (uart_poll_in(uart_dev, &c) >= 0) {
            watchdog_reset();
            *response++ = c;
            n++;
        }
        if (sys_timepoint_expired(end)) {
            break;
        }
    }
    *response = '\0';
    return n;
}

/* send BREAK commnad to ADCP */

static void send_break(void)
{
    printk("Send break\n");
    rs485_transmit(UART_SMART_SENSOR);
    smart_sensor_send_command("@@@@@@", 6);
    smart_sensor_send_command("\r", 1);

    watchdog_reset();
    sleep_microseconds(150000); /* delay 150ms */
    smart_sensor_send_command("K1W%!Q\r", 7);
}

/*
 * Prepare the driver
 */
static int prepare(struct smart_sensor *sensor)
{
    uint8_t response[100];
    uint8_t count_receive = -1;
    uint8_t try = 5;

    rs485_transmit(UART_SMART_SENSOR);
    while (try > 0) {
        send_break();
        count_receive = aquadopp_sensor_gets_with_timeout(response, 100, 1000);
        printk("Count_ %i\n", count_receive);
        if (count_receive > 50) {
            break;
        }
        try--;
    }
    if (count_receive > 50) {
        return 0;
    } else {
        return -1;
    }
}

static int8_t request_current_profiler(struct measurement *measurement, int sensor_number)
{
    int count_receive;
    uint8_t response[BUFFER_RECEP_ADCP];

    static PdAqProf aquadopp;

    rs485_transmit(UART_SMART_SENSOR);
    smart_sensor_send_command("AD\r", 3);
    sleep_microseconds(150000); /* delay 150ms */
    watchdog_reset();
    serial_flush(UART_SMART_SENSOR);
    sleep_microseconds(100000000);
    count_receive = aquadopp_sensor_gets_with_timeout(response, BUFFER_RECEP_ADCP, 40000);

    DEBUG("DATA RECEIVED: %i\n", count_receive);
    DEBUG("Respose 0: %i\n", response[0]);
    DEBUG("Respose 1: %i\n", response[1]);
    DEBUG("Respose 2: %i\n", response[2]);
    DEBUG("DATA_STREAM:\n");
    for (int i = 0; i < count_receive; i++) {
        printk("%.2x", response[i]);
    }
    printk("\n");
    if (response[0] == AQUADOPP_PROFILER_VELOCITY_DATA_SYNC && response[1] == AQUADOPP_PROFILER_VELOCITY_DATA_ID &&
        response[2] == 75) {
        parse_aquadopp_data_frame(response, &aquadopp);
        process_aquadopp_raw_data(&aquadopp, &adcp_processed_data);
        measurement->current_profiler_signature.Heading = (float)aquadopp.hHeading * 0.1f;
        measurement->current_profiler_signature.Pitch = (float)aquadopp.hPitch * 0.1f;
        measurement->current_profiler_signature.Roll = (float)aquadopp.hRoll * 0.1f;
        measurement->current_profiler_signature.Temperature = (float)aquadopp.hTemperature * 0.01f;
        measurement->current_profiler_signature.speed = adcp_processed_data.vel[5];
        measurement->current_profiler_signature.direction = adcp_processed_data.dir[5];
        measurement->current_profiler_signature.current_profiler_signature_status = MEASUREMENT_OK;
        measurement->sensor_status = SENSOR_OK;
        measurement->type = CURRENT_PROFILER_SENSOR;
        return 1;
    }
    return 0;
}

/**
 * Check if there is a Maxbotix range sensor connected
 * @param sensor_number The number of the sensor to check for (Unused here)
 * @param measurement A pointer to return a measurement from the sensor
 * @param sensor A pointer to store the sensor information
 * @return True if a sensor was detected
 */
int detect(int sensor_number, struct smart_sensor *sensor)
{
    int tries;

    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        printk("Detect Aquadopp\n");
        if (prepare(sensor) == 0) {
            sensor->type = CURRENT_PROFILER_SENSOR; /* RANGE_SENSOR; */
            sensor->manufacturer = AQUADOPP;
            sensor->power_up_time = 1000; /* TODO Check from datasheet */
            sensor->channel = 0;          /* We can only have one sensor of this type */
            strcpy(sensor->name, "Aquadopp");
            DEBUG("OK\n");
            return 1;
        } else {
            DEBUG("NO\n");
            return 0;
        }
    }
    return 0; /* should never reach here happy compiler */
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    if (!device_is_ready(uart_dev)) {
        printk("Error device seabird\n");
        return -4; /* Some error on OC4 */
    }
    return 0;
}

/*
 * Finish the operation with the smart sensors
 */
static int finish_driver(void)
{
    /* turn_off_smart_sensor(0);  // TODO Move to main */
    return 0;
}

/**
 * Acquire one smart sensor attached to this device.
 * We read many times into an array, then we sort the array and take the value in the
 * middle, this way we hope to remove extreme values.
 * @param tries The number of retries if there are problems with the sensor
 * @param sensor The sensor to read
 * @param measurements A Pointer to store the result measurement
 * @return 1 if OK, 0 on error // TODO Better return the error code
 */
int acquire(int tries, struct smart_sensor *sensor, struct measurement *measurement)
{

    while (tries > 0) {
        if (request_current_profiler(measurement, sensor->number)) {
            return 1;
        } else {
            DEBUG("Error reading Current Profiler sensor\n");
            tries--;
        }
        watchdog_reset();
    }
    return 0;
}

static int needs_external_voltage(void)
{
    return 0;
}
