/*
 * Communication with the smart sensors
 */

#include <string.h>
#include <stdlib.h>
#include "bsp-config.h"
#include "smart_sensor.h"
#include "serial.h"
#include "errorcodes.h"
#include "microio.h"
#include "crc16.h"
#include "hardware.h"
#include "timeutils.h"
#include "modbus.h"
#include "debug.h"
#include "configuration.h"
#include <math.h>
#include "watchdog.h"

#define DETECTION_TRIES   3
#define MAX_SENSORS       1
#define MAX_RESPONSE_SIZE 128

#define SLAVE_ADDRESS 0x06

/**
 * Driver function prototypes
 */
static int max_sensors(void);                                      /* Maximum number of sensors for this driver */
static const char *name(void);                                     /* Name of the driver */
static int init_driver(void);                                      /* Initialize the driver */
static int finish_driver(void);                                    /* Finish the operations with the driver */
static int detect(int sensor_number, struct smart_sensor *sensor); /* Detect a sensor */
static int prepare(struct smart_sensor *sensor);                   /* Prepare the sensor */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m); /* Acquire data of the sensor */
static int needs_external_voltage(void);

/*
 * Local prototypes
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg,
                                 uint16_t coils);
static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement);

/**
 * Smart sensor driver structure for the Chemins sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_chemins = {
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
    return "CHEMINS";
}

/*
 * Start the operation with the smart sensors
 */

static int init_driver(void)
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

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */

static int prepare(struct smart_sensor *sensor)
{
    struct modbus_frame f;
    int response_status;

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, 0x2002, 1);
    modbus_query(UART_SMART_SENSOR, &f);
    response_status = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (response_status == -E_NOT_DETECTED) {
        return response_status;
    }
    if (response_status == -E_BAD_CHECKSUM) {
        return response_status;
    }
    if (response_status == -E_INVALID) {
        return response_status;
    }
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

    DEBUG("Checking Chemins %i... ", sensor_number);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        sensor->type = CHLOROPHYLL_SENSOR;
        if (prepare(sensor) == 0) {
            sensor->manufacturer = CHEMINS;
            sensor->power_up_time = CHEMINS_POWERUP_TIME;
            sensor->channel = 0;
            strcpy(sensor->name, "CHEMINS");
            DEBUG("OK\n");
            return 1;
        } else {
            DEBUG("NO\n");
        }
    }
    return 0;
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
    while (tries > 0) {
        DEBUG("Trying\n");
        if (modbus_request_measurement(sensor, measurement)) {
            break;
        }
        DEBUG("Error reading sensor %s\n", sensor->name);
        tries--;
    }
    if (tries > 0) {
        return 1;
    } else {
        return 0;
    }
}

/*
 * Prepare modbus frame for communication.
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg,
                                 uint16_t coils)
{
    f->slave_address = SLAVE_ADDRESS;
    sensor->type = CHLOROPHYLL_SENSOR;
    f->function_code = function;
    f->register_address = reg;
    f->n_coils = coils;
}

/*
 * Request measurement of the sensor.
 */

static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    struct modbus_frame f;
    int response_status;
    float chlorophyll = 0.0;
    float temperature = 0.0;

    /*READ THE 4 REGISTERS FOR CLOROFILA AND TEMP*/
    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, 0x0000, 4);
    modbus_query(UART_SMART_SENSOR, &f);
    response_status = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (response_status == -E_NOT_DETECTED) {
        measurement->sensor_status = SENSOR_NOT_DETECTED;
        return 0;
    }
    if (response_status == -E_BAD_CHECKSUM) {
        measurement->sensor_status = SENSOR_COMMUNICATION_BAD_CRC;
        return 0;
    }
    if (response_status == -E_INVALID) {
        measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
        return 0;
    }
    /*frame is ok*/
    /* calculate chlorophyll and temperature using decimals */
    chlorophyll = f.data[0] / powf(10, f.data[1]);
    temperature = f.data[2] / powf(10, f.data[3]);

    DEBUG("CHEMINS CLOROFILA: %.2f TEMP: %.2f\n", (double)chlorophyll, (double)temperature);

    /*put data in measurement struct*/
    struct chlorophyll_measurement *m = &(measurement->chlorophyll);

    measurement->sensor_status = SENSOR_OK;
    measurement->type = CHLOROPHYLL_SENSOR;
    m->chlorophyll = chlorophyll;
    m->temperature = temperature;
    m->depth = 10;
    m->humidity = 0;
    m->depth_status = MEASUREMENT_VALUE_FIXED;
    if (chlorophyll < 0) {
        m->chlorophyll_status = MEASUREMENT_TOO_LOW;
    } else if (chlorophyll >= 0 && chlorophyll <= 400) {
        m->chlorophyll_status = MEASUREMENT_OK;
    } else {
        m->chlorophyll_status = MEASUREMENT_TOO_HIGH;
    }
    if (temperature < 0) {
        m->temperature_status = MEASUREMENT_TOO_LOW;
    } else if (temperature >= 0 && temperature <= 50) {
        m->temperature_status = MEASUREMENT_OK;
    } else {
        m->temperature_status = MEASUREMENT_TOO_HIGH;
    }
    return 1;
}

static int needs_external_voltage(void)
{
    return 1;
}
