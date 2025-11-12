/*
 * Communication with the smart sensors
 */

#include "measurement.h"
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
#include "watchdog.h"

#define DETECTION_TRIES   5
#define MAX_SENSORS       2
#define MAX_RESPONSE_SIZE 128

#define CTDO_1_ADDRESS 0x40
#define CTDO_2_ADDRESS 0x41

/**
 * Driver function prototypes
 */
static int max_sensors(void);                                      /* Maximum number of sensors for this driver */
static const char *name(void);                                     /* Name of the driver */
static int init_driver(void);                                      /* Initialize the driver */
static int finish_driver(void);                                    /* Finish the operations with the driver */
static int detect(int sensor_number, struct smart_sensor *sensor); /* Detect a sensor */
static int prepare(struct smart_sensor *sensor);                   /* Prepare the sensor */
/* static int finish(struct smart_sensor *sensor); //  Finish the communication with the sensor */
/* static int calibrate_zero(struct smart_sensor *sensor); // Calibrate the zero of the sensor */
/* static int calibrate_full(struct smart_sensor *sensor); // Calibrate the full scale of the sensor */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m);
/* static int pass_command(struct smart_sensor *sensor, char *command); */
static int needs_external_voltage(void);

/**
 * Smart sensor driver structure for the Yosemitech sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_ysi = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .finish = NULL,         /* finish, */
    .calibrate_zero = NULL, /* calibrate_zero, */
    .calibrate_full = NULL,
    .acquire = acquire,
    .pass_command = NULL,
    .name = name,
    .needs_external_voltage = needs_external_voltage,
};

/*
 * Local prototypes
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg,
                                 uint16_t coils);
static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement);

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
    return "Ysi";
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    /* serial_set_baudrate(UART_SMART_SENSOR, B9600); */
    return 0;
}

/*
 * Get the maximum number of sensors of this type this driver can handle
 */
static int prepare(struct smart_sensor *sensor)
{
    struct modbus_frame f;
    int response_status;

    serial_flush(UART_SMART_SENSOR);
    /* prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, 0x0000, 22); */
    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, 0x0000, 1);
    modbus_query(UART_SMART_SENSOR, &f);
    serial_flush(UART_SMART_SENSOR);
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

    DEBUG("Checking Ysi %i... ", sensor_number);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        if (prepare(sensor) == 0) {
            sensor->manufacturer = YSI;
            sensor->power_up_time = 2000;
            sensor->type = CTDO_SENSOR;
            sensor->channel = 0;
            strcpy(sensor->name, "YSI");
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
        DEBUG(" Trying\n");
        watchdog_reset();
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
 *
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *s, uint8_t function, uint16_t reg,
                                 uint16_t coils)
{
    if (s->number == 0) {
        f->slave_address = CTDO_1_ADDRESS;
    } else if (s->number == 1) {
        f->slave_address = CTDO_2_ADDRESS;
    }
    f->function_code = function;
    f->register_address = reg;
    f->n_coils = coils;
}

static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    struct modbus_frame f;
    int response_status;
    float param1 = 0.0;
    float param2 = 0.0;
    float param3 = 0.0;

    watchdog_disable();
    sleep_microseconds(1000000);
    watchdog_init();

    /* READ ALL */
    prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, 0x0000, 22);
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
    param1 = modbus_get_float(&f.data[0]);
    param2 = modbus_get_float(&f.data[6]);
    param3 = modbus_get_float(&f.data[12]);
    DEBUG("Ysi TEMP: %.2f COND: %.2f us/cm SAT: %.2f\n", (double)param2, (double)param3, (double)param1);

    /*put data in measurement struct*/
    measurement->type = CTDO_SENSOR;
    struct ctdo_raw_measurement *m = &(measurement->ctdo);

    measurement->sensor_status = SENSOR_OK;
    m->depth = 10;
    m->temperature = param2;
    m->conductivity = param3;
    m->saturation = param1;
    m->humidity = 0;
    m->depth_status = MEASUREMENT_VALUE_FIXED;
    m->temperature_status = MEASUREMENT_OK;
    m->conductivity_status = MEASUREMENT_OK;
    m->saturation_status = MEASUREMENT_OK;

    return 1;
}

static int needs_external_voltage(void)
{
    return 1;
}
