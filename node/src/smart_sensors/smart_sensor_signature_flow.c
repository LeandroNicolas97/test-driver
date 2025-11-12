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
#include "modbus.h"
#include "debug.h"
#include "hardware.h"

#define DETECTION_TRIES   3
#define MAX_SENSORS       1
#define MAX_RESPONSE_SIZE 128

#define DEVICE_ADDRESS 0x02

#define LEVEL_REG       39
#define VELOCITY_REG    99
#define FLOW_RATE_REG   159
#define TEMPERATURE_REG 219
#define TOTAL_FLOW_REG  279

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
 * Smart sensor driver structure
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_signature_flow = {
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
static int get_parameter(float *param, struct smart_sensor *sensor, struct measurement *m, uint16_t reg);

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
    return "Sig-flow";
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
 * Prepare the driver for taking a measurement, it also serves for detect
 * operation.
 */
static int prepare(struct smart_sensor *sensor)
{
    struct modbus_frame f;
    int response_status;

    serial_flush(UART_SMART_SENSOR);
    prepare_modbus_frame(&f, sensor, MODBUS_WRITE_SINGLE_HOLDING_REGISTER, 24, 2);
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
    DEBUG("Checking Signature Flow %i... ", sensor_number);
    for (int tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        if (prepare(sensor) == 0) {
            sensor->manufacturer = TELEDYNE_ISCO;
            sensor->power_up_time = 1000;
            sensor->type = FLOW_ULTRASONIC_SENSOR;
            sensor->channel = 0;
            strcpy(sensor->name, name());
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
 * Prepare the modbus frame struct.
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *s, uint8_t function, uint16_t reg,
                                 uint16_t coils)
{
    f->slave_address = DEVICE_ADDRESS;
    f->function_code = function;
    f->register_address = reg;
    f->n_coils = coils;
}

static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    int rc;
    float flow_rate = 0.0;
    float velocity = 0.0;
    float total_flow = 0.0;
    float temp = 0.0;

    /* READ TEMP */
    rc = get_parameter(&temp, sensor, measurement, TEMPERATURE_REG);
    if (rc == 0) {
        return rc;
    }
    DEBUG("Signature TEMPERATURE: %.2f\n", (double)temp);
    sleep_microseconds(10000);

    /* READ FLOW RATE */
    rc = get_parameter(&flow_rate, sensor, measurement, FLOW_RATE_REG);
    if (rc == 0) {
        return rc;
    }
    flow_rate *= 1000; /* m3/s to L/s */
    DEBUG("Signature FLOW RATE: %.2f\n", (double)flow_rate);
    sleep_microseconds(10000);

    /* READ VELOCITY */
    rc = get_parameter(&velocity, sensor, measurement, VELOCITY_REG);
    if (rc == 0) {
        return rc;
    }
    DEBUG("Signature FLOW VELOCITY: %.2f\n", (double)velocity);
    sleep_microseconds(10000);

    /* READ TOTAL FLOW */
    rc = get_parameter(&total_flow, sensor, measurement, TOTAL_FLOW_REG);
    if (rc == 0) {
        return rc;
    }
    DEBUG("Signature FLOW TOTAL FLOW: %.2f\n", (double)total_flow);
    sleep_microseconds(10000);

    /* READ LEVEL */
    float level = 0.0;

    rc = get_parameter(&level, sensor, measurement, LEVEL_REG);
    if (rc == 0) {
        return rc;
    }
    DEBUG("Signature FLOW LEVEL: %.2f\n", (double)level);

    /*put data in measurement struct*/
    measurement->type = FLOW_ULTRASONIC_SENSOR;
    struct flow_ultrasonic_measurement *m = &(measurement->flow_ultrasonic);

    measurement->sensor_status = SENSOR_OK;
    m->temperature = temp;
    m->speed = velocity;
    m->rate = flow_rate;
    m->totalizer = total_flow;
    m->temperature_status = MEASUREMENT_OK;
    m->speed_status = MEASUREMENT_OK;
    m->rate_status = MEASUREMENT_OK;
    m->totalizer_status = MEASUREMENT_OK;

    return 1;
}

static int get_parameter(float *param, struct smart_sensor *sensor, struct measurement *m, uint16_t reg)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, reg, 2);
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        m->sensor_status = SENSOR_NOT_DETECTED;
        return 0;
    }
    if (rc == -E_BAD_CHECKSUM) {
        m->sensor_status = SENSOR_COMMUNICATION_BAD_CRC;
        return 0;
    }
    if (rc == -E_INVALID) {
        m->sensor_status = SENSOR_COMMUNICATION_ERROR;
        return 0;
    }
    /*frame is ok*/
    union floatingPointIEEE754 value;

    value.raw.msb = f.data[1];
    value.raw.lsb = f.data[0];
    *param = value.f;

    return 1;
}

static int needs_external_voltage(void)
{
    return 1;
}
