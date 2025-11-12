/*
 * Communication with the smart sensors
 */

#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "smart_sensor.h"
#include "clock.h"
#include "serial.h"
#include "errorcodes.h"
#include "microio.h"
#include "crc16.h"
#include "hardware.h"
#include "timeutils.h"
#include "modbus.h"
#include "debug.h"

#define DETECTION_TRIES   3
#define MAX_N_SENSORS     1
#define MAX_RESPONSE_SIZE 128

#define SLAVE_ADDRESS 0x01

/**
 * Driver function prototypes
 */
static int max_sensors(void);                                      /* Maximum number of sensors for this driver */
static int init_driver(void);                                      /* Initialize the driver */
static int finish_driver(void);                                    /* Finish the operations with the driver */
static int detect(int sensor_number, struct smart_sensor *sensor); /* Detect a sensor */
static int prepare(struct smart_sensor *sensor);                   /* Prepare the sensor */
/* static int finish(struct smart_sensor *sensor); Finish the communication with the sensor */
/* static int calibrate_zero(struct smart_sensor *sensor); Calibrate the zero of the sensor */
/* static int calibrate_full(struct smart_sensor *sensor); Calibrate the full scale of the sensor */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m);
/* static int pass_command(struct smart_sensor *sensor, char *command); */

/**
 * Smart sensor driver structure for the Yosemitech sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_aquas = {
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
};

/*
 * Local prototypes
 */
static void prepare_modbus_frame(uint8_t slave, uint8_t function, uint16_t reg, uint16_t coils);
static int modbus_request_measurement(int slave, struct measurement *measurement);
static int aquas_sensor_clean(int slave);

/*
 * Local variables
 */
struct modbus modbus_response, modbus_request;
uint16_t modbus_data_buffer[MAX_RESPONSE_SIZE];

/*
 * Get the maximum number of sensors of this type this driver can handle
 */
static int max_sensors(void)
{
    return MAX_N_SENSORS;
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    smart_sensor_init_serial_port();
    serial_tx_enable(UART_SMART_SENSOR);
    serial_rx_enable(UART_SMART_SENSOR);
    return 0;
}

/*
 *
 */
static int prepare(struct smart_sensor *sensor)
{
    return aquas_sensor_clean(sensor->number);
}

/*
 * Finish the operation with the smart sensors
 */
static int finish_driver(void)
{
    serial_tx_disable(UART_SMART_SENSOR);
    serial_rx_disable(UART_SMART_SENSOR);
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

    DEBUG("Checking Aquas %i... ", sensor_number);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        if (prepare(sensor) == 0) {
            sensor->manufacturer = AQUAS;
            sensor->power_up_time = AQUAS_SENSORS_POWERUP_TIME;
            switch (modbus_request.slave_addr) {
                case SLAVE_ADDRESS:
                    sensor->type = CHLOROPHYLL_SENSOR;
                    break;
                default:
                    break;
            }
            sensor->channel = 0;           /* Currently we have only one channel */
            strcpy(sensor->name, "AQUAS"); /* TODO: make this string from the sensor number */
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
        if (modbus_request_measurement(sensor->number, measurement)) {
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
static void prepare_modbus_frame(uint8_t slave, uint8_t function, uint16_t reg, uint16_t coils)
{
    modbus_request.slave_addr =
        SLAVE_ADDRESS; /*TODO: use slave number from the arguments, so change sensor address in sensor*/
    modbus_request.funct_code = function;
    modbus_request.reg_addr = reg;
    modbus_request.n_coils = coils;
    modbus_request.data = modbus_data_buffer;
    modbus_response.data = modbus_data_buffer;
    modbus_response.msb_or_lsb = MSB_FIRST;
    modbus_request.msb_or_lsb = MSB_FIRST;
}

/*
 *
 */
static int modbus_request_measurement(int slave, struct measurement *measurement)
{
    int response_status;
    float chlo, temp;

    /*READ THE CHLOROPHYLL REGISTER*/
    serial_flush(UART_SMART_SENSOR); /* Clean the receive buffer */
    prepare_modbus_frame(slave, MODBUS_READ_INPUT_REGISTERS, 0x0000, 2);
    modbus_query(UART_SMART_SENSOR, &modbus_request);

    int i = 0;

    while (i < DETECTION_TRIES) {
        response_status = modbus_poll(UART_SMART_SENSOR, &modbus_response, BIG_ENDIAN);
        if (response_status >= 0) {
            break;
        }
        i++;
    }
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
    memcpy(&chlo + 2, &modbus_response.data[0], 2);
    memcpy(&chlo, &modbus_response.data[1], 2);
    DEBUG("      chlo: %.2f\n", chlo);

    /*READ THE TEMPERATURE REGISTER*/
    serial_flush(UART_SMART_SENSOR); /* Clean the receive buffer */
    prepare_modbus_frame(slave, MODBUS_READ_INPUT_REGISTERS, 0x0001, 2);
    modbus_query(UART_SMART_SENSOR, &modbus_request);
    i = 0;
    while (i < DETECTION_TRIES) {
        response_status = modbus_poll(UART_SMART_SENSOR, &modbus_response, BIG_ENDIAN);
        if (response_status >= 0) {
            break;
        }
        i++;
    }
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
    memcpy(&temp + 2, &modbus_response.data[0], 2);
    memcpy(&temp, &modbus_response.data[1], 2);
    DEBUG("      temp: %.2f\n", temp);

    /*put data in measurement struct*/
    measurement->type = CHLOROPHYLL_SENSOR;
    struct chlorophyll_measurement *m = &(measurement->chlorophyll);

    measurement->sensor_status = SENSOR_OK;
    m->depth = 10;
    m->temperature = temp;
    m->chlorophyll = chlo;
    m->humidity = 0;
    m->depth_status = MEASUREMENT_VALUE_FIXED;
    m->temperature_status = MEASUREMENT_OK;
    m->chlorophyll_status = MEASUREMENT_OK;

    return 1;
}

static int aquas_sensor_clean(int slave)
{
    int response_status;

    sleep_microseconds(1000000);     /* 1000ms */
    serial_flush(UART_SMART_SENSOR); /* Clean the receive buffer */
    prepare_modbus_frame(slave, MODBUS_WRITE_SINGLE_HOLDING_REGISTER, 4, 1);
    modbus_query(UART_SMART_SENSOR, &modbus_request);
    response_status = modbus_poll(UART_SMART_SENSOR, &modbus_response, BIG_ENDIAN);
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
