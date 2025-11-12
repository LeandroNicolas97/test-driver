/*
 * Communication with the smart sensors pH-AQ50 from ANBSENSORS
 */

#include <stdlib.h>
#include <string.h>

#include "bsp-config.h"
#include "configuration.h"
#include "crc16.h"
#include "debug.h"
#include "defaults.h"
#include "errorcodes.h"
#include "hardware.h"
#include "microio.h"
#include "modbus.h"
#include "serial.h"
#include "smart_sensor.h"
#include "timeutils.h"
#include "watchdog.h"

#define MAX_SENSORS        1
#define DETECTION_TRIES    2
#define PH_SLAVE_ADDR      0x55
#define SENSOR_TURN_ON_OFF 14000000 /* Delay time of sensor when turn on or turn off */
#define PH_REG             0x00
#define TEMP_REG           0x02
#define TRANSDUCER_REG     0x08

/*
 *Enum for diferent type of sensors
 */
enum anb_sensor_type {
    ANB_PH,
    ANB_END
};

/*
 *Driver function prototypes
 */
static int max_sensors(void);                                      /*Maximum number of sensors for this driver*/
static const char *name(void);                                     /*Name of the driver*/
static int init_driver(void);                                      /*Initialize the driver*/
static int finish_driver(void);                                    /*Finish the operations with the driver*/
static int detect(int sensor_number, struct smart_sensor *sensor); /*Detect a sensor*/
static int prepare(struct smart_sensor *sensor);                   /*Prepare the sensor*/
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m); /*Acquire measurement register*/
static int needs_external_voltage(void);

/*
 *Local prototypes
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg,
                                 uint16_t coils);
static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement);
static int get_parameter(float *param, struct smart_sensor *sensor, struct measurement *m, uint16_t reg);
static int get_parameter_status(int *param, struct smart_sensor *sensor, struct measurement *m, uint16_t reg);
static void turn_off_sensor(struct smart_sensor *sensor);

/*
 *Smart sensor driver structure for the ANB sensors
 *This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_anb = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .acquire = acquire,
    .name = name,
    .needs_external_voltage = needs_external_voltage,
};

/*
 *Get the maximum number of sensors of this type this driver can handle.
 */
static int max_sensors(void)
{
    return MAX_SENSORS;
}

/*
 *get the name of the driver.
 */
static const char *name(void)
{
    return "ANBsensors";
}

/*
 *Prepare the smart-sensors to start a measurement, this means turning them on and
 *enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    return 0;
}

static int finish_driver(void)
{
    return 0;
}
/**
 * Turn off sensor AQ50
 */
static void turn_off_sensor(struct smart_sensor *sensor)
{
    struct modbus_frame f;

    watchdog_disable();
    sleep_microseconds(1000000);
    watchdog_init();

    prepare_modbus_frame(&f, sensor, MODBUS_WRITE_SINGLE_COIL, 0x00, 0x00);
    modbus_query(UART_SMART_SENSOR, &f);
    modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
}

static int prepare(struct smart_sensor *sensor)
{
    struct modbus_frame f;
    int response_status;

    watchdog_disable();
    sleep_microseconds(1000000);
    watchdog_init();

    prepare_modbus_frame(&f, sensor, MODBUS_WRITE_SINGLE_COIL, 0x00, 0x04);
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
 *Check for a single sensor with the specified number on the RS-485 bus
 *@param sensor_number The number of the sensor to check for.
 *@param measurement A pointer to return a measurement from the sensor
 *@param sensor A pointer to store the sensor information
 *@return True if a sensor was detected
 */
static int detect(int sensor_number, struct smart_sensor *sensor)
{
    int tries;

    DEBUG("Checking ANBsensors %i...\n", sensor_number);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        if (prepare(sensor) == 0) {
            sensor->manufacturer = ANBSENSORS;
            sensor->power_up_time = ANBSENSORS_POWERUP_TIME;
            sensor->channel = 0; /*Currently we have only ine channel*/
            strcpy(sensor->name, "ANBSEN");
            turn_off_sensor(sensor);
            DEBUG("OK\n");
            return 1;
        } else {
            DEBUG("NO\n");
        }
    }
    return 0;
}

/**
 *Acquire one smart sensor attached to this device.
 *@param tries The number of retries if there are problems with the sensor
 *@param sensor The sensor to read
 *@param measurements A pointer to store the result measurement
 *@return 1 if ok, o on error
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

/**
 *
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg,
                                 uint16_t coils)
{
    switch (sensor->number) {
        case ANB_PH:
            f->slave_address = PH_SLAVE_ADDR;
            sensor->type = PH_SENSOR;
            break;
        default:
            f->slave_address = 1;
            break;
    }
    f->function_code = function;
    f->register_address = reg;
    f->n_coils = coils;
}

/**
 *static void rawhex2float(const float *f, const uint16_t *c) {
 * char *p = (char *)f;
 * char *q = (char *)c;
 *
 * memcpy(p, q + 2, 2);
 * memcpy(p + 2, q, 2);
 *}
 */

/**
 *
 */
static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    int rc;
    float pH = 0.0;
    float temperature = 0.0;
    int transducer = 0;

    /**
     * Read pH register
     */
    rc = get_parameter(&pH, sensor, measurement, PH_REG);
    if (rc == 0) {
        return rc;
    }
    /**
     * Read temperature register
     */
    rc = get_parameter(&temperature, sensor, measurement, TEMP_REG);
    if (rc == 0) {
        return rc;
    }
    temperature /= 1000;
    /**
     * Read transducer health status register
     */
    rc = get_parameter_status(&transducer, sensor, measurement, TRANSDUCER_REG);
    if (rc == 0) {
        return rc;
    }

    /*put data in measurement struct*/
    struct pH_measurement *m = &(measurement->pH);

    measurement->sensor_status = SENSOR_OK;
    measurement->type = PH_SENSOR;
    m->depth = 0;
    m->temperature = temperature;
    m->pH = pH;
    m->redox = 0;
    m->humidity = 0;
    m->depth_status = MEASUREMENT_VALUE_FIXED;
    m->temperature_status = MEASUREMENT_OK;
    m->pH_status = MEASUREMENT_OK;
    m->redox_status = MEASUREMENT_VALUE_FIXED;
    DEBUG("pH: %.2f\ntemperature: %.2f\n", (double)pH, (double)temperature);
    turn_off_sensor(sensor);
    return 1;
}

static int get_parameter(float *param, struct smart_sensor *sensor, struct measurement *m, uint16_t reg)
{
    int rc;
    struct modbus_frame f;

    watchdog_disable();
    sleep_microseconds(1000000);
    watchdog_init();

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
    /* rawhex2float(param, &f.data[0]); */
    *param = modbus_get_float(&f.data[0]);
    return 1;
}

static int get_parameter_status(int *param, struct smart_sensor *sensor, struct measurement *m, uint16_t reg)
{
    struct modbus_frame f;
    /**
     * Get Transducer Health Status,
     * If "status>0" status transducer error
     * check manual in transducer health
     * https://www.anbsensors.com/docs/docs/AQ50/sensor-output
     *
     */

    watchdog_disable();
    sleep_microseconds(1000000);
    watchdog_init();

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, reg, 1);
    modbus_query(UART_SMART_SENSOR, &f);
    modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    /* f.data[0] = 0; */
    *param = f.data[0];
    if (*param > 0) {
        DEBUG("Status transducer error AQ50: %i\n", *param);
        m->sensor_status = SENSOR_INTERNAL_ERROR;
        return 0;
    }
    return 1;
}

static int needs_external_voltage(void)
{
    return 1;
}
