/*
 * Communication with the smart sensors
 */

#include "measurement.h"
#include "zephyr/sys/printk.h"
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

/* #define DEVICE_ADDRESS 0x05 */
#define DEVICE_ADDRESS 0x01

/* measurements data registers */
#define LEVEL_REG               50
#define VELOCITY_REG            10
#define FLOW_RATE_REG           52
#define TEMPERATURE_REG         48
#define TOTAL_FLOW_REG          54
/* configuration registers */
#define CHANNEL_TYPE_REG        120
#define WIDTH_DIMENSION_REG     126
#define BOTTOM_COMPENSATION_REG 122
#define ANGLE_REG               124
#define SET_CUMULATIVE_FLOW_REG 142

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
static int pass_command(struct smart_sensor *sensor, char *command);
static int needs_external_voltage(void);

/**
 * Smart sensor driver structure
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_jiangsu_flow = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .finish = NULL,         /* finish, */
    .calibrate_zero = NULL, /* calibrate_zero, */
    .calibrate_full = NULL,
    .acquire = acquire,
    .pass_command = pass_command,
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
static int get_parameter_double(float *param, struct smart_sensor *sensor, struct measurement *m, uint16_t reg);
static int get_configs(struct smart_sensor *sensor, char *cmd);
static int set_configs(struct smart_sensor *sensor, char *cmd, char *arg);
static int get_channel_type(struct smart_sensor *sensor);
static int set_channel_type(struct smart_sensor *sensor, uint8_t ch_type);
static int get_width(struct smart_sensor *sensor);
static int set_width(struct smart_sensor *sensor, float width);
static int get_bottom_compensation(struct smart_sensor *sensor);
static int set_bottom_compensation(struct smart_sensor *sensor, float comp);
static int get_angle(struct smart_sensor *sensor);
static int set_angle(struct smart_sensor *sensor, float angle);
static int get_totalizer(struct smart_sensor *sensor);
static int set_totalizer(struct smart_sensor *sensor, double totalizer_value);

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
    return "Jiangsu";
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
    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, TEMPERATURE_REG, 2);
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
    DEBUG("Checking Jiangsu Flow %i... ", sensor_number);
    for (int tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        if (prepare(sensor) == 0) {
            sensor->manufacturer = JIANGSU;
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
    DEBUG("Jiangsu TEMPERATURE: %.2f\n", (double)temp);
    sleep_microseconds(10000);

    /* READ FLOW RATE */
    rc = get_parameter(&flow_rate, sensor, measurement, FLOW_RATE_REG);
    if (rc == 0) {
        return rc;
    }
    flow_rate *= 1000; /* m3/s to L/s */
    DEBUG("Jiangsu FLOW RATE: %.2f\n", (double)flow_rate);
    sleep_microseconds(10000);

    /* READ VELOCITY */
    rc = get_parameter(&velocity, sensor, measurement, VELOCITY_REG);
    if (rc == 0) {
        return rc;
    }
    DEBUG("Jiangsu FLOW VELOCITY: %.2f\n", (double)velocity);
    sleep_microseconds(10000);

    /* READ TOTAL FLOW */
    rc = get_parameter_double(&total_flow, sensor, measurement, TOTAL_FLOW_REG);
    if (rc == 0) {
        return rc;
    }
    DEBUG("Jiangsu FLOW TOTAL FLOW: %.2f\n", (double)total_flow);
    sleep_microseconds(10000);

    /* READ LEVEL */
    float level = 0.0;

    rc = get_parameter(&level, sensor, measurement, LEVEL_REG);
    if (rc == 0) {
        return rc;
    }
    DEBUG("Jiangsu FLOW LEVEL: %.2f\n", (double)level);

    /*put data in measurement struct*/
    measurement->type = FLOW_ULTRASONIC_SENSOR;
    struct flow_ultrasonic_measurement *m = &(measurement->flow_ultrasonic);

    measurement->sensor_status = SENSOR_OK;
    m->temperature = temp;
    m->speed = velocity;
    m->rate = flow_rate;
    m->depth = level;
    m->totalizer = total_flow;
    m->temperature_status = MEASUREMENT_OK;
    m->speed_status = MEASUREMENT_OK;
    m->rate_status = MEASUREMENT_OK;
    m->depth_status = MEASUREMENT_OK;
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

static int get_parameter_double(float *param, struct smart_sensor *sensor, struct measurement *m, uint16_t reg)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, reg, 4);
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
    union DoubleIEEE754 value;

    value.raw.byte4 = f.data[3];
    value.raw.byte3 = f.data[2];
    value.raw.byte2 = f.data[1];
    value.raw.byte1 = f.data[0];
    *param = (float)value.d;

    return 1;
}

static int pass_command(struct smart_sensor *sensor, char *command)
{
    char *s = command;
    char *arg1;
    char *arg2;
    char *temp;
    uint8_t n_of_args = 0;
    int rc = 0;

    arg1 = strtok_r(s, " ", &s);
    if (arg1 == NULL) {
        /* no arguments */
        printk("Sin argumentos, ingresar correctamente.\n");
        return -1;
    }
    n_of_args++;
    arg2 = strtok_r(s, " ", &s);
    if (arg2 != NULL) {
        n_of_args++;
    }
    temp = strtok_r(s, " ", &s);
    if (temp != NULL) {
        /* to many arguments */
        printk("Demasiados argumentos, max 2.\n");
        return -1;
    }

    switch (n_of_args) {
        case 1:
            rc = get_configs(sensor, arg1);
            break;
        case 2:
            rc = set_configs(sensor, arg1, arg2);
            break;
        default:
            /* we should never use this case. */
            break;
    }
    return rc;
}

static int get_configs(struct smart_sensor *sensor, char *cmd)
{
    int rc = 0;

    if (!strcmp(cmd, "config")) {
        rc = get_channel_type(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
        rc = get_width(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
        rc = get_bottom_compensation(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
        rc = get_angle(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
        rc = get_totalizer(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "channel")) {
        rc = get_channel_type(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "width")) {
        rc = get_width(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "compensation")) {
        rc = get_bottom_compensation(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "angle")) {
        rc = get_angle(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "totalizer")) {
        rc = get_totalizer(sensor);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else {
        printk("Error: wrong command!\n");
        printk("jiangsu commands: config, channel, width, compensation, angle, totalizer\n");
        return -1;
    }
    return 0;
}

static int set_configs(struct smart_sensor *sensor, char *cmd, char *arg)
{
    int rc = 0;

    if (!strcmp(cmd, "channel")) {
        uint8_t channel_type = (uint8_t)atoi(arg);

        rc = set_channel_type(sensor, channel_type);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "width")) {
        float w = (float)atof(arg);

        rc = set_width(sensor, w);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "compensation")) {
        float c = (float)atof(arg);

        rc = set_bottom_compensation(sensor, c);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "angle")) {
        float a = (float)atof(arg);

        rc = set_angle(sensor, a);
        if (rc < 0) {
            printk("Error!!\n");
            return rc;
        }
    } else if (!strcmp(cmd, "totalizer")) {
        if (!strcmp(arg, "reset")) {
            rc = set_totalizer(sensor, 0.0);
            if (rc < 0) {
                printk("Error!!\n");
                return rc;
            }
        } else {
            double t = atof(arg);

            rc = set_totalizer(sensor, t);
            if (rc < 0) {
                printk("Error!!\n");
                return rc;
            }
        }
    } else {
        printk("Error: wrong command!\n");
        printk("jiangsu commands: config, channel, width, compensation, angle, totalizer\n");
        return -1;
    }
    return 0;
}

static int get_channel_type(struct smart_sensor *sensor)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, CHANNEL_TYPE_REG, 1);
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }

    uint8_t channel_type = f.data[0];

    switch (channel_type) {
        case 0:
            printk("Channel type: rectangulo\n");
            break;
        case 1:
            printk("Channel type: redondo\n");
            break;
        case 2:
            printk("Channel type: trapezoide\n");
            break;
        case 3:
            printk("Channel type: triangulo\n");
            break;
        default:
            /* we should never came here. */
            break;
    }

    return 0;
}

static int set_channel_type(struct smart_sensor *sensor, uint8_t ch_type)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS, CHANNEL_TYPE_REG, 1);
    f.data[0] = (uint16_t)ch_type;
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }

    return 0;
}

static int get_width(struct smart_sensor *sensor)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, WIDTH_DIMENSION_REG, 2);
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }
    /*frame is ok*/
    union floatingPointIEEE754 value;

    value.raw.msb = f.data[1];
    value.raw.lsb = f.data[0];
    printk("Width dimension: %.2fm\n", (double)value.f);

    return 0;
}

static int set_width(struct smart_sensor *sensor, float width)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS, WIDTH_DIMENSION_REG, 2);
    union floatingPointIEEE754 value;

    value.f = width;
    f.data[0] = value.raw.lsb;
    f.data[1] = value.raw.msb;
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }

    return 0;
}

static int get_bottom_compensation(struct smart_sensor *sensor)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, BOTTOM_COMPENSATION_REG, 2);
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }
    /*frame is ok*/
    union floatingPointIEEE754 value;

    value.raw.msb = f.data[1];
    value.raw.lsb = f.data[0];
    printk("Bottom compensation: %.2fm\n", (double)value.f);

    return 0;
}

static int set_bottom_compensation(struct smart_sensor *sensor, float comp)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS, BOTTOM_COMPENSATION_REG, 2);
    union floatingPointIEEE754 value;

    value.f = comp;
    f.data[0] = value.raw.lsb;
    f.data[1] = value.raw.msb;
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }

    return 0;
}

static int get_angle(struct smart_sensor *sensor)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, ANGLE_REG, 2);
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }
    /*frame is ok*/
    union floatingPointIEEE754 value;

    value.raw.msb = f.data[1];
    value.raw.lsb = f.data[0];
    printk("Angle: %.2f°\n", (double)value.f);

    return 0;
}

static int set_angle(struct smart_sensor *sensor, float angle)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS, ANGLE_REG, 2);
    union floatingPointIEEE754 value;

    value.f = angle;
    f.data[0] = value.raw.lsb;
    f.data[1] = value.raw.msb;
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }

    return 0;
}

static int get_totalizer(struct smart_sensor *sensor)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, SET_CUMULATIVE_FLOW_REG, 4);
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }
    /*frame is ok*/
    union DoubleIEEE754 value;

    value.raw.byte4 = f.data[3];
    value.raw.byte3 = f.data[2];
    value.raw.byte2 = f.data[1];
    value.raw.byte1 = f.data[0];
    printk("Totalizer: %.2fm3\n", value.d);

    return 0;
}

static int set_totalizer(struct smart_sensor *sensor, double totalizer_value)
{
    int rc;
    struct modbus_frame f;

    prepare_modbus_frame(&f, sensor, MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS, SET_CUMULATIVE_FLOW_REG, 4);
    union DoubleIEEE754 value;

    value.d = totalizer_value;
    f.data[0] = value.raw.byte1;
    f.data[1] = value.raw.byte2;
    f.data[2] = value.raw.byte3;
    f.data[3] = value.raw.byte4;
    modbus_query(UART_SMART_SENSOR, &f);
    rc = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);
    if (rc == -E_NOT_DETECTED) {
        return rc;
    }
    if (rc == -E_BAD_CHECKSUM) {
        return rc;
    }
    if (rc == -E_INVALID) {
        return rc;
    }

    return 0;
}

static int needs_external_voltage(void)
{
    return 1;
}
