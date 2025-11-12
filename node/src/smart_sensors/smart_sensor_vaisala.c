
/***************************************************************************
 *   file                 : smart_sensor_vaisala.c                        *
 *   begin                : Sept, 2022                                     *
 *   copyright            : (C) 2011 by Innovex Tecnologias Ltda.          *
 *   Author               : Edgar Osorio                                   *
 *   email: edgar.osorio@innovex.cl                                                                      *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2011. Innovex.                                          *
 ***************************************************************************/
/*
 * Communication with the smart sensors Weather Station Lufft
 */

#include <string.h>
#include <stdlib.h>
#include "bsp-config.h"
#include "smart_sensor.h"
#include "serial.h"
#include "errorcodes.h"
#include "hardware.h"
#include "modbus.h"
#include "debug.h"
#include "configuration.h"

#define DETECTION_TRIES   2
#define MAX_SENSORS       1
#define MAX_RESPONSE_SIZE 128

#define VAISALA_SENSOR_SLAVE_ADDR 0x46

#define DEPRECATED 0

/**
 * Enum for diferent type of sensors
 */
enum vaisala_sensor_type {
    VAISALA_WXT530,
    VAISALA_END
};

struct weather_sensor_vaisala {
    int16_t relative_humidity_avg; /* % */
    int16_t rel_air_pressure_avg;  /* hPa */
    int16_t global_radiation_avg;  /* W/m2 */
    int16_t air_temperature_avg;   /* oC */
    int16_t dew_point_avg;         /* oC */
    int16_t wind_speed_avg;        /* m/s */
    int16_t wind_direction;
    int16_t gust;           /* m/s */
    int16_t gust_direction; /* m/s */
    int16_t precipitation_type;
    int16_t precipitation_abs;    /* mm/h */
    int16_t precipitation_diff;   /* mm/h */
    int16_t precipitation_intens; /* mm/h */
};

/**
 * Driver function prototypes
 */
static int max_sensors(void);                                      /* Maximum number of sensors for this driver */
static const char *name(void);                                     /* Name of the driver */
static int init_driver(void);                                      /* Initialize the driver */
static int finish_driver(void);                                    /* Finish the operations with the driver */
static int detect(int sensor_number, struct smart_sensor *sensor); /* Detect a sensor */
static int prepare(struct smart_sensor *sensor);                   /* Prepare the sensor */
/* static int finish(struct smart_sensor *sensor); // Finish the communication with the sensor */
/* static int calibrate_zero(struct smart_sensor *sensor); // Calibrate the zero of the sensor */
/* static int calibrate_full(struct smart_sensor *sensor); // Calibrate the full scale of the sensor */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m);
/* static int pass_command(struct smart_sensor *sensor, char *command); */
static int needs_external_voltage(void);

/**
 * Smart sensor driver structure for the Yosemitech sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_vaisala = {
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

#if DEPRECATED
/*
 * Local variables
 */
struct modbus modbus_response, modbus_request;
uint16_t modbus_data_buffer[MAX_RESPONSE_SIZE];
#endif

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
    return "Vaisala Weather";
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
 *
 */
static int prepare(struct smart_sensor *sensor)
{
    struct modbus_frame f;
    int response_status;
    int number_registers;
    int start_address;

    start_address = 13;
    number_registers = 10;

    DEBUG("Star Register: 0x%.4x Size Register: %i\n", start_address, number_registers);
    DEBUG("Nro Sensor: %i\n", sensor->number);
    prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, start_address, number_registers);
    serial_flush(UART_SMART_SENSOR);
    modbus_query(UART_SMART_SENSOR, &f);
    response_status = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);

    if (response_status == -E_NOT_DETECTED) {

        return 1;
    }
    if (response_status == -E_BAD_CHECKSUM) {

        return 1;
    }
    if (response_status == -E_INVALID) {

        return 1;
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

    DEBUG("Checking Vaisala Weather Station %i...\n", sensor_number);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        switch (sensor->number) {
            case 0:
                sensor->type = VAISALA_WXT530;
                break;

            default:
                sensor->type = VAISALA_END;
                break;
        }
        if (prepare(sensor) == 0) {
            sensor->manufacturer = VAISALA;
            sensor->power_up_time = VAISALA_POWERUP_TIME;
            sensor->channel = 0;             /* Currently we have only one channel */
            strcpy(sensor->name, "VAISALA"); /* TODO: make this string from the sensor number */
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
 *
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg,
                                 uint16_t coils)
{
    switch (sensor->number) {
        case 0:
            f->slave_address = VAISALA_SENSOR_SLAVE_ADDR;
            break;

        default:
            f->slave_address = 1;
            break;
    }
    f->function_code = function;
    f->register_address = reg;
    f->n_coils = coils;
}

#if DEPRECATED
/*
 * Used for little-endian sensors. Is like memcpy, but inverse.
 * TODO: move elsewhere.
 */
static void *revmemcpy(void *dest, const void *src, size_t len)
{
    char *d = (char *)dest + len - 1;
    const char *s = src;

    while (len--) {
        *d-- = *s++;
    }
    return dest;
}

static void rawhex2float(const float *f, const uint16_t *c)
{
    char *p = (char *)f;
    char *q = (char *)c;

    revmemcpy(p, q, 2);
    revmemcpy(p + 2, q + 2, 2);
}
#endif

/*
 *
 */
static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    struct modbus_frame f;
    struct weather_sensor_vaisala wxt530;
    int response_status;
    int number_registers;
    int start_address;
    /* serial_flush(UART_SMART_SENSOR); */
    /* modbus_query(UART_SMART_SENSOR, &f); */
    sleep_microseconds(20000);

    start_address = 0x0D;
    number_registers = 18;
    DEBUG("Star Register: 0x%.4x Size Register: %i\n", start_address, number_registers);
    DEBUG("Nro Sensor: %i\n", sensor->number);
    prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, start_address, number_registers);
    serial_flush(UART_SMART_SENSOR);
    modbus_query(UART_SMART_SENSOR, &f);
    sleep_microseconds(10000);
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

    wxt530.relative_humidity_avg = f.data[0];
    wxt530.rel_air_pressure_avg = f.data[4];
    wxt530.gust_direction = f.data[7];
    wxt530.wind_direction = f.data[8];
    wxt530.precipitation_type = f.data[12];
    wxt530.global_radiation_avg = f.data[17];

    start_address = 0x22;
    number_registers = 18;
    uint8_t tries = 5;

    DEBUG("Star Register: 0x%.4x Size Register: %i\n", start_address, number_registers);
    DEBUG("Nro Sensor: %i\n", sensor->number);
    while (tries > 0) {
        prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, start_address, number_registers);
        serial_flush(UART_SMART_SENSOR);
        modbus_query(UART_SMART_SENSOR, &f);
        response_status = modbus_poll(UART_SMART_SENSOR, &f, BIG_ENDIAN);

        if (response_status == -E_NOT_DETECTED) {
            measurement->sensor_status = SENSOR_NOT_DETECTED;
            tries--;
        } else if (response_status == -E_BAD_CHECKSUM) {
            measurement->sensor_status = SENSOR_COMMUNICATION_BAD_CRC;
            tries--;
        } else if (response_status == -E_INVALID) {
            measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
            tries--;
        } else {
            break;
        }
        if (tries <= 0) {
            return 0;
        }
    }

    wxt530.air_temperature_avg = f.data[0];
    wxt530.dew_point_avg = f.data[4];
    wxt530.wind_speed_avg = f.data[11];
    wxt530.gust = f.data[13];
    wxt530.precipitation_abs = f.data[14];
    wxt530.precipitation_diff = f.data[15];
    wxt530.precipitation_intens = f.data[16];
    struct weather_station_measurement *m = &(measurement->weather_station);

    m->air_temperature = (float)wxt530.air_temperature_avg / 10;
    m->pressure = (float)wxt530.rel_air_pressure_avg / 10;
    m->relative_humidity = (float)wxt530.relative_humidity_avg / 10;
    m->average_wind = (float)wxt530.wind_speed_avg / 10;
    m->average_direction = (float)wxt530.wind_direction / 10;
    m->wind_gusts = (float)wxt530.gust / 10;
    m->gusts_direction = (float)wxt530.gust_direction / 10;
    m->precipitation_abs = (float)wxt530.precipitation_abs / 100;
    m->precipitation_diff = (float)wxt530.precipitation_diff / 100;
    m->precipitation_intens = (float)wxt530.precipitation_intens / 100;
    m->radiation = (float)wxt530.global_radiation_avg / 10;

    DEBUG("Relative Humidity Avg %0.3f\n", (double)m->relative_humidity);
    DEBUG("Real air pressure Avg %0.3f\n", (double)m->pressure);
    DEBUG("Gobal Radiation Avg %0.3f\n", (double)m->radiation);
    DEBUG("Air Temperature Avg(oC) %0.3f\n", (double)m->air_temperature);
    DEBUG("Dew point Avg(oC) %0.3f\n", (double)wxt530.dew_point_avg / 10);
    DEBUG("Win Speed Avg(m/s) %0.3f\n", (double)m->average_wind);
    DEBUG("Precipitation Type %i\n", wxt530.precipitation_type);
    DEBUG("Precipitation abs %0.3f(mm)\n", (double)m->precipitation_abs);
    DEBUG("Precipitation diff %0.3f(mm)\n", (double)m->precipitation_diff);
    DEBUG("Precipitation intens %0.3f(mm/h)\n", (double)m->precipitation_intens);
    DEBUG("Gust %0.3f\n", (double)m->wind_gusts);
    DEBUG("Win Direction Vect %0.3f\n", (double)m->average_direction);
    DEBUG("Gust Direction  %0.3f\n", (double)m->gusts_direction);

    measurement->sensor_status = SENSOR_OK;
    measurement->type = WEATHER_STATION_SENSOR;

    m->pressure_status = MEASUREMENT_OK;
    m->relative_humidity_status = MEASUREMENT_OK;
    m->average_wind_status = MEASUREMENT_OK;
    m->average_direction_status = MEASUREMENT_OK;
    m->wind_gusts_status = MEASUREMENT_OK;
    m->gusts_direction_status = MEASUREMENT_OK;
    m->precipitation_status = MEASUREMENT_OK;
    return 1;
}

static int needs_external_voltage(void)
{
    return 1;
}
