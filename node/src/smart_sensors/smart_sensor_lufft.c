
/***************************************************************************
 *   file                 : smart_sensor_lufft.c                        *
 *   begin                : Sept, 2022                                     *
 *   copyright            : (C) 2022 by Innovex Tecnologias Ltda.          *
 *   Author               : Edgar Osorio                                   *
 *   email: edgar.osorio@innovex.cl                                                                      *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2022. Innovex.                                          *
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
#include "microio.h"
#include "crc16.h"
#include "hardware.h"
#include "timeutils.h"
#include "modbus.h"
#include "debug.h"
#include "configuration.h"

#define DETECTION_TRIES   2
#define MAX_SENSORS       2
#define MAX_RESPONSE_SIZE 128

#define WS501UMB_SENSOR_SLAVE_ADDR 0x03
#define WS100_SLAVE_ADDR           0x02

#define DEPRECATED 0

/**
 * Enum for diferent type of sensors
 */
enum lufft_sensor_type {
    LUFFT_WS501UMB,
    LUFFT_WS100,
    LUFFT_END
};

struct weather_sensor_lufft {
    int16_t relative_humidity_avg; /* 0-100% */
    int16_t rel_air_pressure_avg;  /* 300-1200 hPa */
    int16_t global_radiation_avg;  /* 0-200 W/m2 */
    int16_t air_temperature_avg;   /* -50 -60 oC */
    int16_t dew_point_avg;         /* -50-60 oC */
    int16_t wind_speed_avg;        /* 0-75 m/s */
    int16_t wind_direction_vect;   /* 0-359 o */
    int16_t gust;                  /* 0-75 m/s */
    int16_t gust_direction;        /* 0-359 o */
    int16_t precipitation_type;
    int16_t precipitation_abs;    /* 0- 100 000 mm */
    int16_t precipitation_diff;   /* 0-100 000 mm */
    int16_t precipitation_intens; /* 0-200 mm/h */

    int16_t compass;
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

/**
 * Smart sensor driver structure for the Yosemitech sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_lufft = {
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
    return "Lufft Weather";
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
    int response_status;
    struct modbus_frame f;

    serial_flush(UART_SMART_SENSOR);
    switch (sensor->type) {
        case LUFFT_WS501UMB:
            DEBUG("Star Register: 0x%.4x Size Register: %i\n", 13, 10);
            DEBUG("Nro Sensor: %i\n", sensor->number);
            prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, 13, 10);
            break;
        case LUFFT_WS100:
            DEBUG("Star Register: 0x%.4x Size Register: %i\n", 159, 5);
            DEBUG("Nro Sensor: %i\n", sensor->number);
            prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, 159, 5);
            break;
        default:
            prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, 13, 10);
            break;
    }
    serial_flush(UART_SMART_SENSOR);
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

    DEBUG("Checking Lufft Weather Station %i...\n", sensor_number);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        switch (sensor->number) {
            case 0:
                sensor->type = LUFFT_WS501UMB;
                break;
            case 1:
                sensor->type = LUFFT_WS100;
                break;
            default:
                sensor->type = LUFFT_END;
                break;
        }
        if (prepare(sensor) == 0) {
            sensor->manufacturer = LUFFT;
            sensor->power_up_time = LUFFT_POWERUP_TIME;
            sensor->channel = 0;           /* Currently we have only one channel */
            strcpy(sensor->name, "LUFFT"); /* TODO: make this string from the sensor number */
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
            f->slave_address = WS501UMB_SENSOR_SLAVE_ADDR;
            break;
        case 1:
            f->slave_address = WS100_SLAVE_ADDR;
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

int read_modbus_register(struct modbus_frame *f, struct measurement *measurement, struct smart_sensor *sensor,
                         u_int16_t slave_address, u_int16_t start_address, u_int16_t *buffer)
{
    int response_status;

    prepare_modbus_frame(f, sensor, MODBUS_READ_INPUT_REGISTERS, start_address, 1);
    f->slave_address = slave_address;
    serial_flush(UART_SMART_SENSOR);
    modbus_query(UART_SMART_SENSOR, f);
    response_status = modbus_poll(UART_SMART_SENSOR, f, BIG_ENDIAN);

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
    *buffer = f->data[0];

    return 1;
}

/*
 *
 */
int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    struct weather_station_measurement *m = &(measurement->weather_station);
    struct modbus_frame f;
    struct weather_sensor_lufft ws501umb;
    int status;
    int response_status;

    m->air_temperature_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->pressure_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->relative_humidity_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->average_wind_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->average_direction_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->wind_gusts_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->gusts_direction_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->precipitation_status = MEASUREMENT_ACQUISITION_FAILURE;
    m->radiation_status = MEASUREMENT_ACQUISITION_FAILURE;
    sleep_microseconds(20000);

    if (sensor->number == 0) {
        /* read wind_direction and gust direction */
        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 18, &ws501umb.wind_direction_vect);
        status =
            read_modbus_register(&f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 22, &ws501umb.gust_direction);

        /* read wrelative_humidity and rel_air_pressure */
        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 10, &ws501umb.relative_humidity_avg);
        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 14, &ws501umb.rel_air_pressure_avg);

        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 10, &ws501umb.relative_humidity_avg);
        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 14, &ws501umb.rel_air_pressure_avg);

        /* read air_temperature */

        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 31, &ws501umb.air_temperature_avg);
        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 31, &ws501umb.air_temperature_avg);

        /* read global_radiation */
        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 30, &ws501umb.global_radiation_avg);
        status = read_modbus_register(
            &f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 30, &ws501umb.global_radiation_avg);

        /* read wind speed and gust */
        status =
            read_modbus_register(&f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 45, &ws501umb.wind_speed_avg);
        status = read_modbus_register(&f, measurement, sensor, WS501UMB_SENSOR_SLAVE_ADDR, 44, &ws501umb.gust);

        if (status < 0) {
            return 0;
        }
        measurement->sensor_status = SENSOR_OK;
        measurement->type = WEATHER_STATION_SENSOR;
        m->air_temperature = (float)ws501umb.air_temperature_avg / 10;
        m->pressure = (float)ws501umb.rel_air_pressure_avg / 10;
        m->relative_humidity = (float)ws501umb.relative_humidity_avg / 10;
        m->average_wind = (float)ws501umb.wind_speed_avg / 10;
        m->average_direction = (float)ws501umb.wind_direction_vect / 10;
        m->wind_gusts = (float)ws501umb.gust / 10;
        m->gusts_direction = (float)ws501umb.gust_direction / 10;
        m->precipitation_abs = (float)ws501umb.precipitation_abs / 100;
        m->precipitation_diff = (float)ws501umb.precipitation_diff / 100;
        m->precipitation_intens = (float)ws501umb.precipitation_intens / 100;
        m->radiation = (float)ws501umb.global_radiation_avg / 10;
        m->pressure_status = MEASUREMENT_OK;
        m->relative_humidity_status = MEASUREMENT_OK;
        m->average_wind_status = MEASUREMENT_OK;
        m->average_direction_status = MEASUREMENT_OK;
        m->wind_gusts_status = MEASUREMENT_OK;
        m->gusts_direction_status = MEASUREMENT_OK;
        m->radiation_status = MEASUREMENT_OK;
        sleep_microseconds(20000);
    } else if (sensor->number == 1) {
        DEBUG("Nro Sensor: %i\n", sensor->number);
        prepare_modbus_frame(&f, sensor, MODBUS_READ_INPUT_REGISTERS, 159, 5);
        f.slave_address = WS100_SLAVE_ADDR;
        DEBUG("Star Register: 0x%.4x Size Register: %i  Slave:%i\n", 159, 5, WS100_SLAVE_ADDR);
        serial_flush(UART_SMART_SENSOR);
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
        measurement->sensor_status = SENSOR_OK;
        measurement->type = WEATHER_STATION_SENSOR;
        ws501umb.precipitation_type = f.data[0];
        ws501umb.precipitation_abs = f.data[1];
        ws501umb.precipitation_diff = f.data[2];
        ws501umb.precipitation_intens = f.data[3];
        m->precipitation_status = MEASUREMENT_OK;
    }

    DEBUG("Relative Humidity Avg() %0.3f, %i\n",
          (double)ws501umb.relative_humidity_avg / 10,
          ws501umb.relative_humidity_avg);
    DEBUG("Real air pressure Avg(hPa) %0.3f, %i\n",
          (double)ws501umb.rel_air_pressure_avg / 10,
          ws501umb.rel_air_pressure_avg);
    DEBUG("Win Direction Vect %0.3f, %i\n", (double)ws501umb.wind_direction_vect / 10, ws501umb.wind_direction_vect);
    DEBUG("Gust Direction  %0.3f, %i\n", (double)ws501umb.gust_direction / 10, ws501umb.gust_direction);
    DEBUG("Gobal Radiation Avg(W/m2) %0.3f, %i\n",
          (double)ws501umb.global_radiation_avg / 10,
          ws501umb.global_radiation_avg);
    DEBUG(
        "Air Temperature Avg(oC) %0.3f, %i\n", (double)ws501umb.air_temperature_avg / 10, ws501umb.air_temperature_avg);
    DEBUG("Win Speed Avg(m/s) %0.3f, %i\n", (double)ws501umb.wind_speed_avg / 10, ws501umb.wind_speed_avg);
    DEBUG("Gust Speed Avg(m/s) %0.3f, %i\n", (double)ws501umb.gust / 10, ws501umb.gust);
    DEBUG("Precipitation Type %i\n", ws501umb.precipitation_type);
    DEBUG("Precipitation abs %0.3f(mm)\n", (double)ws501umb.precipitation_abs / 100);
    DEBUG("Precipitation diff %0.3f(mm)\n", (double)ws501umb.precipitation_diff / 100);
    DEBUG("Precipitation intens %0.3f(mm/h)\n", (double)ws501umb.precipitation_intens / 100);

    sleep_microseconds(20000);

    return 1;
}
