/**
 *  \file smart_sensor_operations.c
 *  \brief Operations with smart sensors
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 08.02.2019
 *
 *  Copyright 2019 Innovex Tecnologias Ltda. All rights reserved.
 */

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "bsp-config.h"
#include "serial.h"
#include "microio.h"
#include "display_fb.h"
#include "hardware.h"
#include "adc.h"
#include "debug.h"
/* #include "temperature.h" */
#include "smart_sensor.h"
#include "measurement.h"
#include "errorcodes.h"
#include "watchdog.h"
/* #include "modbus.h" */
#include "sensor_power_hw.h"
#define HZ 100

/**
 * List of smart sensors detected
 */
static struct smart_sensor sensor[MAX_EXTERNAL_SENSORS];
static uint8_t sensors_detected;
static int preheat_time;

int smart_sensors_detect_voltage(void)
{
    enum sensor_manufacturer manufacturer;

    for (manufacturer = MANUFACTURER_NONE; manufacturer < SENSOR_MANUFACTURER_END; manufacturer++) {
        watchdog_reset();
        const struct smart_sensor_driver *driver = driver_for_manufacturer(manufacturer);

        if (driver != NULL) {
            if (driver->needs_external_voltage()) {
                return 1;
            }
        }
    }
    return 0;
}

/**
 * Detect all the sensors connected to the serial port
 * @return the number of sensors detected, negative on error.
 */
int smart_sensors_detect_all(void)
{
    int n_of_sensors_detected = 0;
    int line = 0;
    /*    turn_on_smart_sensor(0); */
    /* adc_init(); */
    enum sensor_manufacturer manufacturer;

    for (manufacturer = MANUFACTURER_NONE; manufacturer < SENSOR_MANUFACTURER_END; manufacturer++) {
        watchdog_reset();
        const struct smart_sensor_driver *driver = driver_for_manufacturer(manufacturer);

        if (driver != NULL) {
            /* There is a driver for this sensor */
            watchdog_reset();
            display_clear();
            display_printf("Detecting sensors\n%s\n", driver->name());
            int max_sensors = driver->max_sensors();

            driver->init_driver();
            for (int i = 0; i < max_sensors; i++) {
                /*                display_move(0, line*8); */
                display_printf("Sensor %i: ", i);
                watchdog_reset();
                int sensor_supply_mv = adc_read_sensor_supply();

                display_printf("%.1fV ", ((double)sensor_supply_mv) / 1000.0);
                if (sensor_supply_mv < 4000) {
                    display_printf("bajo "); /* Voltage too low */
                }
                struct smart_sensor *s = &(sensor[n_of_sensors_detected]);

                if (driver->detect(i, s)) {
                    /* A sensor has been detected */
                    s->manufacturer = manufacturer;
                    /* Find the biggest power up time */
                    if (preheat_time < s->power_up_time) {
                        preheat_time = s->power_up_time;
                    }
                    display_printf("OK", i);
                    n_of_sensors_detected++;
                } else {
                    display_printf("no");
                }
                display_printf("\n");
                display_flush();
                line++;
            }
            driver->finish_driver();
            watchdog_reset();
            sleep_microseconds(500000);
        }
        if (n_of_sensors_detected > 0 && manufacturer == LUFFT) {
            break;
        } else if (n_of_sensors_detected > 0 && manufacturer == VAISALA) {
            break;
        } else if (n_of_sensors_detected > 0 && manufacturer == NORTEK) {
            break;
        } else if (n_of_sensors_detected > 0 && manufacturer == ACCONEER) {
            break;
        } else if (n_of_sensors_detected > 0 && manufacturer == WITMOTION) {
            break;
        }

        /* comment this line, so we can read sensors from more than 1 manufacturer */
        /* if(n_of_sensors_detected > 0) break;  // Once a manufacturer was detected, continue */
    }
    /*    turn_off_smart_sensor(0); */
    sensors_detected = n_of_sensors_detected;
    return n_of_sensors_detected;
}

int get_sensors_preheat_time_ms(void)
{
    return preheat_time;
}
/**
 * Get the total number of sensors detected.
 */
int total_sensors_detected(void)
{
    return sensors_detected;
}

/**
 * Get a pointer to a smart sensor which has been detected.
 */
struct smart_sensor *smart_sensor_get(uint8_t sensor_number)
{
    if (sensor_number >= sensors_detected) {
        return NULL;
    } else {
        return &(sensor[sensor_number]);
    }
}

/**
 * Prepare all the smart sensor
 */
void smart_sensor_prepare_all(int n_of_sensors)
{
    const struct smart_sensor_driver *driver;

    for (int i = 0; i < n_of_sensors; i++) {
        watchdog_reset();
        driver = driver_for_sensor(i);
        if (driver != NULL) {
            driver->init_driver();
            driver->prepare(&(sensor[i]));
        }
    }
}

/**
 * Get the driver for the specified sensor
 * @param The sensor number
 * @return The driver for the sensor, or NULL if there is no driver
 */
const struct smart_sensor_driver *driver_for_sensor(int sensor_number)
{
    enum sensor_manufacturer manufacturer = sensor[sensor_number].manufacturer;
    const struct smart_sensor_driver *driver = driver_for_manufacturer(manufacturer);
    return driver;
}

/**
 * Check if the smart sensor can be calibrated.
 * If the driver provides functionality to calibrate the sensor, this return true
 * @return 0 if the driver cannot calibrate the sensor. 1 if it can.
 */
int smart_sensor_can_calibrate(int sensor_number)
{
    int ret = 0;
    const struct smart_sensor_driver *driver = driver_for_sensor(sensor_number);

    if (driver != NULL) {
        if (driver->calibrate_full != NULL) {
            ret = 1;
        }
    }
    return ret;
}

/**
 * Calibrate a smart sensor with the specified value
 * @param sensor_number The number of the sensor to calibrate
 * @param cal_value The actual value of the parameter to calibrate.
 * @return 1 if calibrated, 0 if not, negative on error.
 */
int smart_sensor_calibrate(int sensor_number, float cal_value)
{
    int calibrated = 0;
    /* turn_on_smart_sensor(0); */
    sensor_power_on(smart_sensors_detect_voltage());
    smart_sensor_init_serial_port();
    DEBUG("Calibrando Sensor %i: valor: %f\n", sensor_number, (double)cal_value);
    if (smart_sensor_can_calibrate(sensor_number)) {
        /* TODO use the cal_value to calibrate */
        int calstatus = driver_for_sensor(sensor_number)->calibrate_full(&(sensor[sensor_number]));
        /* calibration status == 0 means OK */
        if (calstatus == 0) {
            calibrated = 1;
        } else if (calstatus < 0) {
            calibrated = calstatus; /* Error */
        }
    }
    serial_tx_disable(UART_SMART_SENSOR);
    /* turn_off_smart_sensor(0); */
    sensor_power_off(smart_sensors_detect_voltage());
    return calibrated;
}

/**
 * Acquire all the smart sensors and store the measuruemnts in the specified array.
 * @param n_of_sensor The number of sensors to read
 * @param measurement a pointer to store all the measurements
 * @return the number of measurements acquired
 */
int smart_sensors_aquire_all(int n_of_sensors, int communication_tries, struct measurement *measurement)
{
    int n_of_sensors_acquired = 0;
    const struct smart_sensor_driver *driver;

    for (int i = 0; i < n_of_sensors; ++i) {
        watchdog_reset();
        driver = driver_for_sensor(i);

        if (driver != NULL) {
            driver->init_driver();
            if (driver->acquire(communication_tries, &(sensor[i]), &(measurement[i]))) {
                n_of_sensors_acquired++;
            }
            driver->finish_driver();
        }
        watchdog_reset();
    }
    return n_of_sensors_acquired;
}

/**
 * Check if the configuration of the sensor has changed.
 * It is only important to have the same sequence of sensor types.
 * If the list of sensors detected is the same as the list of sensor types
 * provided, then the configuration has not changed.
 * @param n_of_sensors The number of sensor to check
 * @param configured_sensor An array with the types of sensors configured.
 * @return true if all configured sensors are the same as the detected sensors.
 */
int has_sensor_list_changed(int n_of_sensors, enum sensor_type *configured_sensor)
{
    int ret = 1;

    for (int i = 0; i < n_of_sensors; ++i) {
        if (configured_sensor[i] != sensor[i].type) {
            ret = 0;
            break;
        }
    }
    return ret;
}
