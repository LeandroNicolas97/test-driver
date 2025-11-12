/**
 *  \file smart_sensor_calibration_ui.c
 *  \brief Calibration of smart sensors user interface
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 09.04.2020
 *
 *  Copyright 2020 Innovex Tecnologias Ltda. All rights reserved.
 */

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "config.h"
#include "serial.h"
#include "microio.h"
#include "display_fb.h"
#include "hardware.h"
#include "led.h"
#include "adc.h"
#include "debug.h"
#include "temperature.h"
#include "oxygen_saturation.h"
#include "smart_sensor.h"
#include "measurement.h"
#include "userinterface.h"
#include "errorcodes.h"

int calibrate_sensor(int sensor_nr);

/*
 * Functions for displaying. Move to another file so that they can be implemented for
 * different displays and different languages.
 */
void display_no_sensors_to_calibrate(void)
{
    display_printf("No hay sensores que puedan calibrarse\n");
    display_flush();
}

void display_do_you_want_to_calibrate_sensor(int sensor)
{
    display_printf("Sensor %i\npuede calibrarse\n", sensor);
    display_printf("Quiere calibrarlo\n");
    display_flush();
}

void display_sensor_not_calibrated(void)
{
    display_printf("No calibrado\n");
    display_flush();
}

void display_calibration_message(void)
{
    display_clear();
    display_printf("Calibracion\n");
}

void display_calibrating_sensor_nr(int sensor_nr)
{
    display_clear();
    display_printf("Calibrando sensor %i\n");
}

void display_waiting_measurement(struct measurement *measurement, int count)
{
    display_calibration_message();
    display_all_measurements(1, measurement, 1);
    display_printf("\nTiempo restante: %i\n", count);
    display_flush();
}

void display_calibration_running(void)
{
    display_calibration_message();
    display_printf("Calibrando...\n");
    display_flush();
}

void display_calibration_status(int status)
{
    if (status < 0) {
        display_printf("Error en la calibracion\n");
    } else {
        display_printf("Calibracion OK\n");
    }
    display_flush();
}

void normal_delay(void)
{
    sleep_microseconds(2000000);
}

/**
 * TODO Use some timing to identify yes or no
 */
int user_pressed_yes(int count)
{
    for (int i = 0; i < count; i++) {
        sleep_microseconds(100000);
        if (is_calib_switch_active()) {
            return 1;
        }
        led_toggle(LED_STATUS);
    }
    return 0;
}

/**
 * Check if there are sensors that can be calibrated and ask if the user wants to calibrate it.
 */
int calibrate_all_sensors(void)
{
    int calibrable_sensors = 0;
    int n_of_sensors_detected = total_sensors_detected();

    display_calibration_message();
    for (int i = 0; i < n_of_sensors_detected; i++) {
        if (smart_sensor_can_calibrate(i)) {
            calibrable_sensors++;
            display_do_you_want_to_calibrate_sensor(i);
            if (user_pressed_yes(30)) {
                calibrate_sensor(i);
            } else {
                display_sensor_not_calibrated();
                user_pressed_yes(20);
            }
        }
    }
    if (calibrable_sensors <= 0) {
        display_no_sensors_to_calibrate();
        return 0;
    }
    return 0;
}

/**
 * Calibrate one sensor
 */
int calibrate_sensor(int sensor_nr)
{
    struct measurement measurement;
    int must_calibrate = 0;

    display_calibrating_sensor_nr(sensor_nr);
    display_flush();
    turn_on_smart_sensor(0);
    smart_sensor_init_serial_port();
    const struct smart_sensor_driver *driver = driver_for_sensor(sensor_nr);
    struct smart_sensor *sensor = smart_sensor_get(sensor_nr);

    if (driver == NULL) {
        return 0;
    }
    driver->init_driver();
    driver->prepare(sensor);
    for (int count = 10; count > 0; count--) {
        watchdog_reset();
        driver->acquire(3, sensor, &measurement);
        display_waiting_measurement(&measurement, count);
        if (user_pressed_yes(10)) {
            must_calibrate = 1;
            break;
        }
    }
    if (must_calibrate) {
        display_calibration_running();
        int status = driver->calibrate_full(sensor);

        display_calibration_status(status);
    } else {
        display_sensor_not_calibrated();
    }
    driver->finish_driver();
    user_pressed_yes(20);
    return 0;
}
