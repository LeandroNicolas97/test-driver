/**
 *  \file smart_sensor_driver.c
 *  \brief Driver for connecting with smart sensors
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 08.02.2019
 *
 *  Copyright 2019 Innovex Tecnologias Ltda. All rights reserved.
 */

#include <stdlib.h>
#include "smart_sensor.h"
#include "zephyr/sys/printk.h"
#include "configuration.h"
#include "shell_commands.h"

const struct smart_sensor_driver *sensor_driver[] = {
    [NORTEK] = NULL,
    [LUFFT] = NULL,
    [VAISALA] = NULL,
    [INNOVEX] = &smart_sensor_driver_innovex,
    [MAXBOTIX] = NULL,
    [PONSEL] = NULL,
    [TEXAS_INSTRUMENTS] = NULL,
    [YOSEMITECH] = NULL,
    [AQUAS] = NULL,
    [YSI] = NULL,
    [HUIZHONG] = NULL,
    [TELEDYNE_ISCO] = NULL,
    [ANBSENSORS] = NULL,
    [TDS100] = NULL,
    [GPS] = NULL,
    [CHEMINS] = NULL,
    [SEABIRD] = NULL,
    [JIANGSU] = NULL,
    [ACCONEER] = NULL,
    [AQUADOPP] = NULL,
    [FLOWQUEST] = NULL,
    [WITMOTION] = NULL,
};

/**
 * Get the driver for the specified manufacturer
 * @param manufacturer The manufacturer of the sensor
 * @return The driver for the sensor or NULL if there is no driver
 */
const struct smart_sensor_driver *driver_for_manufacturer(enum sensor_manufacturer manufacturer)
{
    if (manufacturer > MANUFACTURER_NONE && manufacturer < SENSOR_MANUFACTURER_END) {
        return sensor_driver[manufacturer];
    }
    return NULL;
}

void sensor_switch(enum sensor_manufacturer manufacturer, int state)
{
    static const struct smart_sensor_driver *const sensor_driver_directions[] = {
        [INNOVEX] = &smart_sensor_driver_innovex,
        [NORTEK] = &smart_sensor_driver_signature_nortek,
        [FLOWQUEST] = &smart_sensor_driver_flowquest,
        [PONSEL] = &smart_sensor_driver_ponsel,
        [YOSEMITECH] = &smart_sensor_driver_yosemitech,
        [YSI] = &smart_sensor_driver_ysi,
        [VAISALA] = &smart_sensor_driver_vaisala,
        [TDS100] = &smart_sensor_driver_tds100,
        [HUIZHONG] = &smart_sensor_driver_huizhong,
        [TELEDYNE_ISCO] = &smart_sensor_driver_signature_flow,
        [ANBSENSORS] = &smart_sensor_driver_anb,
        [SEABIRD] = &smart_sensor_driver_seabird,
        [CHEMINS] = &smart_sensor_driver_chemins,
        [JIANGSU] = &smart_sensor_driver_jiangsu_flow,
        [ACCONEER] = &smart_sensor_driver_xm126,
        [AQUADOPP] = &smart_sensor_driver_aquadopp_nortek,
        [WITMOTION] = &smart_sensor_driver_wtvb01};

    if (state == ACTIVATE) {
        sensor_driver[manufacturer] = sensor_driver_directions[manufacturer];
        if (manufacturer == INNOVEX && sensor_driver[NORTEK] != NULL) {
            sensor_driver[NORTEK] = NULL;
            sen_drv.sensor_driver[NORTEK] = sensor_driver[NORTEK];
            printk("nortek deactivated.\n");
        } else if (manufacturer == NORTEK) {
            if (sensor_driver[INNOVEX] != NULL) {
                sensor_driver[INNOVEX] = NULL;
                sen_drv.sensor_driver[INNOVEX] = sensor_driver[INNOVEX];
                printk("innovex deactivated.\n");
            }
        }
    } else {
        sensor_driver[manufacturer] = NULL;
    }
    sen_drv.sensor_driver[manufacturer] = sensor_driver[manufacturer];
}

void configure_sensor_drivers(void)
{
    for (int i = 0; i < SENSOR_MANUFACTURER_END; i++) {
        sensor_driver[i] = sen_drv.sensor_driver[i];
    }
}
