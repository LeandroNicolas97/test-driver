/***************************************************************************
 *   file                 : configuration.h                                *
 *   begin                : Jul 01, 2019                                   *
 *   copyright            : (C) 2019 by Innovex Tecnologias Ltda.          *
 *   email                : pablo@innovex.cl                               *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2019. Innovex.                                          *
 ***************************************************************************/

/*
 * Configuration for the END_DEVICES
 */
#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

/* #include "motecommand.h" */
#include "measurement.h"
#include "oxygen_control.h"
#include "defaults.h"
#include "bsp-config.h"
#include "smart_sensor.h"

/**
 * Concentrator runtime configuration parameters. This variables are saved into
 * persistent storage.
 */
struct configuration {
    uint8_t channel;
    uint32_t uplink_channel;            /* The Uplink channel of the network */
    uint32_t downlink_channel;          /* The Downlink channel of the network */
    uint16_t sampling_interval;         /* Sampling interval in seconds */
    uint16_t wake_interval;             /* Wake up interval in seconds */
    uint16_t log_interval;              /* Seconds. Write a measurement log entry at this interval */
    uint16_t ping_interval;             /* Interval for pinging the pancoordinator. Seconds */
    char name[10];                      /* Identifier for this device */
    int16_t n_changes;                  /* If the EEPROM/FLASH was empty, we should get -1 here */
    float salinity;                     /* Fixed salinity for saturation concentration */
    float fixed_temperature;            /* Fixed temperature */
    float battery_coefficient;          /* Constant to convert ADC value to battery voltage */
    float v_ref;                        /* Calibrated value of the internal V reference */
    float low_oxygen_alarm;             /* If oxygen goes below this level is an alarm condition */
    float high_oxygen_alarm;            /* If oxygen goes above this level is an alarm condition */
    uint32_t sensor_powerup_time;       /* Time needed by the sensor to power up */
    uint8_t sensor_communication_tries; /* Number of times we try to communicate with a sensor */
    uint8_t lcd_contrast;               /* Contrast of the LCD */
    uint8_t use_saturation;
    enum sensor_type sensor_type[MAX_N_SENSORS];    /* Type of every configured sensor */
    struct valve_configuration valve[MAX_N_VALVES]; /* Configuration of the valves TODO Separate */
    uint16_t version;
    uint8_t command_state;
    uint8_t conductivity_freshwater;
    uint8_t distance;              /* Transmission rate distance */
    uint8_t bandwidth;             /* Lora bandwidth configuration */
    uint8_t datarate;              /* Lora datarate configuration */
    uint16_t time_on_air;          /* Time it takes to transmit */
    float temp_offset;             /* Offset for sensors without temp calibration */
    uint8_t current_sensor_status; /* Current valve sensor |on - off| */
    uint32_t totalized_flow;       /*variable that stores sensor config*/
    uint16_t total_volume;         /* Total volume to calculate percentage */
};

struct sensor_config {
    int16_t n_changes;
    const struct smart_sensor_driver *sensor_driver[SENSOR_MANUFACTURER_END];
};

/*
 * Node configuration. This is global.
 */
extern struct configuration cfg;
extern struct sensor_config sen_drv;

void set_default_configuration(void);
int write_configuration(void);
int write_sensor_configuration(void);
int read_nvs_data(void);
void set_current_time(uint32_t *time);
uint32_t get_current_time(void);
uint32_t get_timestamp(char *data);
void set_driver_default(void);

#endif /* _CONFIGURATION_H */
