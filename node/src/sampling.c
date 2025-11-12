/**
 *  \file sampling.c
 *  \brief Data collection from all the connected sensors
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 28 June 2019
 *
 *  Copyright 2019 Innovex Tecnologias Ltda. All rights reserved.
 */
#include "bsp-config.h"
#include "adc.h"
#include "hardware.h"
#include "debug.h"
#include "smart_sensor.h"
#include "measurement_operations.h"
#include "timeutils.h"
#include "sampling.h"
#include "watchdog.h"
#include "local_sensors.h"
#include "configuration.h"

#define DEPRECATED 0

static uint64_t uptime_at_last_sample; /* zero-initialized by C; The uptime at the moment we did the last acquisition */

/**
 * Collect all the samples
 */
int sampling(int communication_tries, int n_of_sensors, struct measurement *measurements)
{
    /* Smart sensors */
    DEBUG("Acquiring %i external sensors\n", n_of_sensors);
    if (n_of_sensors > 0) {
        smart_sensor_prepare_all(n_of_sensors);
        watchdog_disable();
        uint32_t t = get_sensors_preheat_time_ms() * 1000;

        DEBUG("Preheating sensors for: %i us\n", t);
        sleep_microseconds(t);
        smart_sensors_aquire_all(n_of_sensors, communication_tries, measurements);
        watchdog_init();
        measurements_join_oxygen_with_salinity(n_of_sensors, measurements);
        measurements_list_calculate_oxygen_concentration(n_of_sensors, measurements);
        measurements_join_two_levels(n_of_sensors, measurements);
        measurements_join_current_ac(n_of_sensors, measurements);
        average_oil_level(n_of_sensors, measurements);
        gets_totalized_flow_measurement(n_of_sensors, measurements);
    }
    uptime_at_last_sample = get_uptime_ms();
    return 0;
}

/**
 * Check if we it should start a sampling process.
 */
int should_start_sampling(uint32_t sampling_interval)
{
    DEBUG("Should sample: %lli\n", get_uptime_ms() - uptime_at_last_sample);
    if (uptime_at_last_sample == 0) {
        return 1; /* Force sampling at the beginning */
    }
    if ((get_uptime_ms() - uptime_at_last_sample) > (sampling_interval * 1000)) {
        return 1;
    }
#if DEPRECATED
    if (is_calib_switch_active()) {
        return 1;
    }
#endif
    return 0;
}

/**
 * TODO move out from this .cfg
 */
void acquire_local_sensors(struct measurement *node_measurement, struct measurement *valve_measurements)
{
    uint16_t humidity;
    float temperature;
    int v_bat;
    int v_sensor;

    node_measurement->type = NODE_INTERNAL_SENSOR;
    node_measurement->sensor_number = 0;
    v_bat = adc_read_battery();
    v_sensor = adc_read_sensor_supply();
    node_measurement->node.battery_voltage = ((float)v_bat / 1000.0f);
    node_measurement->node.sensor_voltage = ((float)v_sensor / 1000.0f);
    watchdog_reset();
    int ret = local_sensors_get_hum_and_temp(&humidity, &temperature);

    if (ret < 0) {
        node_measurement->node.temperature = 0.0;
        node_measurement->node.humidity = 0;
    } else {
        node_measurement->node.temperature = temperature;
        node_measurement->node.humidity = humidity;
    }
    node_measurement->node.signal_quality = 0;
    node_measurement->node.injection_open_level = 0;
    node_measurement->node.injection_close_level = 0;
    node_measurement->node.injection_mode = 0;
    node_measurement->node.valve_open = 0;
    /* Get the valve state and configurations for jenreceiver */
    for (int i = 0; i < MAX_N_VALVES; i++) {
        valve_measurements[i].type = VALVE_SENSOR;
        valve_measurements[i].sensor_number = i + 1;
        valve_measurements[i].valve.valve_nr = i + 1;
        valve_measurements[i].valve.associated_sensor = cfg.valve[i].associated_sensor + 1;
        valve_measurements[i].valve.valve_type = cfg.valve[i].valve_type;
        valve_measurements[i].valve.injection_mode = cfg.valve[i].injection_mode;
        valve_measurements[i].valve.injection_open_level = cfg.valve[i].injection_open_level;
        valve_measurements[i].valve.injection_close_level = cfg.valve[i].injection_close_level;
        valve_measurements[i].valve.valve_open = valve_in_open_state(i);
    }
}
