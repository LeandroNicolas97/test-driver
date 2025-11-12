/**
 *  \file oxycontroller.c
 *  \brief Concentrator for different sensors and controller
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 26 October 2009
 *
 *  Copyright 2009-2021 Innovex Tecnologias Ltda. All rights reserved.
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/state.h>
#include "configuration.h"
#include "hardware.h"
#include "measurement.h"
#include "serial.h"
#include "microio.h"
#include "smart_sensor.h"
#include "sensor_power_hw.h"
#include "display_fb.h"
#include "bsp-config.h"
#include "debug.h"
#include "userinterface.h"
#include "radio.h"
#include "led.h"
#include "multishell.h"
#include "shell_commands.h"
#include "actual_conditions.h"
#include "sampling.h"
#include "oxygen_control.h"
#include "solenoid.h"
#include "adc.h"
#include "local_sensors.h"
#include "watchdog.h"
#include "measurement_storage.h"
#include "comunication.h"
#if CONFIG_EXTERNAL_DATALOGGER
#include "external_datalogger.h"
#include "compressed_measurement.h"
#endif

/* Local prototypes */
static void serialize_and_send_measurements(char *data, size_t size);
static void processes_init(void);
static void should_wake(int tics);
static void serialiaze_and_send_node(void);
/* static const struct device *get_si7006_device(void); */

/* This has to be defined for DEBUG to work */
/* enum debug_level debug_level = DEBUG_LEVEL_NO_MESSAGES; */
enum debug_level debug_level = DEBUG_LEVEL_DEBUG;

/*
 * Device state
 */
struct oxycontroller_state actual_state = {
    .warm_starts = 0,
    .total_scans = 0,
    .calib_pressed = 0,
    .quick_response = 0,
    .n_of_sensors_detected = 0,
    .display_on = 1,
    .has_solenoid_control = 0,
    .start_testing = 0,
    .uptime_at_last_ping = 0,
    .can_sleep = 0,
    .coordinator_found = 0,
    .missed_conection = 0,
    .using_external_memory = 0,

};

struct measurement actual_measurements[MAX_EXTERNAL_SENSORS + 1 + MAX_N_VALVES];
struct measurement node_measurement;
struct measurement valve_measurements[MAX_N_VALVES];
static uint32_t time_of_last_measurement; /* zero-initialized by C */
const struct device *si7007_dev;

int main(void)
{
    uint8_t radio_state = SLEEPING;

    processes_init();
    display_set_auto_flush(0);
    led_off(0);
    uint32_t elapsed = 0;

    while (1) {
        if (should_start_sampling(cfg.sampling_interval)) {
            /* Wait for local command */
            should_wake(2000000);
            display_driver_periodic_refresh();
            DEBUG("Sampling...\n");
            sensor_power_on(smart_sensors_detect_voltage());
            watchdog_reset();
            sleep_microseconds(500000);
            led_on(0);
            sampling(3, actual_state.n_of_sensors_detected, actual_measurements);
            time_of_last_measurement = get_current_time();
            if (!smart_sensors_detect_voltage()) {
                check_oxygen_levels_all_valves(cfg.use_saturation, actual_measurements);
            }
            acquire_local_sensors(&node_measurement, valve_measurements);
            sensor_power_off(smart_sensors_detect_voltage());
            /* Start radio communication */
            char data[255];

            if (check_for_adcp()) {
                struct smart_sensor *s = smart_sensor_get(0);

                send_adcp_measurements(time_of_last_measurement, s->manufacturer);
                serialiaze_and_send_node();
            } else {
                send_ping();
                serialize_and_send_measurements(data, sizeof(data));
            }

            radio_state = if_received_data(data);
            if (radio_state == RECEIVING) {
                radio_state = receiving_commands(data);
            }
            init_and_clear_lcd();
            display_end_device_status(node_measurement.node.battery_voltage);
            display_all_measurements(actual_state.n_of_sensors_detected, actual_measurements, cfg.use_saturation);
            display_flush();
            sensor_power_off(smart_sensors_detect_voltage());
            led_off(0);
            rs485_sleep(UART_SMART_SENSOR);
#if CONFIG_EXTERNAL_DATALOGGER
            int total_measurements = actual_state.n_of_sensors_detected;
            struct compressed_measurement_list *list = k_malloc(256);
            int compressed_size;

            compressed_size = compress_measurement_list(actual_measurements, total_measurements, 256, list);
            list->timestamp = time_of_last_measurement;
            DEBUG("Storing %i bytes from actual_measurements\n", compressed_size);
            watchdog_reset();
            datalogger_append((uint8_t *)list, 256);
            k_free(list);
            /* DEBUG("Free space: %i\n", datalogger_get_free_space()); */
#endif
            elapsed = k_uptime_get();
            DEBUG("--- elapsed: %i\n", elapsed); /* in msec */
            DEBUG("Sleeping for %iseg\n", cfg.sampling_interval);
        } else {
            int32_t sleep_time = k_uptime_get() - elapsed;
            uint32_t interval = 1000 * ((cfg.sampling_interval * 1000) - sleep_time);

            if (sleep_time > (cfg.sampling_interval * 1000)) {
                interval = 0;
            }
            if (interval < 0 || interval > (cfg.ping_interval * 1000000)) {
                interval = cfg.ping_interval * 1000000;
            }
            display_flush();
            watchdog_disable();
            sleep_microseconds(interval);
            watchdog_init();
            if (cfg.sampling_interval > cfg.ping_interval) {
                send_ping();
            }
        }
    }
    return 0;
}

static void processes_init(void)
{
    leds_init();
    led_on(0);
    serial_init(UART_SMART_SENSOR, 9600, 0, 0, 0);
    serial_init(COMM_UART, 11520, 0, 0, 0);
    microio_init(COMM_UART, COMM_UART);
    radio_init();
    adc_init();
    watchdog_init();
    /* INJECTION */
    int status = solenoid_init();

    DEBUG("Solenoid init %i\n", status);
    if (status == SOLENOID_OK) {
        actual_state.has_solenoid_control = 1;
    }
    oxygen_control_init(&(cfg.valve[0]));
    solenoid_release();

    read_nvs_data();

    if (cfg.n_changes < 0) {
        set_default_configuration();
        valves_set_default_configuration();
    }
    shell_init(command_list, cfg.name);
    init_and_clear_lcd();
    display_welcome_message();
    watchdog_disable();
    sleep_microseconds(1000000);
    display_clear();
    display_flush();
    display_set_auto_flush(1);

    if (sen_drv.n_changes < 0) {
        set_driver_default();
    }

    detect_sensors();
    processes_init_complete();
}

static void serialize_and_send_measurements(char *data, size_t size)
{
    size_t n_size = 110;
    int n_active_valves = 0;
    struct smart_sensor s;
    int sensor_number = 0;
    /* send valves measurements here if they are active */
    actual_measurements[actual_state.n_of_sensors_detected] = node_measurement;
    for (int i = 0; i < MAX_N_VALVES; i++) {
        if (cfg.valve[i].is_active) {
            n_active_valves++;
            actual_measurements[actual_state.n_of_sensors_detected + n_active_valves] = valve_measurements[i];
        }
    }
    /* add n_of_sensors_detected+1 for NODE measurement */
    for (int i = (actual_state.n_of_sensors_detected + n_active_valves); i >= 0; --i) {
        if (i >= actual_state.n_of_sensors_detected) {
            sensor_number = actual_measurements[i].sensor_number;
        } else if (i < actual_state.n_of_sensors_detected) {
            s = *smart_sensor_get(i);
            sensor_number = s.number;
        }
        int pos = usnprintf(data, size, ":%u:%s:%i:", time_of_last_measurement, cfg.name, sensor_number);

        serialize_measurement(&(actual_measurements[i]), 255, &(data[pos]));
        if (i == actual_state.n_of_sensors_detected) {
            if (is_channel_free()) {
                send_frame(data, size);
                check_acknowledgment(data, cfg.name, time_of_last_measurement);
            }
        } else {
            measurement_storage_append(data, n_size);
        }
    }
    if (is_channel_free()) {
        send_data_from_storage(time_of_last_measurement);
    }
}

static void should_wake(int tics)
{
    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint8_t c;

    printk("Wake?\n");
    for (int j = 0; j < tics; ++j) {
        watchdog_reset();
        while (uart_poll_in(dev, &c) >= 0) {
            shell_char_received(c);
        }
    }
    while (cfg.command_state == WAKE) {
        watchdog_reset();
        while (uart_poll_in(dev, &c) >= 0) {
            shell_char_received(c);
        }
    }
    printk("Timeout\n");
}

int cmd_debug_level(char *str)
{
    if (!str) {
        printk("Debug %i\n", debug_level);
    } else {
        if (!strcmp("on", str)) {
            debug_level = DEBUG_LEVEL_DEBUG;
            printk("Debug activado\n");
        } else if (!strcmp("off", str)) {
            debug_level = DEBUG_LEVEL_NO_MESSAGES;
            printk("Debug desactivado\n");
        } else {
            printk("Enter on or off\n");
        }
    }
    return 0;
}

int detect_sensors(void)
{
    configure_sensor_drivers();
    int rc = measurement_storage_mount();

    if (rc != 0) {
        DEBUG("Measurement storage mount error: %i\n", rc);
    }
#if CONFIG_EXTERNAL_DATALOGGER
    rc = datalogger_mount();
    if (rc == 0) {
        DEBUG("External memory mount OK\n");
    } else {
        DEBUG("External memory mount ERROR %i\n", rc);
    }
#endif
    local_sensors_init();
    int status = sensor_power_init();

    DEBUG("Sensor power status %i\n", status);

    if (actual_state.has_solenoid_control) {
        detect_all_valves();
    } else {
        display_printf("No solenoid control.\n");
    }
    if (cfg.command_state == SLEEP) {
        should_wake(2000000);
    }
    watchdog_init();
    restore_meas_unit_flag();
    sensor_power_on(smart_sensors_detect_voltage());
    actual_state.n_of_sensors_detected = smart_sensors_detect_all();
    sensor_power_off(smart_sensors_detect_voltage());

    return 0;
}

static void serialiaze_and_send_node(void)
{
    char buffer[256];
    int pos = usnprintf(
        buffer, sizeof(buffer), ":%u:%s:%i:", time_of_last_measurement, cfg.name, node_measurement.sensor_number);
    serialize_measurement(&node_measurement, 255, &(buffer[pos]));
    if (is_channel_free()) {
        send_frame(buffer, 255);
        check_acknowledgment(buffer, cfg.name, time_of_last_measurement);
    }
}
