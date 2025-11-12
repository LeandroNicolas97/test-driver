#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/drivers/lora.h>
#include "configuration.h"
#include "oxygen_control.h"
#include "radio.h"
#include "microio.h"
#include "actual_conditions.h"
#include "display_fb.h"
#include "hardware.h"
#include "zephyr/sys/printk.h"
#include "errorcodes.h"
#include "version.h"
#include "smart_sensor.h"
#include "sensor_power_hw.h"
#include "serial.h"
#include "adc.h"
#include "watchdog.h"
#include "led.h"
#include "measurement_storage.h"
#include "shell_commands.h"
#include <stdio.h>
#if CONFIG_EXTERNAL_DATALOGGER
#include "compressed_measurement.h"
#include "external_datalogger.h"
#endif

struct shell_command command_list[] = {
    {"status",            show_status                        },
    {"ok",                wake_up                            },
    {"sleep",             to_sleep                           },
    {"name",              set_name                           },
    {"interval",          set_sampling_interval              },
    {"contrast",          set_display_contrast               },
    {"commit",            save_configuration                 },
    {"reboot",            cmd_reboot                         },
    {"injection",         cmd_injection                      },
    {"usesat",            cmd_set_saturation_conf            },
    {"channel",           cmd_set_channel                    },
    {"distance",          cmd_set_distance                   },
    {"mac",               cmd_get_mac_address                },
    {"tunnel",            cmd_tunnel                         },
    {"temp_offset",       cmd_temp_offset                    },
    {"date",              cmd_date                           },
    {"current",           cmd_current_sensor_status          },
    {"erase",             cmd_erase_data_flash               },
    {"debug",             cmd_debug_level                    },
    {"totalized",         cmd_set_totalized                  },
    {"driver",            cmd_set_sensor_driver              },
#if CONFIG_EXTERNAL_DATALOGGER
    {"datalogger_dump",   cmd_external_flash_datalogger_dump },
    {"datalogger_format", cmd_externalflash_datalogger_format},
    {"put",               cmd_external_flash_datalogger_put  },
#endif
    /* Jiangsu ultrasonic flowmeter configurations */
    {"jiangsu",           cmd_jiangsu_config                 },
    {"savedrivers",       cmd_set_sensor_config              },
    {"detect",            cmd_detect_sensors                 },
    {"volume",            cmd_volume_porcentage              },
    {0,                   0                                  }
};

static uint8_t should_detect; /* zero-initialized by C */

/**
 * Show the status
 */
int show_status(char *str)
{
    char buffer[140];
    size_t size = sizeof(buffer);
    float v_bat = (adc_read_battery() / 1000.0);

    printk("Name: %s\n", cfg.name);
    printk("Interval: %i\n", cfg.sampling_interval);
    printk("Channel: %i\n", cfg.channel);
    printk("Cambios de configuracion: %i\n", cfg.n_changes);
    printk("Version %s\n", VERSION_STRING);
    printk("Microlib version %s\n", MICROLIB_VERSION_STRING);
    usnprintf(buffer,
              size,
              "%s %s: %i\n %s: %i\n %s: %i\n %s: %s\n %s: %s\n %s: %.3f",
              cfg.name,
              "Interval",
              cfg.sampling_interval,
              "Channel",
              cfg.channel,
              "Use saturation",
              cfg.use_saturation,
              "Version",
              VERSION_STRING,
              "Microlib",
              MICROLIB_VERSION_STRING,
              "Bat",
              (double)v_bat);
    radio_send_str(buffer, strlen(buffer) + 1);
    return 0;
}

/**
 * Wake up
 */
int wake_up(char *str)
{
    printk("Wake!\n");
    cfg.command_state = WAKE;
    return 0;
}

/**
 * To sleep
 */
int to_sleep(char *str)
{
    printk("To sleep\n");
    cfg.command_state = SLEEP;
    write_configuration();
    return 0;
}

/**
 * Set the name
 */
int set_name(char *str)
{
    char buffer[20];
    size_t size = sizeof(buffer);
    char old_name[10] = {0};

    if (!str) {
        printk("%s\n", cfg.name);
        usnprintf(buffer, size, "%s %s %s", cfg.name, "Name", cfg.name);
        radio_send_str(buffer, strlen(buffer) + 1);
    } else {
        strncpy(old_name, cfg.name, 10);
        strncpy(cfg.name, str, 10);
        printk("Name set: %s\n", cfg.name);
        usnprintf(buffer, size, "%s %s %s", old_name, "Name", cfg.name);
        radio_send_str(buffer, strlen(buffer) + 1);
    }
    return 0;
}

/**
 * Set sampling interval
 */
int set_sampling_interval(char *str)
{
    char buffer[20];
    size_t size = sizeof(buffer);

    if (!str) {
        printk("Interval: %i\n", cfg.sampling_interval);
        usnprintf(buffer, size, "%s %s %i", cfg.name, "Interval", cfg.sampling_interval);
        radio_send_str(buffer, strlen(buffer) + 1);
    } else {
        cfg.sampling_interval = atol(str);
        printk("Set interval in: %i\n", cfg.sampling_interval);
    }
    return 0;
}

/**
 * Set display contrast
 */
int set_display_contrast(char *str)
{
    char buffer[20];
    size_t size = sizeof(buffer);

    if (!str) {
        printk("Contrast: %i\n", cfg.lcd_contrast);
        usnprintf(buffer, size, "%s %s %i", cfg.name, "Contrast", cfg.lcd_contrast);
        radio_send_str(buffer, strlen(buffer) + 1);
    } else {
        cfg.lcd_contrast = atol(str);
        printk("Set contrast in: %i\n", cfg.lcd_contrast);
        display_driver_set_contrast(cfg.lcd_contrast);
        display_clear();
    }
    return 0;
}

/*
 * Reboot the system
 */
int cmd_reboot(char *s)
{
    char buffer[20];
    size_t size = sizeof(buffer);

    if (!s) {
        cfg.command_state = SLEEP;
        write_configuration();
        usnprintf(buffer, size, "%s %s", cfg.name, "Rebooting...");
        radio_send_str(buffer, strlen(buffer) + 1);
        printk("Rebooting...\n");
        soft_reboot();
    }
    return 0;
}

/**
 * Save configuration
 */
int save_configuration(char *str)
{
    char buffer[20];
    size_t size = sizeof(buffer);

    write_configuration();
    printk("Commit ok\n");
    usnprintf(buffer, size, "%s %s", cfg.name, "OK");
    radio_send_str(buffer, strlen(buffer) + 1);
    return 0;
}

/*
 * Set use saturation or concentration.
 */
int cmd_set_saturation_conf(char *str)
{
    if (!str) {
        printk("Use saturation %i\n", cfg.use_saturation);
    } else {
        uint8_t use_saturation = (uint8_t)atoi(str);

        if (use_saturation != 1 && use_saturation != 0) {
            use_saturation = 0;
        }
        cfg.use_saturation = use_saturation;
    }
    return 0;
}

/*
 * Prepare the injection levels command.
 * The format of the string is:
 * injection <valve> <sensor> <type> <mode> <open level> <close level>
 */
int cmd_injection(char *args)
{
    /* char buffer[80]; */
    /* size_t size = sizeof(buffer); */
    struct valve_configuration injection;

    int valve_nr = 0;

    if (args) {
        char *arg;
        char *s = args;
        float level;

        arg = strtok_r(s, " ", &s); /* Extract valve number */
        if (arg == NULL) {
            return -E_INVALID;
        }
        valve_nr = atol(arg);
        if (valve_nr < 1 || valve_nr > 4) {
            return -E_INVALID;
        }
        valve_nr -= 1; /* The system counts from 0 */
        /* We have a first argument, it can still be a GET command */
        arg = strtok_r(s, " ", &s); /* Extract associated sensor */
        if (arg == NULL) {
            /* No more arguments, it means GET configuration */
            goto get_injection;
        }

        injection.associated_sensor = atol(arg);
        if (injection.associated_sensor < 1 || injection.associated_sensor > 4) {
            return -E_INVALID;
        }
        injection.associated_sensor--;

        arg = strtok_r(s, " ", &s); /* Extract  valve type */
        if (arg == NULL) {
            return -E_INVALID;
        }
        if (!strcmp("bistable", arg)) {
            injection.valve_type = VALVE_BISTABLE;
        } else if (!strcmp("inverse", arg)) {
            injection.valve_type = VALVE_BISTABLE_INVERSE;
        } else if (!strcmp("no", arg)) {
            injection.valve_type = VALVE_NORMALLY_OPEN;
        } else if (!strcmp("nc", arg)) {
            injection.valve_type = VALVE_NORMALLY_CLOSE;
        } else {
            return -E_INVALID;
        }

        arg = strtok_r(s, " ", &s); /* Extract injection mode */
        if (arg == NULL) {
            return -E_INVALID;
        }
        if (!strncmp(arg, "off", 3)) {
            injection.injection_mode = INJECTION_DISABLED;
        } else if (!strncmp(arg, "auto", 4)) {
            injection.injection_mode = INJECTION_AUTO;
        } else if (!strncmp(arg, "on", 2)) {
            injection.injection_mode = INJECTION_FORCE_ON;
        } else {
            return -E_INVALID;
        }

        arg = strtok_r(s, " ", &s); /* Extract open level */
        if (arg == NULL) {
            return -E_INVALID;
        }
        string_to_float(arg, &level);
        if (level < 0.01f) {
            return -E_INVALID;
        }
        injection.injection_open_level = level;

        arg = strtok_r(s, " ", &s); /* Extract close level */
        if (arg == NULL) {
            return -E_INVALID;
        }
        string_to_float(arg, &level);
        if (level < injection.injection_open_level || level > 150.0f) {
            return -E_INVALID;
        }
        injection.injection_close_level = level;

        injection.is_active = 1;
        injection.solenoid_pulse_length = 100000;
        injection.valve_number_of_pulses = 3;
        valve_set_configuration(valve_nr, &injection);
        return 0;
    }

get_injection:

    valve_get_configuration(valve_nr, &injection);
    printk("Valvula: %i\n", valve_nr + 1);
    printk("Sensor asociado: %i\n", injection.associated_sensor + 1);
    printk("Injection mode: %i\n", injection.injection_mode);
    printk("Valve type: %i\n", injection.valve_type);
    printk("Open level: %.2f\n", (double)injection.injection_open_level);
    printk("Close level: %.2f\n", (double)injection.injection_close_level);
    return 0;
}

/*
 * Set channel of network
 */
int cmd_set_channel(char *args)
{
    if (!args) {
        printk("Channel %i\n", cfg.channel);
        printk("Frequency uplink %i Downlink %i\n", cfg.uplink_channel, cfg.downlink_channel);
        return 0;
    } else {
        uint8_t channel = atoi(args);

        if (channel < 0 || channel > 7) {
            printk("Configure channels between 0 and 7\n");
            return 0;
        }

        char buffer[20];
        size_t size = sizeof(buffer);

        usnprintf(buffer, size, "%s %s %i", cfg.name, "Channel", channel);
        radio_send_str(buffer, strlen(buffer) + 1);

        radio_init();
        cfg.channel = channel;
        switch (cfg.channel) {
            case 0:
                cfg.downlink_channel = CHANNEL_DOWNLINK_0;
                cfg.uplink_channel = CHANNEL_UPLINK_64;
                break;
            case 1:
                cfg.downlink_channel = CHANNEL_DOWNLINK_1;
                cfg.uplink_channel = CHANNEL_UPLINK_65;
                break;
            case 2:
                cfg.downlink_channel = CHANNEL_DOWNLINK_2;
                cfg.uplink_channel = CHANNEL_UPLINK_66;
                break;
            case 3:
                cfg.downlink_channel = CHANNEL_DOWNLINK_3;
                cfg.uplink_channel = CHANNEL_UPLINK_67;
                break;
            case 4:
                cfg.downlink_channel = CHANNEL_DOWNLINK_4;
                cfg.uplink_channel = CHANNEL_UPLINK_68;
                break;
            case 5:
                cfg.downlink_channel = CHANNEL_DOWNLINK_5;
                cfg.uplink_channel = CHANNEL_UPLINK_69;
                break;
            case 6:
                cfg.downlink_channel = CHANNEL_DOWNLINK_6;
                cfg.uplink_channel = CHANNEL_UPLINK_70;
                break;
            case 7:
                cfg.downlink_channel = CHANNEL_DOWNLINK_7;
                cfg.uplink_channel = CHANNEL_UPLINK_71;
                break;
        }
        cfg.command_state = SLEEP;
        write_configuration();
        printk("Set channel %i\n", cfg.channel);
        soft_reboot();
        return 0;
    }
}

int cmd_set_distance(char *str)
{
    char buffer[20];
    size_t size = sizeof(buffer);

    if (!str) {
        printk("Distance %i\n", cfg.distance);
        printk("BW: %i, SF: %i\n", cfg.bandwidth, cfg.datarate);
        usnprintf(buffer, size, "%s %s %i", cfg.name, "Distance", cfg.distance);
        radio_send_str(buffer, strlen(buffer) + 1);
        return 0;
    } else {
        uint8_t distance = atoi(str);
        char buffer[20];
        size_t size = sizeof(buffer);

        usnprintf(buffer, size, "%s %s %i", cfg.name, "Distance", distance);
        radio_send_str(buffer, strlen(buffer) + 1);

        cfg.distance = distance;
        switch (distance) {
            case 0:
                cfg.bandwidth = BW_500_KHZ;
                cfg.datarate = SF_7;
                cfg.time_on_air = 33;
                break;
            case 1:
                cfg.bandwidth = BW_500_KHZ;
                cfg.datarate = SF_11;
                cfg.time_on_air = 350;
                break;
            case 2:
                cfg.bandwidth = BW_250_KHZ;
                cfg.datarate = SF_11;
                cfg.time_on_air = 699;
                break;
        }
        cfg.command_state = SLEEP;
        write_configuration();
        printk("Set distance %i\n", cfg.distance);
        soft_reboot();
        return 0;
    }
}

int cmd_get_mac_address(char *str)
{
    struct mac_address mac;
    char mac_address[48];
    char buffer[50];

    get_mac_address(&mac);
    mac_address_to_string(&mac.dev_id, mac.length, mac_address);
    printk("Mac %s\n", mac_address);

    size_t size = sizeof(buffer);

    usnprintf(buffer, size, "%s %s", cfg.name, mac_address);
    radio_send_str(buffer, strlen(buffer) + 1);
    return 0;
}

int cmd_tunnel(char *str)
{
    int response_status;
    char sensor_response[255];
    char buffer[255];
    const struct smart_sensor_driver *driver = driver_for_manufacturer(INNOVEX);

    printk("power on\n");
    sensor_power_on(smart_sensors_detect_voltage());
    watchdog_disable();
    sleep_microseconds(500000);
    watchdog_init();
    serial_flush(UART_SMART_SENSOR); /* Clean the receive buffer */
    driver->init_driver();
    printk("Tunnel: %s--\n", str);
    watchdog_reset();
    driver->pass_command(NULL, str);
    while (1) {
        watchdog_reset();
        response_status = smart_sensor_get_response(sensor_response, sizeof(sensor_response));
        printk("Response %s\n", sensor_response);
        usnprintf(buffer, sizeof(buffer), "%s %s", cfg.name, sensor_response);
        radio_send_str(buffer, strlen(buffer) + 1);
        if (response_status < 0) {
            break;
        }
    }
    /* sensor_power_off(); */
    return 0;
}

int cmd_current_sensor_status(char *str)
{
    char buffer[25];
    size_t size = sizeof(buffer);

    if (!str) {
        if (cfg.current_sensor_status == 0) {
            printk("Sensor current OFF\n");
            usnprintf(buffer, size, "%s %s", cfg.name, "Sensor current OFF");
            radio_send_str(buffer, strlen(buffer) + 1);
        } else if (cfg.current_sensor_status == 1) {
            printk("Sensor current ON\n");
            usnprintf(buffer, size, "%s %s", cfg.name, "Sensor current ON");
            radio_send_str(buffer, strlen(buffer) + 1);
        }
    } else {
        if (!strncmp(str, "off", 3)) {
            printk("Desactivando sensor\n");
            cfg.current_sensor_status = 0;
            solenoid_prepare();
            solenoid_activate_reverse(0);
            watchdog_reset();
            sleep_microseconds(100000);
            solenoid_release();
            usnprintf(buffer, size, "%s %s", cfg.name, "current OFF");
            radio_send_str(buffer, strlen(buffer) + 1);
        } else if (!strncmp(str, "on", 2)) {
            printk("Activando sensor\n");
            cfg.current_sensor_status = 1;
            solenoid_prepare();
            solenoid_activate_forward(0);
            watchdog_reset();
            sleep_microseconds(100000);
            solenoid_release();
            usnprintf(buffer, size, "%s %s", cfg.name, "current ON");
            radio_send_str(buffer, strlen(buffer) + 1);
        } else {
            printk("Enter on or off\n");
            usnprintf(buffer, size, "%s %s", cfg.name, "Enter on or off");
            radio_send_str(buffer, strlen(buffer) + 1);
        }
    }
    return 0;
}

/**
 * Set temperature offset
 */
int cmd_temp_offset(char *str)
{
    char buffer[30];
    size_t size = sizeof(buffer);

    if (!str) {
        printk("Temp offset: %.2f\n", (double)cfg.temp_offset);
        usnprintf(buffer, size, "%s %s %.2f", cfg.name, "temp offset", (double)cfg.temp_offset);
        radio_send_str(buffer, strlen(buffer) + 1);
    } else {
        float offset;

        string_to_float(str, &offset);
        cfg.temp_offset = offset;
        printk("Set temp offset to: %.2f\n", (double)cfg.temp_offset);
    }
    return 0;
}

/**
 *
 */
int cmd_date(char *str)
{
    uint32_t time;

    if (!str) {
        time = get_current_time();
        printk("Time: %i\n", time);
    } else {
        time = atol(str);
        if (time > 1548000000) {
            set_current_time(&time);
            printk("Set time\n");
        }
    }
    return 0;
}

int cmd_erase_data_flash(char *str)
{
    int rc = 0;

    printk("Erase flash data\n");
    measurement_storage_format();
    rc = measurement_storage_mount();
    if (rc != 0) {
        printk("Measurement storage mount error: %i\n", rc);
    }
    return 0;
}

int cmd_set_sensor_driver(char *str)
{
    static const char *const sensor_names[] = {
        [INNOVEX] = "innovex",
        [NORTEK] = "nortek",
        [PONSEL] = "ponsel",
        [YOSEMITECH] = "yosemitech",
        [YSI] = "ysi",
        [VAISALA] = "vaisala",
        [TDS100] = "tds100",
        [HUIZHONG] = "huizhong",
        [TELEDYNE_ISCO] = "teledyne",
        [ANBSENSORS] = "anbsensors",
        [SEABIRD] = "seabird",
        [CHEMINS] = "chemins",
        [JIANGSU] = "jiangsu",
        [ACCONEER] = "acconeer",
        [AQUADOPP] = "aquadopp",
        [FLOWQUEST] = "flowquest",
        [WITMOTION] = "wtvb01",
        NULL /* Sentinel value */
    };

    /* If no string provided, print current states */
    if (!str) {
        printk("\nSensor states:\n");
        for (int i = 0; i < SENSOR_MANUFACTURER_END; i++) {
            if (sensor_names[i]) {
                const char *status = (sen_drv.sensor_driver[i] != NULL) ? "active" : "inactive";
                char buffer[50];
                size_t size = sizeof(buffer);

                usnprintf(buffer, size, "%s %s: %s", cfg.name, sensor_names[i], status);
                radio_send_str(buffer, strlen(buffer) + 1);
            }
        }
        return 0;
    }

    /* Check for on/off commands */
    for (int i = 0; i < SENSOR_MANUFACTURER_END; i++) {
        if (!sensor_names[i]) {
            continue;
        }
        char on_cmd[32], off_cmd[32];

        snprintf(on_cmd, sizeof(on_cmd), "%s on", sensor_names[i]);
        snprintf(off_cmd, sizeof(off_cmd), "%s off", sensor_names[i]);

        if (strcmp(str, on_cmd) == 0) {
            printk("%s turned ON\n", sensor_names[i]);
            sensor_switch(i, ACTIVATE);

            char buffer[20];
            size_t size = sizeof(buffer);

            usnprintf(buffer, size, "%s %s ON\n", cfg.name, sensor_names[i]);
            radio_send_str(buffer, strlen(buffer) + 1);

            return 0;
        } else if (strcmp(str, off_cmd) == 0) {
            printk("%s turned OFF\n", sensor_names[i]);
            sensor_switch(i, DEACTIVATE);
            char buffer[20];
            size_t size = sizeof(buffer);

            usnprintf(buffer, size, "%s %s OFF\n", cfg.name, sensor_names[i]);
            radio_send_str(buffer, strlen(buffer) + 1);

            return 0;
        }
    }
    /* If no match found, show usage information */
    printk("\nCommand not found.\n");
    printk("The following commands are available:\n");
    printk("\n * '1 driver' to check the current state.\n");
    printk(" * '1 driver [brand] on/off' to activate or deactivate sensor.\n");
    printk("\nThe following brands of sensors are available:\n");
    for (int i = 0; i < SENSOR_MANUFACTURER_END; i++) {
        if (!sensor_names[i]) {
            continue;
        }
        printk(" * %s\n", sensor_names[i]);
    }
    return 0;
}

int cmd_set_sensor_config(char *str)
{
    write_sensor_configuration();
    char buffer[20];
    size_t size = sizeof(buffer);

    if (!str) {
        usnprintf(buffer, size, "%s %s", cfg.name, "Drivers saved...\n");
        radio_send_str(buffer, strlen(buffer) + 1);
        printk("Sensor drivers saved succesfully\n");
    }

    return 0;
}

#if CONFIG_EXTERNAL_DATALOGGER
/**
 * Serialize a measurement with its time-stamp, mote_name name and measurement number.
 */
static void serialize_full_measurement(uint32_t timestamp, char *mote_name, int meas_number, struct measurement *m,
                                       char *buffer, size_t buffsize)
{
    int pos = usnprintf(buffer, buffsize, ":%u:%s:%i:", timestamp, mote_name, meas_number);

    serialize_measurement(m, buffsize - pos, &(buffer[pos]));
}

int cmd_external_flash_datalogger_dump(char *str)
{
    printk("START\n");
    struct compressed_measurement_list *list = k_malloc(256);

    printk("dataloger_dump unsended data: %d\n", datalogger_unsended_data_get());
    for (int i = datalogger_unsended_data_get(); i > 0; --i) {
        watchdog_reset();
        datalogger_get((uint8_t *)list, 256, i - 1);
        struct measurement measurement;
        int n_of_measurements = list->n_of_elements;
        uint8_t *p = &(list->list[0]);

        for (int j = 0; j < n_of_measurements; j++) {
            watchdog_reset();
            int compressed_size = extract_measurement_from_compressed_list(p, &measurement);

            if (compressed_size < 0) {
                printk("Corrupted measurements\n");
                break;
            }
            p += compressed_size;
            char buffer[128];

            serialize_full_measurement(
                list->timestamp, "ASDF", measurement.sensor_number, &measurement, buffer, sizeof(buffer));
            printk("%s\n", buffer);
        }
    }
    k_free(list);
    printk("END\n");
    return 0;
}

int cmd_externalflash_datalogger_format(char *str)
{
    int rc = 0;

    watchdog_disable();
    printk("Erase external datalogger flash. Wait a 1 minute\n");
    datalogger_format();
    watchdog_init();

    rc = datalogger_mount();
    if (rc != 0) {
        printk("Measurement storage mount error: %i\n", rc);
    }
    return 0;
}

int cmd_external_flash_datalogger_put(char *str)
{
    struct measurement measurement[1];
    struct compressed_measurement_list *list = k_malloc(110);

    char *frame;

    frame = strtok_r(str, ":", &str);
    uint32_t timestamp = atoi(frame);

    frame = strtok_r(str, ":", &str);
    frame = strtok_r(str, ":", &str);
    uint8_t sensor_number = atoi(frame);

    int status = deserialize_measurement(str, &measurement[0]);

    if (!status) {
        printk("Frame error\n");
        return 0;
    }
    measurement[0].sensor_number = sensor_number;
    compress_measurement_list(measurement, 1, 110, list);
    list->timestamp = timestamp;
    datalogger_append((uint8_t *)list, 110);
    k_free(list);
    return 0;
}
#endif

int cmd_set_totalized(char *str)
{
    char buffer[20];
    size_t size = sizeof(buffer);

    if (!str) {
        printk("Totalized: %i\n", cfg.totalized_flow);
        usnprintf(buffer, size, "%s %s %i", cfg.name, "Totalized", cfg.totalized_flow);
        radio_send_str(buffer, strlen(buffer) + 1);
    } else if (!strncmp("reset", str, 5)) {
        cfg.totalized_flow = 0;
        printk("Reset totalized\n");
    } else {
        cfg.totalized_flow = atol(str);
        printk("Set totalized in: %i\n", cfg.totalized_flow);
    }
    return 0;
}

int cmd_jiangsu_config(char *str)
{
    const struct smart_sensor_driver *driver = driver_for_manufacturer(JIANGSU);

    if (driver == NULL) {
        /* TODO: test it!!! */
        printk("No driver para jiangsu\n");
        return 0;
    }
    serial_flush(UART_SMART_SENSOR); /* Clean the receive buffer */
    driver->init_driver();
    watchdog_reset();
    driver->pass_command(NULL, str);
    return 0;
}

int cmd_detect_sensors(char *str)
{
    char buffer[20];
    size_t size = sizeof(buffer);

    usnprintf(buffer, size, "%s %s", cfg.name, "Detecting\n");
    radio_send_str(buffer, strlen(buffer) + 1);
    if (should_detect) {
        detect_sensors();
    }
    cfg.command_state = SLEEP;
    return 0;
}

void processes_init_complete(void)
{
    should_detect = 1;
}

int cmd_volume_porcentage(char *str)
{
    char buffer[20];
    size_t size = sizeof(buffer);

    if (!str) {
        printk("Total Volume: %i\n", cfg.total_volume);
        usnprintf(buffer, size, "%s %s %i", cfg.name, "Total volume", cfg.total_volume);
        radio_send_str(buffer, strlen(buffer) + 1);
    } else {
        cfg.total_volume = atol(str);
        printk("Set total volume in: %i\n", cfg.total_volume);
    }
    return 0;
}
