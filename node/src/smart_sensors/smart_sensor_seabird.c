/*
 * Communication with the Seabird sensors
 */

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/_stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include "bsp-config.h"
#include "hardware.h"
#include "measurement.h"
#include "smart_sensor.h"
#include "timeutils.h"
#include "bubblesort.h"
#include "debug.h"
#include "parse_utils.h"
#include "watchdog.h"
#include "zephyr/sys_clock.h"
#include "bubblesort.h"

/* UART device, for now debug is not allowed */
#define UART_DEVICE_NODE DT_ALIAS(iridium_port)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* How many times we try to detect a sensor */
#define DETECTION_TRIES   3
#define MAX_N_SENSORS     6
#define MAX_RESPONSE_SIZE 200 /* Maximum size for a response from the sensors */

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
static int needs_external_voltage(void);

/**
 * Smart sensor driver structure for the Seabird range sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_seabird = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .finish = NULL,
    .calibrate_zero = NULL, /* This sensors don't require calibration */
    .calibrate_full = NULL,
    .acquire = acquire,
    .pass_command = NULL,
    .name = name,
    .needs_external_voltage = needs_external_voltage,
};

struct seabird_sensors {
    float temperature;
    float conductivity;
    float pressure;
    float concentration;
    float pH;
    float chlorophyll;
    float turbidity;
    float salinity;
    float saturation;
    uint8_t temperature_status;
    uint8_t conductivity_staus;
    uint8_t pressure_status;
    uint8_t concentration_status;
    uint8_t pH_status;
    uint8_t chlorophyll_status;
    uint8_t turbidity_status;
    uint8_t salinity_status;
    uint8_t saturation_status;
};

static struct seabird_sensors seabird;

/*
 * Local prototypes
 */
static int seabird_sensor_gets_with_timeout(char *response, int size, uint32_t timeout);
static int seabird_sensor_gets(char *response, int size);
static int request_measurement(struct smart_sensor *sensor, struct measurement *measurement);
static void smart_sensor_send_command(unsigned char *data, size_t size);
static int process_data(char *response, struct measurement *measurement);
/*
 * Get the maximum number of sensors of this type this driver can handle
 */
static int max_sensors(void)
{
    return MAX_N_SENSORS;
}

/**
 * Get the name of the driver.
 */
static const char *name(void)
{
    return "Seabird";
}

/*
 * Send command to Seabird sensors
 */
static void smart_sensor_send_command(unsigned char *data, size_t size)
{
    for (int i = 0; i < size; i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout of 1 second. The newline is not included in the buffer.
 * @param s A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @return The number of bytes stored in the buffer
 */
static int seabird_sensor_gets(char *s, int size)
{
    return seabird_sensor_gets_with_timeout(s, size, 200000);
}

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @param timeout After this time, signal timeout (microseconds)
 * @return The number of bytes stored in the buffer
 */
static int seabird_sensor_gets_with_timeout(char *response, int size, uint32_t timeout)
{
    uint8_t n = 0;
    char c;
    k_timepoint_t end = sys_timepoint_calc((K_MSEC(timeout)));

    while (1) {
        watchdog_reset();
        if (uart_poll_in(uart_dev, &c) >= 0) {
            watchdog_reset();
            if (c == '\r') {
                break;
            }
            if (c == '\n') {
            }
            *response++ = c;
            n++;
        }
        if (sys_timepoint_expired(end)) {
            break;
        }
    }
    *response = '\0';
    return n;
}

/*
 * Prepare the driver
 */
static int prepare(struct smart_sensor *sensor)
{
    return 0;
}

/**
 * Check if there is a Seabird range sensor connected
 * @param sensor_number The number of the sensor to check for (Unused here)
 * @param measurement A pointer to return a measurement from the sensor
 * @param sensor A pointer to store the sensor information
 * @return True if a sensor was detected
 */
static int detect(int sensor_number, struct smart_sensor *sensor)
{
    char response[MAX_RESPONSE_SIZE];
    char *p = response;
    int tries;

    DEBUG("Checking Seabird sensors\n");
    for (tries = 0; tries < 10; tries++) {
        smart_sensor_send_command("\r", 1); /* wake seabird. */
        if (seabird_sensor_gets_with_timeout(response, sizeof(response), 2000) > 0) {
            DEBUG("Data Seabird: %s\n", response);
            while (isspace((int)*p)) {
                p++;
            }
            if ((p[0] == '<') && (p[1] == 'E')) {
                /* Sensor detected */
                sensor->number = sensor_number;
                switch (sensor->number) {
                    case 0:
                        sensor->type = CTDO_SENSOR;
                        break;
                    case 1:
                        sensor->type = PRESSURE_SENSOR;
                        break;
                    case 2:
                        sensor->type = OXYGEN_SENSOR;
                        break;
                    case 3:
                        sensor->type = CHLOROPHYLL_SENSOR;
                        break;
                    case 4:
                        sensor->type = TURBIDITY_SENSOR;
                        break;
                    case 5:
                        sensor->type = PH_SENSOR;
                        break;
                }
                sensor->manufacturer = SEABIRD;
                sensor->power_up_time = 1000; /* TODO Check from datasheet */
                sensor->channel = 0;          /* We can only have one sensor of this type */
                DEBUG("OK\n");
                return 1;
            }
        }
    }
    DEBUG("NO\n");
    return 0;
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    /*	turn_on_smart_sensor(0);   // TODO Move to main */
    if (!device_is_ready(uart_dev)) {
        printk("Error device seabird\n");
        return -4; /* Some error on OC4 */
    }
    return 0;
}

/*
 * Finish the operation with the smart sensors
 */
static int finish_driver(void)
{
    /*    turn_off_smart_sensor(0);  // TODO Move to main */
    return 0;
}

/**
 * Acquire one smart sensor attached to this device.
 * We read many times into an array, then we sort the array and take the value in the
 * middle, this way we hope to remove extreme values.
 * @param tries The number of retries if there are problems with the sensor
 * @param sensor The sensor to read
 * @param measurements A Pointer to store the result measurement
 * @return 1 if OK, 0 on error // TODO Better return the error code
 */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *measurement)
{
    const int n = 20;
    int i = 0;

    while ((tries > 0) && (i < n)) {
        DEBUG("Trying\n");
        if (request_measurement(sensor, measurement)) {
            return 1;
            break;
        } else {
            DEBUG("Error reading Seabird sensor\n");
            tries--;
        }
    }
    return 0;
}

static int request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    char response[MAX_RESPONSE_SIZE];
    uint8_t try = 10;
    uint8_t ret = 0;

    if (sensor->number == 0) {
        watchdog_disable();
        smart_sensor_send_command("\r", 1);
        sleep_microseconds(1000000);
        smart_sensor_send_command("\r", 1);
        printk("Send TPSS\n");
        smart_sensor_send_command("TPSS\n\r", 6);
        smart_sensor_send_command("\r", 1);
        sleep_microseconds(40000000);
        init_driver();
        while (try > 0) {
            try--;
            printk("Try\n");
            ret = seabird_sensor_gets(response, sizeof(response));
            printk("Ret: %i\n", ret);
            if (ret > 100) {
                printk("Size correcto\n");
                break;
            }
        }
        printk("Request data: %s\n", response);
        watchdog_init();
        process_data(response, measurement);
        struct ctdo_raw_measurement *m = &(measurement->ctdo);

        measurement->type = CTDO_SENSOR;
        measurement->sensor_status = SENSOR_OK;
        m->depth = 15;
        m->temperature = seabird.temperature;
        m->conductivity = seabird.conductivity;
        m->saturation = seabird.saturation;
        m->humidity = 15;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = seabird.temperature_status;
        m->conductivity_status = seabird.conductivity_staus;
        m->saturation_status = seabird.saturation_status;
        return 1;
    } else if (sensor->number == 1) {
        struct pressure_measurement *m = &(measurement->pressure);

        measurement->type = PRESSURE_SENSOR;
        measurement->sensor_status = SENSOR_OK;
        m->temperature = seabird.temperature;
        m->pressure = seabird.pressure * 10000; /* dbar to pascal. */
        m->humidity = 15;
        m->temperature_status = seabird.temperature_status;
        m->pressure_status = seabird.pressure_status;
        return 1;
    } else if (sensor->number == 2) {
        struct oxygen_measurement *m = &(measurement->oxygen);

        measurement->type = OXYGEN_SENSOR;
        measurement->sensor_status = SENSOR_OK;
        m->depth = 15;
        m->temperature = seabird.temperature;
        m->concentration = seabird.concentration;
        m->saturation = seabird.saturation;
        m->salinity = seabird.salinity;
        m->humidity = 80;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = seabird.temperature_status;
        m->concentration_status = seabird.concentration_status;
        m->saturation_status = seabird.saturation_status;
        m->salinity_status = seabird.salinity_status;
        return 1;
    } else if (sensor->number == 3) {
        struct chlorophyll_measurement *m = &(measurement->chlorophyll);

        measurement->type = CHLOROPHYLL_SENSOR;
        measurement->sensor_status = SENSOR_OK;
        m->depth = 0.0;
        m->temperature = seabird.temperature;
        m->chlorophyll = seabird.chlorophyll;
        m->humidity = 15;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = seabird.temperature_status;
        m->chlorophyll_status = seabird.chlorophyll_status;
        return 1;
    } else if (sensor->number == 4) {
        struct turbidity_measurement *m = &(measurement->turbidity);

        measurement->type = TURBIDITY_SENSOR;
        measurement->sensor_status = SENSOR_OK;
        m->depth = 0.0;
        m->temperature = seabird.temperature;
        m->turbidity = seabird.turbidity;
        m->humidity = 15;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = seabird.temperature_status;
        m->temperature_status = seabird.turbidity_status;
        return 1;
    } else if (sensor->number == 5) {
        struct pH_measurement *m = &(measurement->pH);

        measurement->type = PH_SENSOR;
        measurement->sensor_status = SENSOR_OK;
        m->depth = 0.0;
        m->temperature = seabird.temperature;
        m->pH = seabird.pH;
        m->redox = 0.0;
        m->humidity = 0.0;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = seabird.temperature_status;
        m->pH_status = seabird.pH_status;
        m->redox_status = MEASUREMENT_VALUE_FIXED;
        return 1;
    }
    return 1;
}

static int process_data(char *response, struct measurement *measurement)
{
    char *buffer = response;
    float fluorescence_sd;
    float turbidity_sd;
    float sound_velocity;
    float specific_cond;

    buffer += 16;

    printk("Buffer %s\n", buffer);
    if (!get_next_float(&buffer, &seabird.temperature)) {
        seabird.temperature = 0.0;
        seabird.temperature_status = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.temperature_status = MEASUREMENT_OK;
    }
    if (!get_next_float(&buffer, &seabird.conductivity)) {
        seabird.conductivity = 0.0;
        seabird.conductivity_staus = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.conductivity_staus = MEASUREMENT_OK;
    }
    if (!get_next_float(&buffer, &seabird.pressure)) {
        seabird.pressure = 0.0;
        seabird.pressure_status = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.pressure_status = MEASUREMENT_OK;
    }
    if (!get_next_float(&buffer, &seabird.concentration)) {
        seabird.concentration = 0.0;
        seabird.concentration_status = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.concentration_status = MEASUREMENT_OK;
    }
    if (!get_next_float(&buffer, &seabird.pH)) {
        seabird.pH = 0.0;
        seabird.pH_status = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.pH_status = MEASUREMENT_OK;
    }
    if (!get_next_float(&buffer, &seabird.chlorophyll)) {
        seabird.chlorophyll = 0.0;
        seabird.chlorophyll_status = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.chlorophyll_status = MEASUREMENT_OK;
    }
    if (!get_next_float(&buffer, &seabird.turbidity)) {
        seabird.turbidity = 0.0;
        seabird.turbidity_status = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.turbidity_status = MEASUREMENT_OK;
    }
    get_next_float(&buffer, &fluorescence_sd);
    get_next_float(&buffer, &turbidity_sd);
    if (!get_next_float(&buffer, &seabird.salinity)) {
        seabird.salinity = 0.0;
        seabird.salinity_status = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.salinity_status = MEASUREMENT_OK;
    }
    get_next_float(&buffer, &sound_velocity);
    get_next_float(&buffer, &specific_cond);
    if (!get_next_float(&buffer, &seabird.saturation)) {
        seabird.saturation = 0.0;
        seabird.saturation_status = MEASUREMENT_VALUE_FIXED;
    } else {
        seabird.saturation_status = MEASUREMENT_OK;
    }
    printk("Temperature %.3f\n", (double)seabird.temperature);
    printk("Conductivity %.3f\n", (double)seabird.conductivity);
    printk("Pressure %.3f\n", (double)seabird.pressure);
    printk("Concentration %.3f\n", (double)seabird.concentration);
    printk("pH %.3f\n", (double)seabird.pH);
    printk("Chlorophyll %.3f\n", (double)seabird.chlorophyll);
    printk("Turbidity %.3f\n", (double)seabird.turbidity);
    printk("Salinity %.3f\n", (double)seabird.salinity);
    printk("Saturation %.3f\n", (double)seabird.saturation);

    return 0;
}

static int needs_external_voltage(void)
{
    return 1;
}
