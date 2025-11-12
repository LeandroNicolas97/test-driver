/*
 * Communication with the Maxbotix sensors
 * These sensors transmit continuously.
 */

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "config.h"
#include "smart_sensor.h"
#include "smart_sensor_protocol.h"
#include "clock.h"
#include "serial.h"
#include "errorcodes.h"
#include "microio.h"
#include "crc16.h"
#include "hardware.h"
#include "timeutils.h"
#include "bubblesort.h"
#include "debug.h"

/* How many times we try to detect a sensor */
#define DETECTION_TRIES   3
#define MAX_N_SENSORS     1
#define MAX_RESPONSE_SIZE 30 /* Maximum size for a response from the sensors */

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

/**
 * Smart sensor driver structure for the Maxbotix range sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_maxbotix = {
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
};

/*
 * Local prototypes
 */
static int maxbotix_sensor_gets_with_timeout(char *response, int size, uint32_t timeout);
static int maxbotix_sensor_gets(char *response, int size);

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
    return "Maxbotix";
}

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout of 1 second. The newline is not included in the buffer.
 * @param s A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @return The number of bytes stored in the buffer
 */
static int maxbotix_sensor_gets(char *s, int size)
{
    return maxbotix_sensor_gets_with_timeout(s, size, 1000000);
}

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @param timeout After this time, signal timeout (microseconds)
 * @return The number of bytes stored in the buffer
 */
static int maxbotix_sensor_gets_with_timeout(char *response, int size, uint32_t timeout)
{
    int n = 0;
    int c;
    struct timeval start;
    struct timeval now;

    getuptime(&start);
    rs485_receive();
    while (1) {
        c = serial_getchar(UART_SMART_SENSOR);
        if (c < 0) {
            getuptime(&now);
            if (microseconds_elapsed(&start, &now) >= timeout) {
                break;
            }
        } else {
            if (c == '\r' || n >= (size - 2)) {
                break;
            }
            *response++ = c;
            getuptime(&start);
            n++;
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
 * Check if there is a Maxbotix range sensor connected
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

    DEBUG("Checking Data-stream sensors\n");
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        if (maxbotix_sensor_gets(response, sizeof(response)) > 0) {
            DEBUG("DATA_STREAM: %s\n", response);
            while (isspace((int)*p)) {
                p++;
            }
            if (p[0] == 'R' && strlen(p) < 8) {
                /* Sensor detected */
                sensor->type = OXYGEN_SENSOR; /* RANGE_SENSOR; */
                sensor->manufacturer = MAXBOTIX;
                sensor->power_up_time = 1000; /* TODO Check from datasheet */
                sensor->channel = 0;          /* We can only have one sensor of this type */
                DEBUG("OK\n");
                return 1;
            } else if (p[0] == 'Z' && strlen(p) < 17) {
                /* Sensor detected */
                sensor->type = CO2_SENSOR; /* RANGE_SENSOR; */
                sensor->manufacturer = MAXBOTIX;
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
    /* turn_on_smart_sensor(0);   // TODO Move to main */
    serial_set_baudrate(UART_SMART_SENSOR, B9600);
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

/*
 *  Read the Maxbotix sensor once.
 *  If we can read something from the sensor, return that value, otherwise -1
 */
static int read_sensor_once(void)
{
    char response[MAX_RESPONSE_SIZE];
    char *p = response;
    char raw_value[6];
    long range = -1;

    if (maxbotix_sensor_gets(response, sizeof(response)) > 0) {
        DEBUG("Maxbotix: %s\n", response);
        while (isspace((int)*p)) {
            p++;
        }
        if (p[0] == 'R' && strlen(p) < 8) {
            /* Looks fine, extract the range value */
            range = atol(&response[1]);
        } else if (p[0] == 'Z' && strlen(p) < 17) {
            /* Looks fine, extract the range value */
            strncpy(raw_value, &p[2], 5);
            DEBUG("datastream value: %s\n", raw_value);
            range = atol(raw_value);
        }
    }
    return (int)range;
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
    int d;    /* Single range measurement */
    int m[n]; /* We take n range measurements */

    while ((tries > 0) && (i < n)) {
        DEBUG(" Trying ");
        d = read_sensor_once();
        if (d > 0) { /* Measurement is OK */
            DEBUG("OK\n");
            m[i] = d;
            i++;
        } else {
            DEBUG("Error reading Maxbotix sensor\n");
            tries--;
        }
    }
    if ((tries > 0) || (i >= n)) { /* We have measurements */
        bubblesort_int(n, m);
        int range = m[n / 2];

        if (sensor->type == CO2_SENSOR) {
            measurement->type = CO2_SENSOR;
            measurement->co2.depth = (float)range;
            measurement->co2.depth_status = MEASUREMENT_OK;
            measurement->co2.temperature = 0;
            measurement->co2.temperature_status = MEASUREMENT_VALUE_FIXED;
            measurement->co2.co2 =
                (float)range / 1000.0; /* x10 as indicated in datasheet for ppm; /10000 for ppm to percentage*/
            measurement->co2.co2_status = MEASUREMENT_OK;
            measurement->co2.humidity = 0;
            measurement->sensor_status = SENSOR_OK;
            return 1;
        }
        measurement->type = OXYGEN_SENSOR;
        measurement->oxygen.depth = (float)range;
        measurement->oxygen.depth_status = MEASUREMENT_OK;
        measurement->oxygen.concentration = ((float)range) / 100;
        measurement->oxygen.concentration_status = MEASUREMENT_OK;
        measurement->oxygen.saturation = ((float)range);
        measurement->oxygen.saturation_status = MEASUREMENT_OK;
        measurement->oxygen.humidity = 0;
        measurement->oxygen.salinity = 0;
        measurement->oxygen.salinity_status = MEASUREMENT_VALUE_FIXED;
        measurement->sensor_status = SENSOR_OK;
        return 1;
    } else {
        return 0;
    }
}
