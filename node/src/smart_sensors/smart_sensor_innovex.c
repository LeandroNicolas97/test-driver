/*
 * Communication with the smart sensors
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "bsp-config.h"
#include "measurement.h"
#include "smart_sensor.h"
#include "smart_sensor_protocol.h"
#include "serial.h"
#include "errorcodes.h"
#include "microio.h"
#include "crc16.h"
#include "hardware.h"
#include "timeutils.h"
#include "parse_utils.h"
#include "salinity.h"
#include "debug.h"
#include "watchdog.h"

/* How many times we try to detect a sensor */
#define DETECTION_TRIES   2
#define MAX_SENSORS       8
#define MAX_REQUEST_SIZE  128 /* Max size of the request to the sensors */
#define MAX_RESPONSE_SIZE 128 /* Maximum size for a response from the sensors */

#define DEPRECATED 0

/**
 * Driver function prototypes
 */
static int max_sensors(void);                                      /* Maximum number of sensors for this driver */
static const char *name(void);                                     /* Name of the driver */
static int init_driver(void);                                      /* Initialize the driver */
static int finish_driver(void);                                    /* Finish the operations with the driver */
static int detect(int sensor_number, struct smart_sensor *sensor); /* Detect a sensor */
static int prepare(struct smart_sensor *sensor);                   /* Prepare the sensor */
/* static int finish(struct smart_sensor *sensor); //  Finish the communication with the sensor */
/* static int calibrate_zero(struct smart_sensor *sensor); // Calibrate the zero of the sensor */
static int calibrate_full(struct smart_sensor *sensor); /* Calibrate the full scale of the sensor */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m);
static int pass_command(struct smart_sensor *sensor, char *command);
static int smart_sensor_request_name(char *name);
static int needs_external_voltage(void);

/**
 * Smart sensor driver structure for the Innovex sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_innovex = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .finish = NULL,         /* finish, */
    .calibrate_zero = NULL, /* calibrate_zero, */
    .calibrate_full = calibrate_full,
    .acquire = acquire,
    .pass_command = pass_command,
    .name = name,
    .needs_external_voltage = needs_external_voltage,
};

/*
 * Local prototypes
 */
static int smart_sensor_gets_with_timeout(char *response, int size, uint32_t timeout);
static int smart_sensor_request_measurement(char *name, struct measurement *measurement);
static void measurement_unit(char *name, int type);

/*
 * local variables
 */
static uint8_t pressure_unit;
static uint8_t unit_obtained;
static uint8_t phreatic_unit;

/**
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
    return "Innovex";
}

/**
 * Flush the input buffer
 */
static void smart_sensor_flush_input_buffer(void)
{
    int i = 100; /* Max number of chars to flush, to avoid dead-loop */

    while ((i > 0) && (serial_getchar(UART_SMART_SENSOR) > 0)) {
        i--;
    }
}

/**
 * Send a string to the smart sensor
 */
static void smart_sensor_send_string(char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s) {
        serial_putchar(UART_SMART_SENSOR, *s);
        s++;
    }
}

/**
 * Send a char to the smart sensor
 */
static void smart_sensor_send_char(char c)
{
    serial_putchar(UART_SMART_SENSOR, c);
}

/*
 * Send a command string over the Sensors UART using the name of the sensor
 * Prepend an escape and name and append a carriage return
 * @param name The name of the sensor
 * @param s The command to send
 */
static void smart_sensor_send_command_with_name(char *name, char *s)
{
    char request[MAX_REQUEST_SIZE];

    rs485_transmit(UART_SMART_SENSOR);
    sleep_microseconds(2000); /* 2ms to stabilize */
    DEBUG("Sending: %s %s\n", name, s);
    smart_sensor_send_char('\x1b');
    smart_sensor_flush_input_buffer();
    usnprintf(request, sizeof(request), "%s %s\r", name, s);
    smart_sensor_send_string(request);
    serial_drain(UART_SMART_SENSOR);
    rs485_receive(UART_SMART_SENSOR);
}

/*
 * Pass a command string to the sensors
 * Prepend an escape and name and append a carriage return
 * @param name The name of the sensor
 * @param s The command to send
 */
static int pass_command(struct smart_sensor *sensor, char *command)
{
    rs485_transmit(UART_SMART_SENSOR);
    sleep_microseconds(2000); /* 2ms to stabilize */
    DEBUG("Passing: %s\n", command);
    smart_sensor_send_char('\x1b');
    smart_sensor_flush_input_buffer();
    smart_sensor_send_string(command);
    smart_sensor_send_string("\r");
    serial_drain(UART_SMART_SENSOR);
    sleep_microseconds(2000); /* XXX */
    rs485_receive(UART_SMART_SENSOR);
    return 0;
}

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @param timeout After this time, signal timeout (milliseconds)
 * @return The number of bytes stored in the buffer
 * TODO Move to smart_sensor_communication.c
 */
static int smart_sensor_gets_with_timeout(char *response, int size, uint32_t timeout)
{
    int n = 0;
    int c;
    int64_t start;

    start = get_uptime_ms();
    while (1) {
        watchdog_reset();
        c = serial_getchar(UART_SMART_SENSOR);
        if (c < 0) {
            if (ms_elapsed(&start) > timeout) {
                break;
            }
        } else {
            if (c == '\n' || n >= (size - 2)) {
                break;
            }
            *response++ = c;
            start = get_uptime_ms();
            n++;
        }
    }
    *response = '\0';
    return n;
}

/*
 * Get the maximum number of sensors of this type this driver can handle
 */
static int prepare(struct smart_sensor *sensor)
{
    /* empty, this function is used in modbus sensors, so is used in
     * concentrator.c when we start the sampling machine state; we need
     * this empty function to not get stuck there
     */
    return 0;
}

/**
 * Check for a single sensor with the specified number on the RS-485 bus
 * @param sensor_number The number of the sensor to check for.
 * @param measurement A pointer to return a measurement from the sensor
 * @param sensor A pointer to store the sensor information
 * @return True if a sensor was detected
 */
static int detect_new_protocol(int sensor_number, struct smart_sensor *sensor)
{
    struct measurement measurement;
    char name[SIZE_SMART_SENSOR_NAME];
    int tries;

    usnprintf(name, sizeof(name), "SENS%i", sensor_number);
    DEBUG("Checking sensor %s... ", name);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        /*        if(!smart_sensor_request_name(name)) { */
        /*            // Sensor not detected with name command */
        /*            continue; */
        /*        } */
        if (smart_sensor_request_measurement(name, &measurement)) {
            /* We got a correct measurement from that sensor. Mark it as Innovex sensor */
            sensor->type = measurement.type;
            sensor->manufacturer = INNOVEX;
            sensor->version = 1;
            switch (sensor->type) {
                case FLOW_SENSOR:
                    sensor->power_up_time =
                        1000; /* INNOVEX_FLOW_SENSORS_POWERUP_TIME;   TODO Determine this from the sensor */
                    break;
                case WAVE_SENSOR:
                    sensor->power_up_time =
                        1000; /* INNOVEX_WAVE_SENSORS_POWERUP_TIME;   TODO Determine this from the sensor */
                    break;
                case FLOW_WATER_SENSOR:
                    sensor->power_up_time =
                        1000; /* INNOVEX_WATER_FLOW_SENSORS_POWERUP_TIME;   TODO Determine this from the sensor */
                    break;
                case GPS_SENSOR:
                    sensor->power_up_time =
                        30000; /* INNOVEX_GPS_SENSORS_POWERUP_TIME;   TODO Determine this from the sensor */
                    break;
                default:
                    sensor->power_up_time =
                        1000; /* INNOVEX_SENSORS_POWERUP_TIME;   TODO Determine this from the sensor */
                    break;
            }
            sensor->channel = 0; /* Currently we have only one channel */
            sensor->number = sensor_number;
            strcpy(sensor->name, name);
            DEBUG("OK\n");
            return 1;
        } else {
            DEBUG("NO\n");
        }
    }
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
    int status;

    status = detect_new_protocol(sensor_number, sensor);
    return status;
}

/**
 * Pass a command direct to a sensor
 */
#if DEPRECATED
void smart_sensor_pass_command(struct smart_sensor *sensor, char *command)
{
    char sensor_response[60];
    uint8_t counter = 0;

    select_smart_sensor_channel(sensor->channel);
    serial_flush(UART_SMART_SENSOR); /* Clean the receive buffer */
    delay_ds(5);
    smart_sensor_send_command_with_name(sensor->name, command);
    while (smart_sensor_gets(sensor_response, sizeof(sensor_response)) != 0) {
        uprintf("%s\n", sensor_response);
        counter++;
        if (counter > 100) {
            break;
        }
    }
}
#endif

/**
 * Wait for a specific response
 * @param expected_response A text with the expected response
 * @param timeout in milliseconds
 * Return negative on error. 0 if OK
 */
static int wait_for_specific_response(char *expected_response, uint32_t timeout)
{
    char response[20]; /* Response from smart sensor */
    int response_size;
    int expected_size = strlen(expected_response);

    response_size = smart_sensor_gets_with_timeout(response, sizeof(response), timeout);
    if (response_size == 0) {
        DEBUG("Timeout waiting for response\n");
        return -E_TIMEDOUT;
    }
    if (response_size < expected_size) {
        DEBUG("Response shorter than expected %i:%s\n", response_size, response);
        return -E_INVALID;
    }
    /* The response is larger or equal to the expected response, check if we have a spurious 0 */
    char *p = &(response[0]);

    if (*p == '\0') {
        p++;
    }
    if (strncmp(expected_response, p, expected_size) != 0) {
        DEBUG("Not the expected response: %s\n", response);
        return -E_INVALID;
    }
    return 0;
}

/**
 * Send the calibration command to the specified sensor.
 * The sensor must be already ON
 * @param sensor A pointer to the sensor to calibrate
 * @return 0 if OK, negative on error
 */
static int calibrate_full(struct smart_sensor *sensor)
{
    int wait_status;

    smart_sensor_flush_input_buffer();
    smart_sensor_send_command_with_name(sensor->name, "caloxy");
    wait_status = wait_for_specific_response("OK", 5000);
    if (wait_status < 0) {
        return wait_status;
    }
    smart_sensor_flush_input_buffer();
    smart_sensor_send_command_with_name(sensor->name, "commit");
    wait_status = wait_for_specific_response("OK", 3000);
    if (wait_status < 0) {
        return wait_status;
    }
    return 0;
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    /*	turn_on_smart_sensor(0);   // TODO Move to main */
    /*	serial_set_baudrate(UART_SMART_SENSOR, 9600); */
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

/* TODO Move to some other place */
/*
 * Strip the right part of a string, remove any CR or NL
 */
void strip_right(char *s)
{
    int last;

    last = strlen(s) - 1;
    while (last > 0) {
        if (s[last] == '\n' || s[last] == '\r') {
            s[last] = '\0';
            last--;
        } else {
            break;
        }
    }
}

/**
 * Get the start of a frame.
 * The start of frame char is ':' , sometimes there is some
 * garbage at the beginning. Skip the garbage and get the position
 * of the start byte.
 * If the start of frame is not detected, returns the original position.
 * @param length The length of the frame
 * @param frame A pointer to the frame data
 * @return the position of the start of frame char.
 * i
 */
char *get_start_of_frame(int length, char *frame)
{
    int i = 0;
    char *start_of_frame = frame;

    while (i < length) {
        if (frame[i] == ':') {
            start_of_frame = &(frame[i]);
            break;
        }
        i++;
    }
    return start_of_frame;
}

/**
 * Send a request to the smart sensor using its name and asking for a measurement.
 * Process the received string and set the measurement values.
 * return true if the measurement was parsed correctly
 */
static int smart_sensor_request_measurement(char *name, struct measurement *measurement)
{
    char sensor_response[MAX_RESPONSE_SIZE];
    char *start_of_frame;
    int response_size;

    serial_flush(UART_SMART_SENSOR); /* Clean the receive buffer */
    smart_sensor_send_command_with_name(name, "data");
    /**
     * timeout for now 1200ms, oxygen sensor respond in 600ms aprox, so 2times
     * that is our timeout
     */
    response_size = smart_sensor_gets_with_timeout(sensor_response, sizeof(sensor_response), 3000);
    DEBUG("Response [%i] %s\n", response_size, sensor_response);
    start_of_frame = get_start_of_frame(response_size, sensor_response);
    /*    print_hex_buffer(response_size, sensor_response); */
    /*    uprintf("\n"); */
    if (response_size < 4) {
        DEBUG("Too few data from sensor\n"); /* TODO log */
        measurement->sensor_status = SENSOR_NOT_DETECTED;
        return 0;
    }
    strip_right(sensor_response);
    if (smart_sensor_check_frame(start_of_frame) < 0) {
        DEBUG("Bad CRC from smart sensor\n"); /* TODO log */
        measurement->sensor_status = SENSOR_COMMUNICATION_BAD_CRC;
        return 0;
    }

    /* The frame is OK, try to parse it. Skip the first byte of the frame */
    if (deserialize_measurement((start_of_frame + 1), measurement) > 0) {
        /* Measurement parsed correctly */
        measurement_unit(name, measurement->type);
        if (measurement->type == RAIN_SENSOR) { /*Reset rain sensor memory*/
            smart_sensor_send_command_with_name(name, "reset");
        }
        DEBUG("Measurement parsed OK\n");
        return 1;
    } else {
        /* Measurement parsed with errors, this is also a communication error */
        measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
        /* XXX        uprintf("Y%s %i %i\n", sensor_response, strlen(sensor_response), response_size); */
        DEBUG("Error parsing measurement\n");
    }
    return 0;
}

/**
 * Send a request to the smart sensor using its name and asking for the name
 * This should be very fast, as the sensor does not have to acquire any measurement
 * and can be used to accelerate the detection
 * return true if sensor answered correctly
 */
static int __attribute__((unused)) smart_sensor_request_name(char *name)
{
    char sensor_response[MAX_RESPONSE_SIZE];
    int response_size;

    serial_flush(UART_SMART_SENSOR); /* Clean the receive buffer */
    smart_sensor_send_command_with_name(name, "name");
    response_size = smart_sensor_gets_with_timeout(sensor_response, sizeof(sensor_response), 500);
    DEBUG("Response [%i] %s\n", response_size, sensor_response);
    if (response_size < strlen(name)) {
        DEBUG("Too few data from sensor\n");
        return 0;
    }
    return 1;
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
        DEBUG(" Trying ");
        if (smart_sensor_request_measurement(sensor->name, measurement)) {
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

static int needs_external_voltage(void)
{
    return 0;
}

/*
 * @brief: get measurements units to show in the display
 */
static void measurement_unit(char *name, int type)
{
    if ((type == PHREATIC_LEVEL_SENSOR || type == PRESSURE_SENSOR) && !unit_obtained) {
        char sensor_response[MAX_RESPONSE_SIZE] = {0};

        smart_sensor_send_command_with_name(name, "unit");
        int response_size = smart_sensor_gets_with_timeout(sensor_response, sizeof(sensor_response), 500);

        if (response_size < strlen(name)) {
            pressure_unit = KPA;
            phreatic_unit = CENTMETER;
        }

        char *buffer = strtok(sensor_response, " ");

        if (buffer != NULL) {
            size_t len = strlen(buffer);

            while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
                buffer[--len] = '\0';
            }
        }
        while (strcmp(buffer, "kPa") && strcmp(buffer, "bar")) {
            buffer = strtok(NULL, " ");
            if (buffer != NULL) {
                size_t len = strlen(buffer);

                while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
                    buffer[--len] = '\0';
                }
            }
            if (buffer == NULL) {
                break;
            }
        }

        /* Here we check the pressure unit */
        if (!strcmp(buffer, "kPa")) {
            pressure_unit = KPA;
        } else if (!strcmp(buffer, "bar")) {
            pressure_unit = BAR;
        } else {
            pressure_unit = KPA;
        }

        if (type == PHREATIC_LEVEL_SENSOR) {
            for (int i = 0; i < 3; i++) {
                buffer = strtok(NULL, " ");
            }
            /*Here we clean the str of special characters*/
            if (buffer != NULL) {
                size_t len = strlen(buffer);

                while (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
                    buffer[--len] = '\0';
                }
            }
            /*check phreatic level unit*/
            if (!strcmp(buffer, "m")) {
                phreatic_unit = METER;
            } else {
                phreatic_unit = CENTMETER;
            }
        }

        unit_obtained = 1;
    }
}

int pass_pressure_unit(void)
{
    return pressure_unit;
}

int pass_phreatic_unit(void)
{
    return phreatic_unit;
}

void restore_meas_unit_flag(void)
{
    unit_obtained = 0;
}
