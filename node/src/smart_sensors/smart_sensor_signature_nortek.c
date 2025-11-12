/***************************************************************************
 *   file                 : smart_sensor_signature_nortek.c                *
 *   begin                : May 25, 2022                                   *
 *   copyright            : (C) 2011 by Innovex Tecnologias Ltda.          *
 *   Author               : Edgar Osorio                                   *
 *   Email                : edgar.osorio@innovex.cl                        *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2011. Innovex.                                          *
 ***************************************************************************/

/*
 * Communication with the smart sensors ADCP AQUAPRO NORTEK
 */

#include <string.h>
#include <stdlib.h>
#include "bsp-config.h"
#include "measurement.h"
#include "smart_sensor.h"
#include "serial.h"
#include "errorcodes.h"
#include "microio.h"
#include "crc16.h"
#include "hardware.h"
#include "timeutils.h"
#include "modbus.h"
#include "debug.h"
#include "configuration.h"
#include "nortek_signature.h"
#include "adcp_compression.h"
#include "watchdog.h"
#include "zephyr/sys_clock.h"
#include "adcp.h"

/* How many times we try to detect a sensor */
#define DETECTION_TRIES   3
#define MAX_SENSORS       1
#define MAX_RESPONSE_SIZE 256 /* Maximum size for a response from the sensors */
/* #define MAX_BEAMS             4 */
/* #define MAX_CELLS             129 */
#define MAX_CELLS_BOYA    120
/* #define PD_CELL_SIZE          2.0 */
#define EAST_ENU          0
#define NORTH_ENU         1

#define DEPRECATED 0

/**
 * HEAD_SIZE_AQUADOPP_PROFILER_VELOCITY_DATA= sizeof(aquadop->cSync) +  sizeof(aquadop->cId) + sizeof(aquadop->hSize) +
 * sizeof(aquadop->clock) + sizeof(aquadop->hError) + sizeof(aquadop->hAnaln1) + sizeof(aquadop->hBattery) +
 * sizeof(aquadop->hSoundSpeed)
 * + sizeof(aquadop->hHeading) + sizeof(aquadop->hPitch) + sizeof(aquadop->hRoll) + sizeof(aquadop->hPressureMSB) +
 * sizeof(aquadop->cStatus) + sizeof(aquadop->hPressureLSW) + sizeof(aquadop->hTemperature)
 */

/* #define HEAD_SIZE_AQUADOPP_PROFILER_VELOCITY_DATA      30 */
/* #define AQUADOPP_PROFILER_VELOCITY_DATA_SYNC           165 */
/* #define AQUADOPP_PROFILER_VELOCITY_DATA_ID             33 */
#define BUFFER_RECEP_ADCP 2220
struct adcp_data adcp_processed_data;

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
 * Smart sensor driver structure for the Maxbotix range sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_signature_nortek = {
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

/*
 * Local prototypes
 */
int adcp_sensor_gets_with_timeout(char *response, int size, uint32_t timeout);
/* static int adcp_sensor_gets(char *response, int size); */

/*
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
    return "NORTEK";
}

/**
 * Mod a%b
 */
int mod(int a, int b)
{
    int ret = a % b;

    if (ret < 0) {
        ret += b;
    }
    return ret;
}

/**
 * convert hex 8 to hex 16 bit
 */

int convert_hex8_hex16(unsigned char msb, unsigned char lsb)
{
    unsigned int r_msb;
    unsigned int r_lsb;

    r_msb = 0;
    r_lsb = 0;
    r_msb = r_msb + msb;
    r_msb = (r_msb << 8);
    r_lsb = r_lsb + lsb;
    return (r_msb | r_lsb);
}

/**
 * Convert from BCD to char
 */
unsigned char BCDToChar(unsigned char cBCD)
{
    unsigned char c1;
    unsigned char c2;

    c1 = cBCD & 0xf0;
    c1 = c1 >> 4;
    c2 = cBCD & 0x0f;
    return ((10 * c1) + c2);
}

/* Calculate checksum */

/* short Checksum(short *phBuff,int n) */
/* { */
/*     int i; */
/*     short hChecksum = 0xB58C; */
/*     for (i = 0; i < n; i++) */
/*     hChecksum += phBuff[i]; */
/*     return hChecksum; */
/* } */

#if DEPRECATED
/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout of 1 second. The newline is not included in the buffer.
 * @param s A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @return The number of bytes stored in the buffer
 */
static int adcp_sensor_gets(char *s, int size)
{
    return adcp_sensor_gets_with_timeout(s, size, 1000);
}
#endif

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @param timeout After this time, signal timeout (microseconds)
 * @return The number of bytes stored in the buffer
 */
int adcp_sensor_gets_with_timeout(char *response, int size, uint32_t timeout)
{
    int n = 0;
    int c;
    /* int64_t start; */
    /* struct timeval now; */
    int enable;
    /* int8_t find_break=0; */
    /* int8_t find_a5=0; */
    /* int16_t inicio_data; */

    /* start = get_uptime_ms(); */
    rs485_receive(UART_SMART_SENSOR);
    enable = 0;
    k_timepoint_t end = sys_timepoint_calc(K_MSEC(timeout));

    while (1) {
        c = serial_getchar(UART_SMART_SENSOR);

        if (c < 0) {
            /* getuptime(&now); */
            /* if(ms_elapsed(&start) > timeout) break; */
            if (sys_timepoint_expired(end)) {
                break;
            }
        } else {
            if (n >= size) {
                break;
            }
            if (n == 0) {
                enable = 1;
            }
            if (enable == 1) {
                /* *response++ = c; */
                n++;
            }
            /* start = get_uptime_ms(); */
            DEBUG("%c ", c);
        }
        watchdog_reset();
    }
    /* *response = '\0'; */
    /* DEBUG("\n"); */
    return n;
}

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @param timeout After this time, signal timeout (microseconds)
 * @return The number of bytes stored in the buffer
 */
int adcp_sensor_gets_with_timeout2(char *response, int size, uint32_t timeout)
{
    int n = -1;
    int c;
    /* int64_t start; */
    /* struct timeval now; */
    int enable = 0;
    int8_t find_break = 0;
    int8_t find_a5 = 0;
    /* int16_t inicio_data; */

    /* start = get_uptime_ms(); */
    rs485_receive(UART_SMART_SENSOR);
    enable = 0;
    k_timepoint_t end = sys_timepoint_calc(K_MSEC(timeout));

    while (n < BUFFER_RECEP_ADCP) {
        c = serial_getchar(UART_SMART_SENSOR);
        if (sys_timepoint_expired(end)) {
            break;
        }
        if (n > BUFFER_RECEP_ADCP) {
            break;
        }
        if (c < 0) {
            /* getuptime(&now); */
            /* if(ms_elapsed(&start) > timeout) break; */
        } else {
            if (c == 0x00 && find_a5 == 0 && find_break == 0) {
                find_break = 1;
            }
            if (c == 0xA5 && find_break == 1 && find_a5 == 0) {
                find_a5 = 1;
            }
            if (find_break == 1 && find_a5 == 1) {
                DEBUG("Inicio de Trama en A5 : %i\n", n);
                find_break = 2;
                find_a5 = 2;
                n = 0;
            }

            if (n >= size) {
                printk("Break\n");
                break;
            }
            if (n == 0) {
                enable = 1;
            }
            if (enable == 1) {
                *response++ = c;
                n++;
            }
            /* start = get_uptime_ms(); */
            /* DEBUG("%c, ", c); */
            /* DEBUG("%i, ", n); */
        }
        watchdog_reset();
    }
    /* *response = '\0'; */
    /*  DEBUG("\n"); */
    return n;
}

#if DEPRECATED
/**
 * calculate current direction
 */
float direction(float vel_east, float vel_north)
{
    float degrees;

    degrees = atan2(-1 * (vel_east), -1 * (vel_north));
    return (180 + 180 / 3.1416 * degrees);
}

/**
 * calculate current speed
 */
float speed(float vel_east, float vel_north, float vel_up)
{
    float total;

    total = (vel_east * vel_east) + (vel_north * vel_north) + (vel_up * vel_up);
    /* sp=(float) sqrt(pow(vel_east,2)+pow(vel_north,2)+pow(vel_up,2)); */
    return sqrt(total);
}
#endif

/*
 * Prepare the driver
 */
static int prepare(struct smart_sensor *sensor)
{
    return 0;
}

/*send BREAK commnad to ADCP*/

void send_break(void)
{
    rs485_transmit(UART_SMART_SENSOR);
    serial_putchar(UART_SMART_SENSOR, '@');
    serial_putchar(UART_SMART_SENSOR, '@');
    serial_putchar(UART_SMART_SENSOR, '@');
    serial_putchar(UART_SMART_SENSOR, '@');
    serial_putchar(UART_SMART_SENSOR, '@');
    serial_putchar(UART_SMART_SENSOR, '@');
    serial_putchar(UART_SMART_SENSOR, '\r');

    watchdog_reset();
    sleep_microseconds(150000); /* delay 150ms */
    serial_putchar(UART_SMART_SENSOR, 'K');
    serial_putchar(UART_SMART_SENSOR, '1');
    serial_putchar(UART_SMART_SENSOR, 'W');
    serial_putchar(UART_SMART_SENSOR, '%');
    serial_putchar(UART_SMART_SENSOR, '!');
    serial_putchar(UART_SMART_SENSOR, 'Q');
    serial_putchar(UART_SMART_SENSOR, '\r');

    watchdog_reset();
    sleep_microseconds(400000); /* delay 400ms */
    serial_putchar(UART_SMART_SENSOR, 'K');
    serial_putchar(UART_SMART_SENSOR, '1');
    serial_putchar(UART_SMART_SENSOR, 'W');
    serial_putchar(UART_SMART_SENSOR, '%');
    serial_putchar(UART_SMART_SENSOR, '!');
    serial_putchar(UART_SMART_SENSOR, 'Q');
    serial_putchar(UART_SMART_SENSOR, '\r');
}

int8_t go_command_mode(int32_t timeout)
{
    int count_receive = 0;
    uint8_t response[100];

    k_timepoint_t end = sys_timepoint_calc(K_MSEC(timeout));

    while (count_receive != 85 && count_receive != 86) {
        if (sys_timepoint_expired(end)) {
            break;
        }

        send_break();
        DEBUG("\nPreparando ADCP.Esperando Respuesta break");
        count_receive = adcp_sensor_gets_with_timeout(response, 100, 3000);
        DEBUG("\nDATA RECEIVED break: %i", count_receive);

        if (count_receive < 85) {
            rs485_transmit(UART_SMART_SENSOR);
            watchdog_reset();
            sleep_microseconds(400000); /* delay 400ms */
            serial_putchar(UART_SMART_SENSOR, 'M');
            serial_putchar(UART_SMART_SENSOR, 'C');
            serial_putchar(UART_SMART_SENSOR, 0X0D);

            watchdog_reset();
            sleep_microseconds(400000); /* delay 400ms */

            DEBUG("\nPreparando ADCP.Esperando Respuesta MC");
            count_receive = adcp_sensor_gets_with_timeout(response, 100, 3000);
            DEBUG("\nDATA RECEIVED MC: %i", count_receive);
            watchdog_reset();
            sleep_microseconds(400000); /* delay 400ms */
        }
        watchdog_reset();
    }

    if (count_receive == 85 || count_receive == 86) {
        return 1;
    } else {

        return 0;
    }
}

int8_t go_powerdown(int32_t timeout)
{
    int count_receive = 0;
    uint8_t response[100];
    /* int64_t start;*/

    /* start = get_uptime_ms(); */
    k_timepoint_t end = sys_timepoint_calc(K_MSEC(timeout));

    while (count_receive != 3) {
        /* if(ms_elapsed(&start) > timeout) break; */
        if (sys_timepoint_expired(end)) {
            break;
        }

        rs485_transmit(UART_SMART_SENSOR);
        watchdog_reset();
        sleep_microseconds(400000); /* delay 400ms */
        serial_putchar(UART_SMART_SENSOR, 'P');
        serial_putchar(UART_SMART_SENSOR, 'O');
        serial_putchar(UART_SMART_SENSOR, 'W');
        serial_putchar(UART_SMART_SENSOR, 'E');
        serial_putchar(UART_SMART_SENSOR, 'R');
        serial_putchar(UART_SMART_SENSOR, 'D');
        serial_putchar(UART_SMART_SENSOR, 'O');
        serial_putchar(UART_SMART_SENSOR, 'W');
        serial_putchar(UART_SMART_SENSOR, 'N');

        serial_putchar(UART_SMART_SENSOR, 0X0D);
        DEBUG("\nPreparando ADCP.Esperando Respuesta powerdown");
        count_receive = adcp_sensor_gets_with_timeout(response, 100, 3000);
    }

    if (count_receive == 3) {
        DEBUG("\n Recibida Confirmacion OK ADCP Dormido\n");
        return 1;
    } else {

        return 0;
    }
}

void check_mode_adcp(int32_t timeout)
{
    int count_receive = 0;
    uint8_t response[100];
    int64_t start;

    start = get_uptime_ms();
    rs485_transmit(UART_SMART_SENSOR);
    watchdog_reset();
    sleep_microseconds(400000); /* delay 400ms */
    serial_putchar(UART_SMART_SENSOR, 'I');
    serial_putchar(UART_SMART_SENSOR, 'N');
    serial_putchar(UART_SMART_SENSOR, 'Q');
    serial_putchar(UART_SMART_SENSOR, 0X0D);

    DEBUG("\nPreparando ADCP.Esperando Respuesta INQ");
    count_receive = adcp_sensor_gets_with_timeout(response, 100, 6000);

    DEBUG("\nResponse size INQ: %d", count_receive);
}

int8_t request_current_profiler(struct adcp_raw_data *adcp, int sensor_number)
{
    int count_receive;

    /* int i; */
    /* int j; */
    /* int bytes_to_receive=0; */
    uint8_t *p_response;
    /* struct PdVecVel signature_250; */
    /* int indice_vel; */
    int status;
    uint8_t response_adcp[BUFFER_RECEP_ADCP];

    watchdog_reset();
    sleep_microseconds(400000); /* delay 400ms */

    rs485_transmit(UART_SMART_SENSOR);
    DEBUG("\nSend START");
    serial_putchar(UART_SMART_SENSOR, 'S');
    serial_putchar(UART_SMART_SENSOR, 'T');
    serial_putchar(UART_SMART_SENSOR, 'A');
    serial_putchar(UART_SMART_SENSOR, 'R');
    serial_putchar(UART_SMART_SENSOR, 'T');
    serial_putchar(UART_SMART_SENSOR, 0X0D);

    DEBUG("\nPreparando ADCP.Esperando Respuesta START\n");
    count_receive = adcp_sensor_gets_with_timeout2(response_adcp, BUFFER_RECEP_ADCP, 20000);

    p_response = response_adcp;
    DEBUG("\nDATA RECEIVED: %i", count_receive);

    if (count_receive > 0) {
        DEBUG("\nDATA_STREAM: ");

        parse_nortek_adcp_data_frame(p_response, adcp);

        /* DEBUG("Size of adcp_raw_data: %lu\n", sizeof(adcp)); */
        DEBUG("Heading:     %i  %.2f\n", adcp->heading, (double)adcp->heading / 100);
        DEBUG("Pitch:       %i  %.2f\n", adcp->pitch, (double)adcp->pitch / 100);
        DEBUG("Roll:        %i  %.2f\n", adcp->roll, (double)adcp->roll / 100);
        DEBUG("Temperature: %i  %.2fC\n", adcp->temperature, (double)adcp->temperature / 100);
        DEBUG("Pressure:    %i  %.2fdBar\n", adcp->pressure, (double)adcp->pressure / 1000);
        DEBUG("Battery:     %i  %.2fV\n", adcp->battery_voltage, (double)adcp->battery_voltage / 10);
        DEBUG("Cells:       %i\n", adcp->cells);
        DEBUG("Beams:       %i\n", adcp->beams);
        DEBUG("Coords:      %i\n", adcp->coordinates);
        DEBUG("Blanking:    %i  %.2f\n", adcp->blanking, (double)adcp->blanking / 100);

    } else {
        return 0;
    }

    /* status = adcp_sensor_gets_with_timeout(response_adcp,4000,5000); */
    /* DEBUG("\nDatos descartados despues start: %i", status); */
    serial_flush(UART_SMART_SENSOR);
    watchdog_disable();
    sleep_microseconds(2000000); /* delay 2 seg */
    watchdog_init();
    serial_flush(UART_SMART_SENSOR);
    go_command_mode(10000);
    go_powerdown(10000);
    status = adcp_sensor_gets_with_timeout(response_adcp, 100, 5000);
    DEBUG("\nDatos descartados despues powerdown: %i", status);
    return 1;
}

/**
 * Check if there is a Maxbotix range sensor connected
 * @param sensor_number The number of the sensor to check for (Unused here)
 * @param measurement A pointer to return a measurement from the sensor
 * @param sensor A pointer to store the sensor information
 * @return True if a sensor was detected
 */
int detect(int sensor_number, struct smart_sensor *sensor)
{
    int count_receive;
    /* uint8_t response[MAX_RESPONSE_SIZE]; */
    /* unsigned char *p = response; */
    struct adcp_raw_data aquadop;
    /* int i; */
    /* int j; */
    /* int indice_vel; */
    /* int indice_anl; */
    /* int ubc; */

    int tries;
    int count_sensor;

    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        count_receive = 0;
        sensor->number = sensor_number;
        count_sensor = sensor_number;
        if (go_command_mode(7000) == 1) {
            if (request_current_profiler(&aquadop, sensor->number) == 1) {
                sensor->type = CURRENT_PROFILER_SENSOR; /* RANGE_SENSOR; */
                sensor->manufacturer = NORTEK;
                sensor->power_up_time = 4000; /* TODO Check from datasheet */
                sensor->channel = 0;          /* We can only have one sensor of this type */
                strcpy(sensor->name, "Sig250");
                DEBUG("OK\n");
                return 1;
            } else {
                DEBUG("NO\n");
                return 0;
            }
        }
    }
    return 0; /* should never reach here happy compiler */
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    /*	turn_on_smart_sensor(0);   // TODO Move to main */
    /* serial_set_baudrate(UART_SMART_SENSOR, B9600); */
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

/* print ascii caracter*/

void print_ascii(char *s)
{
    int n = 0;
    char *p;

    p = s;

    do {
        DEBUG("Caracter %d: %x\n", n, *p);
        p++;
        n++;
    } while (*p != '\0');
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
int acquire(int tries, struct smart_sensor *sensor, struct measurement *measurement)
{
    struct adcp_raw_data aquadop;

    while (tries > 0) {
        if (go_command_mode(7000) == 1) {
            if (request_current_profiler(&aquadop, sensor->number)) {
                aquadop.cells = 120; /* we do not need to 129 */
                process_adcp_raw_data(&aquadop, &adcp_processed_data);
                /* TODO we dont need all below, but acquire needs a *measurement */
                /*      find a way todo do this in a better way */
                measurement->type = CURRENT_PROFILER_SENSOR;
                measurement->current_profiler_signature.Heading = (float)aquadop.heading * 0.01f;
                /* DEBUG("Measurement Heading: */
                /* %.1f\n",measurement->current_profiler_signature.Heading); */
                measurement->current_profiler_signature.Pitch = (float)aquadop.pitch * 0.01f;
                /* DEBUG("Measurement Pitch: %.1f\n",measurement->current_profiler_signature.Pitch); */
                measurement->current_profiler_signature.Roll = (float)aquadop.roll * 0.01f;
                /* DEBUG("Measurement Roll: %.1f , */
                /* %i\n",measurement->current_profiler_signature.Roll,aquadop.hRoll); */
                measurement->current_profiler_signature.Temperature = (float)aquadop.temperature * 0.01f;
                /* DEBUG("Measurement Temperature: */
                /* %.1f\n",measurement->current_profiler_signature.Temperature); */
                measurement->current_profiler_signature.current_profiler_signature_status = MEASUREMENT_OK;
                measurement->sensor_status = SENSOR_OK;
                measurement->type = CURRENT_PROFILER_SENSOR;

                measurement->current_profiler_signature.speed = adcp_processed_data.vel[5];
                measurement->current_profiler_signature.direction = adcp_processed_data.dir[5];
                return 1;
            } else {
                DEBUG("Error reading Current Profiler sensor\n");
                tries--;
            }
        }
        watchdog_reset();
    }
    measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
    measurement->current_profiler_signature.current_profiler_signature_status = MEASUREMENT_ACQUISITION_FAILURE;
    return 0;
}

static int needs_external_voltage(void)
{
    return 0;
}
