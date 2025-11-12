/***************************************************************************
 *   file                 : smart_sensor_GPS.c                *
 *   begin                : May 25, 2022                                   *
 *   copyright            : (C) 2011 by Innovex Tecnologias Ltda.          *
 *   Author               : Edgar Osorio                                   *
 *   Email                : edgar.osorio@innovex.cl                        *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2011. Innovex.                                          *
 ***************************************************************************/

/*
 * Communication with the smart sensors GPS PA1010D
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
#include "watchdog.h"
#include "zephyr/sys_clock.h"
/* #include "nortek_signature.h" */
/* #include "adcp_compression.h" */

/**
 * Different commands to set the update rate from once a second (1 Hz) to 10 times
 * a second (10Hz) Note that these only control the rate at which the position is
 * echoed, to actually speed up the position fix you must also send one of the
 * position fix rate commands below too.
 */
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ "$PMTK220,10000*2F\r\n" /*/< Once every 10 seconds, 100 millihertz. */
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ "$PMTK220,5000*1B\r\n"  /*/< Once every 5 seconds, 200 millihertz. */
#define PMTK_SET_NMEA_UPDATE_1HZ            "$PMTK220,1000*1F\r\n"  /*/<  1 Hz */
#define PMTK_SET_NMEA_UPDATE_2HZ            "$PMTK220,500*2B\r\n"   /*/<  2 Hz */
#define PMTK_SET_NMEA_UPDATE_5HZ            "$PMTK220,200*2C\r\n"   /*/<  5 Hz */
#define PMTK_SET_NMEA_UPDATE_10HZ           "$PMTK220,100*2F\r\n"   /*/< 10 Hz */
/* Position fix update rate commands. */
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ                                                                            \
    "$PMTK300,10000,0,0,0,0*2C\r\n" /*/< Once every 10 seconds, 100 millihertz. */
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ                                                                            \
    "$PMTK300,5000,0,0,0,0*18\r\n"                              /*/< Once every 5 seconds, 200 millihertz              \
                                                                 */
#define PMTK_API_SET_FIX_CTL_1HZ "$PMTK300,1000,0,0,0,0*1C\r\n" /*/< 1 Hz */
#define PMTK_API_SET_FIX_CTL_5HZ "$PMTK300,200,0,0,0,0*2F\r\n"  /* /< 5 Hz */
/* Can't fix position faster than 5 times a second! */

#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F\r\n" /*/< 115200 bps */
#define PMTK_SET_BAUD_57600  "$PMTK251,57600*2C\r\n"  /*/<  57600 bps */
#define PMTK_SET_BAUD_9600   "$PMTK251,9600*17\r\n"   /*/<   9600 bps */

#define PMTK_SET_NMEA_OUTPUT_GLLONLY "$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n" /*/< turn on only the */
                                                                                             /*/< GPGLL sentence */
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n" /*/< turn on only the */
                                                                                             /*/< GPRMC sentence */
#define PMTK_SET_NMEA_OUTPUT_VTGONLY "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n" /*/< turn on only the */
                                                                                             /*/< GPVTG */
#define PMTK_SET_NMEA_OUTPUT_GGAONLY "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n" /*/< turn on just the */
                                                                                             /*/< GPGGA */
#define PMTK_SET_NMEA_OUTPUT_GSAONLY "$PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n" /*/< turn on just the */
                                                                                             /*/< GPGSA */
#define PMTK_SET_NMEA_OUTPUT_GSVONLY "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n" /*/< turn on just the */
                                                                                             /*/< GPGSV */
#define PMTK_SET_NMEA_OUTPUT_RMCGGA  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" /*/< turn on GPRMC and */
                                                                                             /*/< GPGGA */
#define PMTK_SET_NMEA_OUTPUT_RMCGGAGSA                                                                                 \
    "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n" /*/< turn on GPRMC, GPGGA */
                                                            /*/< and GPGSA */
#define PMTK_SET_NMEA_OUTPUT_ALLDATA                                                                                   \
    "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"                              /*/< turn on ALL THE DATA */
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" /*/< turn off output */

/* to generate your own sentences, check out the MTK command datasheet and use a */
/* checksum calculator such as the awesome */
/* http://www.hhhh.org/wiml/proj/nmeaxor.html */

#define PMTK_LOCUS_STARTLOG     "$PMTK185,0*22\r\n" /*/< Start logging data */
#define PMTK_LOCUS_STOPLOG      "$PMTK185,1*23\r\n" /*/< Stop logging data */
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C" /*/< Acknowledge the start or stop command */
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38\r\n"   /*/< Query the logging status */
#define PMTK_LOCUS_ERASE_FLASH  "$PMTK184,1*22\r\n" /*/< Erase the log flash data */
#define LOCUS_OVERLAP           0                   /*/< If flash is full, log will overwrite old data with new logs */
#define LOCUS_FULLSTOP          1                   /*/< If flash is full, logging will stop */

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E\r\n" /*/< Enable search for SBAS satellite (only works with 1Hz */
                                             /*/< output rate) */
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E\r\n" /*/< Use WAAS for DGPS correction data */

#define PMTK_STANDBY         "$PMTK161,0*28\r\n"     /*/< standby command & boot successful message */
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36\r\n" /*/< Not needed currently */
#define PMTK_AWAKE           "$PMTK010,002*2D\r\n"   /*/< Wake up */

#define PMTK_Q_RELEASE "$PMTK605*31\r\n" /*/< ask for the release and version */

#define PGCMD_ANTENNA   "$PGCMD,33,1*6C"     /*/< request for updates on antenna status */
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D\r\n" /*/< don't show antenna status messages */

/* How many times we try to detect a sensor */
#define DETECTION_TRIES   6
#define MAX_SENSORS       1
#define MAX_RESPONSE_SIZE 100 /* Maximum size for a response from the sensors */
#define BUFFER_RECEP_ADCP 100

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

char response_adcp[100];
/**
 * Smart sensor driver structure for the Maxbotix range sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_gps_pa1010d = {
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
/*Function to find minimum of x and y*/

struct gps {
    float latitude;
    float longitude;
};

/*
 * Local prototypes
 */
int read_gps_output(char *response, int size, uint32_t timeout);
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
    return "GPS_PA1010D";
}

/*
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @param timeout After this time, signal timeout (microseconds)
 * @return The number of bytes stored in the buffer
 */
int read_gps_output(char *response, int size, uint32_t timeout)
{
    int n = 0;
    int c;
    /* char *p; */
    /* p = response; */
    rs485_receive(UART_SMART_SENSOR);
    k_timepoint_t end = sys_timepoint_calc(K_MSEC(timeout));

    while (1) {
        c = serial_getchar(UART_SMART_SENSOR);
        if (c <= 0) {
            /* getuptime(&now); */
            /* if(ms_elapsed(&start) > timeout) break; */
            if (sys_timepoint_expired(end)) {
                break;
            }
        } else {
            if (n >= size) {
                break;
            }

            /* start = get_uptime_ms(); */
            if (c != 0x0A && c != 0x0D) {

                response_adcp[n] = c;
                /* p++; */
                n++;
                DEBUG("%c", c);
            }

            /* DEBUG("%c ", c); */
        }
        watchdog_reset();
    }
    /* *p = '\0'; */
    /* DEBUG("\n"); */
    return n;
}

/***
 *
 *
 * Delete character from buffer
 */

void delete_string(char *src, char *dest)
{

    char *p;

    p = src;
    while (*p != '\0') {
        if (*p != 0x0A && *p != 0x0D) {
            *dest = *p;
            ++dest;
        }
        ++p;
    }
    *dest = '\0';
}
/**
 * to Write command to gps
 */
int send_command_gps(char *str_command)
{
    int ret = 1;

    while ((*str_command) != '\0') {
        serial_putchar(UART_SMART_SENSOR, *str_command);
        str_command++;
    }

    return ret;
}

/*
 * Prepare the driver
 */
static int prepare(struct smart_sensor *sensor)
{

    return 0;
}

int8_t request_gps_output(struct gps *pa1010d, int sensor_number)
{
    int count_receive;

    char *p_response;
    /* char *p_response2; */

    rs485_transmit(UART_SMART_SENSOR);
    send_command_gps(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    send_command_gps(PMTK_SET_NMEA_UPDATE_1HZ);
    watchdog_disable();
    sleep_microseconds(1000000); /* delay 1 seg */
    watchdog_init();

    DEBUG("\nPrepare. Send Comand GPS");
    DEBUG("\nEsperando Respuesta GPS");
    count_receive = read_gps_output(response_adcp, 90, 1000);
    /* sleep_microseconds(1000000);  // delay 10 seg */
    serial_flush(UART_SMART_SENSOR);

    watchdog_reset();
    p_response = strchr(response_adcp, '$');
    /* p_response2  = strchr(p_response +1 , '$'); */
    /* if (*p_response2 != '\0') */
    /* { */
    /*    p_response = p_response2; */
    /* } */

    DEBUG("\nDATA RECEIVED: %i", count_receive);

    if (count_receive > 0) {
        DEBUG("\nDATA_STREAM: ");
        /* char *p = response_adcp; */
        DEBUG("\n Response GPS: %s", response_adcp);
        DEBUG("\n Response GPS Buffer filtrado: %s", p_response);

        /* strlcpy(buffer,response_adcp,6); */

        DEBUG("\nData Valida: %c", p_response[18]);

        if (!strncmp("$GNRMC", p_response, 6)) {
            DEBUG("\nEncontrada Trama RMC");
            if (p_response[18] == 'A') {
                DEBUG("\n \r ++++++++++++ Data Valida +++++++++++++  \r\n");

                char *p = p_response;
                char latitude_indicator;
                char longitud_indicator;
                long dddmm;
                long ddmm;
                float degrees;
                float minutes;
                float decminutes;
                long fixed;
                float deg;
                float ang;

                p = strchr(p, ',') + 1;
                p = strchr(p, ',') + 1;
                p = strchr(p, ',') + 1;

                char buff[10] = {0}; /* Ensure string is terminated after strncpy */
                char *e = strchr(p, '.');

                strncpy(buff, p, e - p); /* get DDDMM */
                /* DEBUG("Degreebuff Latitude String: %s",buff); */

                ddmm = atol(buff);
                degrees = (float)(ddmm / 100);  /* truncate the minutes */
                minutes = ddmm - degrees * 100; /* remove the degrees */

                p = e + 1;
                e = strchr(p, ',');
                strncpy(buff, p, e - p); /* get Deciminutes */

                /* DEBUG("\n \r Latitude Decminutes String: %s",buff); */
                decminutes = atof(buff) / 10000;
                /* DEBUG("\n \r Latitude Decminutes float: %f",decminutes); */

                DEBUG("Latitude degress: %f, minutes:%f, decmines: %f",
                      (double)degrees,
                      (double)minutes,
                      (double)decminutes);
                deg = degrees + (minutes / 60) + (decminutes / 60);
                /* deg = fixed / (float)10000000.; */
                ang = degrees + minutes + decminutes;

                latitude_indicator = e[1];
                DEBUG("\n \r Latitude Indicator: %c", latitude_indicator);

                if (latitude_indicator == 'S' ||
                    latitude_indicator == 'W') { /* fixed and deg are signed, but DDDMM.mmmm is not */
                    fixed = -fixed;
                    deg = -deg;
                }

                DEBUG("\n\rDegrees Latitude: %f", (double)deg);
                DEBUG("\n\rAngle Latitude: %f", (double)ang);
                pa1010d->latitude = deg;
                p = strchr(p, ',') + 1;
                p = strchr(p, ',') + 1;

                e = strchr(p, '.');

                strncpy(buff, p, e - p); /* get Deggrees longitud */
                /* DEBUG("\n\r Degreebuff Longitud: %s",buff); */
                dddmm = atol(buff);
                degrees = (float)(dddmm / 100);  /* truncate the minutes */
                minutes = dddmm - degrees * 100; /* remove the degrees */

                p = e;
                e = strchr(p, ',');
                strncpy(buff, p, e - p); /* get Deciminutes */

                /* DEBUG("\n \r Longitud Decminutes: %s",buff); */
                decminutes = atof(buff);
                /* fixed = degrees * 10000000 + (minutes * 10000000) / 60 + (decminutes * 10000000) / 60; */
                DEBUG("Longitud degress: %f, minutes:%f, decmines: %f",
                      (double)degrees,
                      (double)minutes,
                      (double)decminutes);
                deg = degrees + (minutes / 60) + (decminutes / 60);
                ang = degrees + minutes + decminutes;

                longitud_indicator = e[1];
                DEBUG("\n \r Longitud Indicator: %c", longitud_indicator);

                if (longitud_indicator == 'S' ||
                    longitud_indicator == 'W') { /* fixed and deg are signed, but DDDMM.mmmm is not */
                    fixed = -fixed;
                    deg = -deg;
                }

                DEBUG("\n\rDegrees Longitud: %f", (double)deg);
                DEBUG("\n\rAngle Longitud: %f", (double)ang);
                pa1010d->longitude = deg;

            } else {
                DEBUG("\n\r+++++++Data No Valida.GPS .No encuentra Satelite +++++++++++");

                return 0;
            }

        } else {

            serial_flush(UART_SMART_SENSOR);
            return 0;
        }

    } else {

        serial_flush(UART_SMART_SENSOR);
        return 0;
    }

    serial_flush(UART_SMART_SENSOR);

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
    struct gps pa1010d;
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
        /* if (go_command_mode(7000)==1) */
        /* { */

        if (request_gps_output(&pa1010d, sensor->number) == 1) {
            sensor->type = GPS_SENSOR; /* RANGE_SENSOR; */
            sensor->manufacturer = GPS;
            sensor->power_up_time = 4000; /* TODO Check from datasheet */
            sensor->channel = 0;          /* We can only have one sensor of this type */
            strcpy(sensor->name, "GPS");
            DEBUG("OK\n");
            DEBUG("Apaga Salida GPS\n");
            send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF);
            watchdog_disable();
            sleep_microseconds(1000000); /* delay 2 seg */
            send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF);
            sleep_microseconds(1000000); /* delay 2 seg */
            watchdog_init();
            return 1;
        } else {
            if (tries == 5) {
                DEBUG("NO\n");
                rs485_transmit(UART_SMART_SENSOR);
                send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF);
                send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF);
                return 0;
            }
        }
        /* } */
    }
    return 0; /* should never reach here happy compiler */
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    /*	turn_on_smart_sensor(0);   TODO Move to main */
    /* serial_set_baudrate(UART_SMART_SENSOR, B9600); */
    rs485_transmit(UART_SMART_SENSOR);
    DEBUG("\nInit. Send Comand GPS");
    watchdog_disable();
    sleep_microseconds(20000000); /* delay 10 seg */
    watchdog_init();
    send_command_gps(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    send_command_gps(PMTK_SET_NMEA_UPDATE_1HZ);
    /* send_command_gps(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); */
    /* send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF); */
    serial_flush(UART_SMART_SENSOR);
    /* serial_flush(UART_SMART_SENSOR); */

    return 0;
}

/*
 * Finish the operation with the smart sensors
 */
static int finish_driver(void)
{
    /*    turn_off_smart_sensor(0);  TODO Move to main */
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
int acquire(int tries, struct smart_sensor *sensor, struct measurement *measurement)
{
    struct gps pa1010d;

    /* sleep_microseconds(10000000);  // delay 10 seg */
    /* sleep_microseconds(10000000);  // delay 10 seg */
    /* sleep_microseconds(10000000);  // delay 10 seg */
    tries = 6;
    while (tries > 0) {
        /* if (go_command_mode(7000) == 1) { */
        if (request_gps_output(&pa1010d, sensor->number)) {
            /* aquadop.cells = 120; //we do not need to 129 */
            /* process_adcp_raw_data(&aquadop, &adcp_processed_data); */
            /* TODO we dont need all below, but acquire needs a *measurement */
            /*      find a way todo do this in a better way */
            measurement->type = GPS_SENSOR;
            measurement->gps.Latitude = pa1010d.latitude;
            measurement->gps.Longitude = pa1010d.longitude;

            /* DEBUG("Measurement Heading: */
            /* %.1f\n",measurement->current_profiler_signature.Heading); */
            /* measurement->current_profiler_signature.Pitch = (float)aquadop.pitch * 0.01; */
            /* DEBUG("Measurement Pitch: %.1f\n",measurement->current_profiler_signature.Pitch); */
            /* measurement->current_profiler_signature.Roll = (float)aquadop.roll * 0.01; */
            /* DEBUG("Measurement Roll: %.1f , */
            /* %i\n",measurement->current_profiler_signature.Roll,aquadop.hRoll); */
            /* measurement->current_profiler_signature.Temperature = */
            /*   (float)aquadop.temperature * 0.01; */
            /* DEBUG("Measurement Temperature: */
            /* %.1f\n",measurement->current_profiler_signature.Temperature); */
            /* measurement->current_profiler_signature.current_profiler_signature_status = */
            /*     MEASUREMENT_OK; */
            measurement->sensor_status = SENSOR_OK;
            measurement->type = GPS_SENSOR;

            /* measurement->current_profiler_signature.speed = adcp_processed_data.vel[5]; */
            /* measurement->current_profiler_signature.direction = adcp_processed_data.dir[5]; */
            watchdog_reset();
            rs485_transmit(UART_SMART_SENSOR);
            DEBUG("Apaga Salida GPS\n");
            send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF);
            watchdog_disable();
            sleep_microseconds(2000000); /* delay 2 seg */
            send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF);
            sleep_microseconds(1000000); /* delay 2 seg */
            watchdog_init();
            return 1;
        } else {
            DEBUG("Error reading GPS sensor\n");

            tries--;
        }
        /* } */
    }
    rs485_transmit(UART_SMART_SENSOR);
    DEBUG("Apaga Salida GPS\n");
    send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF);
    watchdog_disable();
    sleep_microseconds(2000000); /* delay 2 seg */
    send_command_gps(PMTK_SET_NMEA_OUTPUT_OFF);
    sleep_microseconds(1000000); /* delay 2 seg */
    watchdog_init();
    measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
    measurement->current_profiler_signature.current_profiler_signature_status = MEASUREMENT_ACQUISITION_FAILURE;
    return 0;
}
