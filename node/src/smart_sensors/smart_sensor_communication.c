/**
 *  \file smart_sensor_communication.c
 *  \brief Communication with the smart sensors
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 22.06.2021
 *
 *  Copyright 2021 Innovex Tecnologias Ltda. All rights reserved.
 */

#include <string.h>
#include <stdlib.h>
#include "serial.h"
#include "microio.h"
#include "timeutils.h"
#include "hardware.h"
#include "debug.h"
#include "smart_sensor.h"
#include "errorcodes.h"
#include "watchdog.h"

/* Serial ports buffers */
#define SERIAL_BUFFER_SIZE 32
static uint8_t rx_buffer[SERIAL_BUFFER_SIZE];
static uint8_t tx_buffer[SERIAL_BUFFER_SIZE];

/**
 * Initialize the serial port used to communicate with the smart sensor
 */
void smart_sensor_init_serial_port(void)
{
    serial_init(UART_SMART_SENSOR, 9600, SERIAL_BUFFER_SIZE, rx_buffer, tx_buffer);
}

/* TODO Use the same timeout mechanism as smart_sensor_receive_data */
#define RESPONSE_TIMEOUT 500000 /* To try to get an answer from the sensor */

/**
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @return The number of bytes stored in the buffer
 */
int smart_sensor_get_response(char *response, uint8_t size)
{
    uint32_t t = 0;
    int n = 0;
    int c;

    while (1) {
        watchdog_disable();
        c = serial_getchar(UART_SMART_SENSOR);
        if (c < 0) {
            if (++t > RESPONSE_TIMEOUT) {
                DEBUG("--Sensor timeout %i\n", RESPONSE_TIMEOUT);
                n = -E_TIMEDOUT;
                break;
            }
        } else {
            *response++ = c;
            t = 0;
            n++;
            if (n >= (size - 2)) {
                break;
            }
        }
    }
    *response = '\0';
    watchdog_init();
    return n;
}

/**
 * Get as much as possible data from a sensor until the amount specified
 * is full or the sensor stopped sending data.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @return The number of bytes stored in the buffer
 */
int smart_sensor_receive_data(char *response, uint8_t size)
{
    int n = 0;
    int c;

    int64_t ms_start = get_uptime_ms();

    DEBUG("ms start: %lli\n", ms_start);
    while (1) {
        c = serial_getchar(UART_SMART_SENSOR);
        if (c < 0) {
            if (ms_elapsed(&ms_start) > 1000) {
                DEBUG("--Sensor timeout\n");
                break;
            }
        } else {
            *response++ = c;
            ms_start = get_uptime_ms();
            n++;
            if (n >= (size - 2)) {
                break;
            }
        }
    }
    DEBUG("Received: %i\n", n);
    *response = '\0';
    return n;
}
