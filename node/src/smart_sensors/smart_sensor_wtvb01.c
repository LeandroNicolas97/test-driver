/**
 *
 *  \file smart_sensor_wtvb01.c
 *
 *  \author Leandro Atero Catalan
 *
 *  \date  Noviembre 2025
 *
 *  Copyright 2025 Innovex Tecnologías Ltda. All rights reserved.
 */

#include <zephyr/kernel.h>
#include "measurement.h"
#include "zephyr/sys/printk.h"
#include <string.h>
#include <stdlib.h>
#include "bsp-config.h"
#include "smart_sensor.h"
#include "serial.h"
#include "errorcodes.h"
#include "modbus.h"
#include "debug.h"
#include "hardware.h"
#include "wtvb01.h"
#include "watchdog.h"
#include "crc16.h"
#include "timeutils.h"

#define DETECTION_TRIES   3
#define MAX_SENSORS       1

/* MODBUS default address for WTVB01 */
#define DEVICE_ADDRESS 0x50

/* UART baudrate for WTVB01 */
#define WTVB01_BAUDRATE 9600

#define WTVB01_RESPONSE_DELAY_MS    150
#define WTVB01_RETRY_DELAY_MS       200
#define WTVB01_PREPARE_RETRIES      5

/* Driver function prototypes */
static int max_sensors(void);
static const char *name(void);
static int init_driver(void);
static int finish_driver(void);
static int detect(int sensor_number, struct smart_sensor *sensor);
static int prepare(struct smart_sensor *sensor);
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m);
static int needs_external_voltage(void);

/* Smart sensor driver structure */
const struct smart_sensor_driver smart_sensor_driver_wtvb01 = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .finish = NULL,
    .calibrate_zero = NULL,
    .calibrate_full = NULL,
    .acquire = acquire,
    .pass_command = NULL,
    .name = name,
    .needs_external_voltage = needs_external_voltage,
};

/* Local prototypes */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg, uint16_t count);
static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement);
static int get_parameter_u16(uint16_t *regs, int n_regs, struct smart_sensor *sensor, struct measurement *m, uint16_t reg);

static int max_sensors(void)
{
    return MAX_SENSORS;
}

static const char *name(void)
{
    return "WTVB01";
}

static int init_driver(void)
{
    /* Configure UART baudrate for WTVB01 sensor */
    serial_set_baudrate(UART_SMART_SENSOR, WTVB01_BAUDRATE);

    sleep_microseconds(200000);  // 200ms para estabilización

    serial_flush(UART_SMART_SENSOR);
    sleep_microseconds(100000);  // 100ms adicional

    return 0;
}

static int finish_driver(void)
{
    return 0;
}

#define WTVB01_MAX_WAIT_MS 2000
#define WTVB01_INTER_BYTE_TIMEOUT_MS 100

static int wtvb01_wait_for_response(uint8_t *buffer, int max_size, int expected_bytes)
{
    int64_t start;
    int64_t last_byte_time;
    int n = 0;
    int c;

    rs485_receive(UART_SMART_SENSOR);
    start = get_uptime_ms();
    last_byte_time = start;

    while (n < max_size) {
        watchdog_reset();
        c = serial_getchar(UART_SMART_SENSOR);

        if (c < 0) {
            if (n > 0) {
                if (ms_elapsed(&last_byte_time) > WTVB01_INTER_BYTE_TIMEOUT_MS) {
                    DEBUG("Inter-byte timeout after %d bytes\n", n);
                    break;
                }
            } else {
                if (ms_elapsed(&start) > WTVB01_MAX_WAIT_MS) {
                    DEBUG("Initial byte timeout\n");
                    break;
                }
            }
        } else {
            buffer[n++] = c;
            last_byte_time = get_uptime_ms();

            if (n >= expected_bytes) {
                break;
            }
        }
    }

    return n;
}

static int wtvb01_verify_response(uint8_t *buffer, int size, uint8_t expected_addr, uint8_t expected_func)
{
    uint16_t crc_received;
    uint16_t crc_calculated = 0xFFFF;

    if (size < 5) {
        DEBUG("Response too short: %d bytes\n", size);
        return -E_INVALID;
    }

    if (buffer[0] != expected_addr) {
        DEBUG("Wrong address: expected 0x%02x, got 0x%02x\n", expected_addr, buffer[0]);
        return -E_INVALID;
    }

    if (buffer[1] != expected_func) {
        DEBUG("Wrong function: expected 0x%02x, got 0x%02x\n", expected_func, buffer[1]);
        return -E_INVALID;
    }

    for (int i = 0; i < (size - 2); i++) {
        crc_calculated = crc16_update(crc_calculated, buffer[i]);
    }

    crc_received = buffer[size - 2] | (buffer[size - 1] << 8);

    if (crc_calculated != crc_received) {
        DEBUG("CRC error: calculated 0x%04x, received 0x%04x\n", crc_calculated, crc_received);
        return -E_BAD_CHECKSUM;
    }

    return 0;
}

static int wtvb01_read_registers(uint8_t addr, uint16_t reg, uint16_t count, uint16_t *data)
{
    uint8_t cmd[8];
    uint8_t response[256];
    uint16_t crc;
    int response_size;
    int expected_response_size;
    int data_length;

    cmd[0] = addr;
    cmd[1] = MODBUS_READ_HOLDING_REGISTERS;
    cmd[2] = (reg >> 8) & 0xFF;
    cmd[3] = reg & 0xFF;
    cmd[4] = (count >> 8) & 0xFF;
    cmd[5] = count & 0xFF;

    crc = 0xFFFF;
    for (int i = 0; i < 6; i++) {
        crc = crc16_update(crc, cmd[i]);
    }
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;

    DEBUG("<<<Manual Query: ");
    for (int i = 0; i < 8; i++) {
        DEBUG("%.2x ", cmd[i]);
    }
    DEBUG("\n");

    serial_flush(UART_SMART_SENSOR);
    sleep_microseconds(100000);

    rs485_transmit(UART_SMART_SENSOR);
    sleep_microseconds(100);

    for (int i = 0; i < 8; i++) {
        serial_putchar(UART_SMART_SENSOR, cmd[i]);
    }
    serial_drain(UART_SMART_SENSOR);

    rs485_receive(UART_SMART_SENSOR);

    expected_response_size = 5 + (count * 2);
    response_size = wtvb01_wait_for_response(response, sizeof(response), expected_response_size);

    DEBUG(">>>Manual RESP: ");
    for (int i = 0; i < response_size; i++) {
        DEBUG("%.2x ", response[i]);
    }
    DEBUG("  size: %d (expected: %d)\n", response_size, expected_response_size);

    if (response_size == 0) {
        return -E_NOT_DETECTED;
    }

    int verify_result = wtvb01_verify_response(response, response_size, addr, MODBUS_READ_HOLDING_REGISTERS);
    if (verify_result != 0) {
        return verify_result;
    }

    data_length = response[2];
    if (data_length != count * 2) {
        DEBUG("Wrong data length: expected %d, got %d\n", count * 2, data_length);
        return -E_INVALID;
    }

    for (int i = 0; i < count; i++) {
        data[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
    }

    return 0;
}


static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *s, uint8_t function,
                                 uint16_t reg, uint16_t count)
{
    /* WTVB01 has fixed address 0x50 (only 1 sensor supported) */
    f->slave_address = DEVICE_ADDRESS;
    f->function_code = function;
    f->register_address = reg;
    f->n_coils = count;
}


static int prepare(struct smart_sensor *sensor)
{
    uint16_t temp_reg;
    int result;

    result = wtvb01_read_registers(DEVICE_ADDRESS, WTVB01_REG_TEMP, 1, &temp_reg);

    DEBUG("Prepare result: %d\n", result);

    if (result == 0) {
        int16_t temp_signed = (int16_t)temp_reg;
        float temp = temp_signed / 100.0f;
        DEBUG("Temperature: %.2f C\n", (double)temp);
    }

    return result;
}

static int detect(int sensor_number, struct smart_sensor *sensor)
{
    DEBUG("Checking WTVB01-485 %i... ", sensor_number);

    if (sensor_number != 0) {
        DEBUG("NO (invalid sensor number)\n");
        return 0;
    }

    sensor->number = sensor_number;

    for (int tries = 0; tries < WTVB01_PREPARE_RETRIES; tries++) {
        if (tries > 0) {
            DEBUG("Retry %d/%d... ", tries + 1, WTVB01_PREPARE_RETRIES);
            serial_flush(UART_SMART_SENSOR);
            sleep_microseconds(WTVB01_RETRY_DELAY_MS * 1000);
        }

        if (prepare(sensor) == 0) {
            sensor->manufacturer = WTVB01_MANUFACTURER;
            sensor->power_up_time = WTVB01_STARTUP_TIME_MS;
            sensor->type = VIBRATION_SENSOR;
            sensor->channel = 0;
            strcpy(sensor->name, name());
            DEBUG("OK\n");
            return 1;
        }
    }

    DEBUG("NO\n");
    return 0;
}

static int get_parameter_u16(uint16_t *regs, int n_regs, struct smart_sensor *sensor,
                             struct measurement *m, uint16_t reg)
{
    int rc = wtvb01_read_registers(DEVICE_ADDRESS, reg, n_regs, regs);

    if (rc == -E_NOT_DETECTED) {
        m->sensor_status = SENSOR_NOT_DETECTED;
        return 0;
    }
    if (rc == -E_BAD_CHECKSUM) {
        m->sensor_status = SENSOR_COMMUNICATION_BAD_CRC;
        return 0;
    }
    if (rc < 0) {
        m->sensor_status = SENSOR_COMMUNICATION_ERROR;
        return 0;
    }

    return 1;
}

static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    int rc;
    uint16_t regs[10];
    int16_t signed_val;

    if (sensor == NULL || measurement == NULL) {
        return 0;
    }

    rc = get_parameter_u16(regs, 3, sensor, measurement, WTVB01_REG_VX);
    if (rc == 0) {
        return 0;
    }
    DEBUG("WTVB01 Velocity: X=%u Y=%u Z=%u mm/s\n", regs[0], regs[1], regs[2]);
    sleep_microseconds(20000);

    rc = get_parameter_u16(&regs[3], 1, sensor, measurement, WTVB01_REG_TEMP);
    if (rc == 0) {
        return 0;
    }
    signed_val = (int16_t)regs[3];
    float temp = (float)signed_val / 100.0f;
    DEBUG("WTVB01 Temperature: %.2f C\n", (double)temp);
    sleep_microseconds(20000);

    rc = get_parameter_u16(&regs[4], 3, sensor, measurement, WTVB01_REG_DX);
    if (rc == 0) {
        return 0;
    }
    DEBUG("WTVB01 Displacement: X=%u Y=%u Z=%u um\n", regs[4], regs[5], regs[6]);
    sleep_microseconds(20000);

    rc = get_parameter_u16(&regs[7], 3, sensor, measurement, WTVB01_REG_HZX);
    if (rc == 0) {
        return 0;
    }
    DEBUG("WTVB01 Frequency: X=%.1f Y=%.1f Z=%.1f Hz\n",
          (double)(regs[7] / 10.0f), (double)(regs[8] / 10.0f), (double)(regs[9] / 10.0f));

    measurement->type = VIBRATION_SENSOR;
    measurement->sensor_number = sensor->number;

    struct vibration_measurement m;

    m.velocity_x = (float)regs[0];
    m.velocity_y = (float)regs[1];
    m.velocity_z = (float)regs[2];
    m.velocity_status = MEASUREMENT_OK;

    m.temperature = temp;
    m.temperature_status = MEASUREMENT_OK;

    m.displacement_x = (float)regs[4];
    m.displacement_y = (float)regs[5];
    m.displacement_z = (float)regs[6];
    m.displacement_status = MEASUREMENT_OK;

    m.frequency_x = (float)regs[7] / 10.0f;
    m.frequency_y = (float)regs[8] / 10.0f;
    m.frequency_z = (float)regs[9] / 10.0f;
    m.frequency_status = MEASUREMENT_OK;

    measurement->sensor_status = SENSOR_OK;

    memcpy(&measurement->vibration, &m, sizeof(struct vibration_measurement));

    return 1;
}

static int acquire(int tries, struct smart_sensor *sensor, struct measurement *measurement)
{
    if (sensor == NULL || measurement == NULL) {
        DEBUG("ERROR: NULL pointer in acquire\n");
        return 0;
    }

    int original_tries = tries;

    while (tries > 0) {
        DEBUG("Trying (attempt %d/%d)\n", original_tries - tries + 1, original_tries);
        if (modbus_request_measurement(sensor, measurement)) {
            return 1;
        }
        DEBUG("Error reading sensor %s (remaining tries: %d)\n", sensor->name, tries - 1);
        tries--;

        if (tries > 0) {
            serial_flush(UART_SMART_SENSOR);
            sleep_microseconds(100000);
        }
    }

    measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
    measurement->type = VIBRATION_SENSOR;
    measurement->sensor_number = sensor->number;
    return 0;
}


static int needs_external_voltage(void)
{
    return 1;
}
