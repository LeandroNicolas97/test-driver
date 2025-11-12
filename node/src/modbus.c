/*
 * Communication via MODBUS
 */

#include <string.h>
#include "serial.h"
#include "timeutils.h"
#include "errorcodes.h"
#include "microio.h"
#include "crc16.h"
#include "hardware.h"
#include "modbus.h"
#include "debug.h"
#include "watchdog.h"

#define MAX_RESPONSE_SIZE 100
#define TIMEOUT_MS        500

/**
 * Send a MODBUS frame over the specified serial line. The CRC16 is calculated
 * and appended to the frame.
 * @param serial_port The serial pport to use
 * @param frame A pointer to the frame to be sended
 * @param max_size Maximum size of the frame buffer
 */
static void modbus_send_frame(uint8_t serial_port, uint8_t *frame, int max_size)
{
    uint16_t crc = 0xFFFF;
    int i;
    uint8_t c;

    rs485_transmit(UART_SMART_SENSOR);
    sleep_microseconds(100); /* 2000 */
    for (i = 0; i < max_size; i++) {
        c = *frame;
        serial_putchar(serial_port, c);
        crc = crc16_update(crc, c);
        frame++;
    }
    serial_putchar(serial_port, crc & 0xFF);
    serial_putchar(serial_port, crc >> 8);
    serial_drain(serial_port);
    rs485_receive(UART_SMART_SENSOR);
}

/**
 * Get a response from the MODBUS
 * @param serial_port The serial port to use
 * @param response A pointer to a buffer to store the received data
 * @param max_size Maximum size of the receive buffer
 * @return The number of bytes stored in the buffer
 */
static int modbus_get_response(uint8_t serial_port, uint8_t *response, int max_size)
{
    int64_t start;
    int n = 0;
    int c;

    rs485_receive(UART_SMART_SENSOR);
    start = get_uptime_ms();

    while (n < max_size) {
        watchdog_reset();
        c = serial_getchar(serial_port);
        if (c < 0) {
            /* For now the max response time is 200ms, so we use 400ms to be sure is a timeout*/
            if (ms_elapsed(&start) > TIMEOUT_MS) {
                break;
            }
        } else {
            *response++ = c;
            start = get_uptime_ms();
            n++;
        }
    }
    return n;
}

/**
 * Check if the MODBUS frame received is valid.
 * If frame is valid, return 0, negative otherwise
 * @param frame A pointer to the frame
 * @param length the length of the frame
 * @return 0 if OK, negative on error
 * TODO Maybe include the received CRC in the sum and check if result is 0.
 */
static int modbus_check_frame(uint8_t *frame, int length)
{
    uint16_t crc = 0xFFFF;
    uint16_t received_crc;
    int i;

    if (length < 5) {
        return -E_INVALID; /* At least 7 bytes for a valid frame.  5 or 7??*/
    }
    /* Calculate CRC of received data */
    for (i = 0; i < (length - 2); i++) {
        crc = crc16_update(crc, frame[i]);
    }
    /* Verify the checksum */
    received_crc = (((uint16_t)frame[length - 1]) << 8) | (frame[length - 2]);
    if (crc != received_crc) {
        return -E_BAD_CHECKSUM;
    }
    return 0;
}

/**
 * Converting floating-point number to IEEE754
 * @param data_sb Data pointer most significant bit
 * @return Floating-point number converted
 */
float modbus_get_float(const uint16_t *data_sb)
{

    union floatingPointIEEE754 endian;

    endian.raw.msb = data_sb[0];
    endian.raw.lsb = data_sb[1];

    return endian.f;
}

/**
 * Modbus function 3 and 4 according to the standard, i.e. reading holding/input registers
 * @param response The buffer where the response is allocated
 * @param frame The modbus structure who is filled with the response buffer data
 * @return the number of data bytes
 */
static int modbus_read_holding_input_register(uint8_t *response, struct modbus_frame *f, bool endianness)
{
    int i, n;

    f->slave_address = response[ID];
    f->function_code = response[FUNC];
    f->n_coils = (uint16_t)response[2];

    if (endianness == BIG_ENDIAN) {
        /*Big-endian format converter*/
        for (i = 0; i < f->n_coils / 2; i++) {
            f->data[i] = ((response[3 + (i * 2)] << 8)) | (response[4 + (i * 2)]);
        }
    } else {
        /*Little-endian format converter*/
        for (i = 0; i < f->n_coils / 4; i++) {
            f->data[i * 2] = ((response[5 + (i * 4)])) | (response[6 + (i * 4)] << 8);
            f->data[(i * 2) + 1] = ((response[3 + (i * 4)])) | (response[4 + (i * 4)] << 8);

            if (i == (f->n_coils / 4) - 1) {
                i = (f->n_coils / 2) - 1;
            }
        }
    }

    n = (i) * 2;
    return n;
}

/**
 * Make a modbus query
 * @param serial_port The serial port to be used
 * @param frame From where the modbus data is extracted to be sended over serial interface
 */
int modbus_query(const uint8_t serial_port, const struct modbus_frame *f)
{
    int size = -1;
    uint8_t buffer[7 + f->n_coils]; /* maximum size needed is when writing multiple registers */

    buffer[ID] = f->slave_address;
    buffer[FUNC] = f->function_code;
    buffer[ADDR_HI] = (f->register_address >> 8) & 0xFF;
    buffer[ADDR_LO] = f->register_address & 0xFF;
    buffer[NCOILS_HI] = (f->n_coils >> 8) & 0xFF;
    buffer[NCOILS_LO] = f->n_coils & 0xFF;

    switch (f->function_code) {
        case MODBUS_READ_COILS:
            /*Not implemented*/
            break;
        case MODBUS_READ_DISCRETE_INPUTS:
            /*Not implemented*/
            break;
        case MODBUS_READ_HOLDING_REGISTERS:
            size = 6;
            /* char cadene[63]; */
            /* DEBUG("send-read-----"); */
            /* for (int i = 0; i < size; i++){ */
            /*     itoa(buffer[i],cadene, 16); */
            /*     DEBUG("%s-", cadene); */
            /* } */
            /* DEBUG("\n"); */
            break;
        case MODBUS_READ_INPUT_REGISTERS:
            size = 6;
            break;
        case MODBUS_WRITE_SINGLE_COIL:
            size = 6;
            /* char cadena[63]; */
            /* DEBUG("send-write-----"); */
            /* for (int i = 0; i < size; i++){ */
            /*     itoa(buffer[i],cadena, 16); */
            /*     DEBUG("%s-", cadena); */
            /* } */
            /* DEBUG("\n"); */
            /* memset(response, '\0', sizeof(response)); */
            break;
        case MODBUS_WRITE_SINGLE_HOLDING_REGISTER:
            size = 6;
            break;
        case MODBUS_WRITE_MULTIPLE_COILS:
            /*Not implemented*/
            break;
        case MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS:
            buffer[BYTE_CNT] = (uint8_t)(f->n_coils * 2);
            for (int i = 0; i < f->n_coils; i++) {
                buffer[DATA + (i * 2)] = (f->data[i] >> 8) & 0xFF;
                buffer[DATA + (i * 2) + 1] = f->data[i] & 0xFF;
            }
            size = 7 + (f->n_coils * 2);
            break;
        default:
            break;
    }
    if (size < 0) {
        return -1; /* send/implement correct error code */
    }

    DEBUG("<<<Query: ");
    for (int i = 0; i < size; i++) {
        watchdog_reset();
        DEBUG("%.2x ", buffer[i]);
    }
    DEBUG("  size: %i\n", size);

    serial_flush(serial_port);
    modbus_send_frame(serial_port, buffer, size);
    return 0;
}

/**
 * Fill the modbus response structure with the data received from serial, and act accordingly to this.
 * @param serial_port The serial port to be used
 * @param frame The modbus structure to be filled
 * @return 0 if Ok, negative on error
 */
int modbus_poll(const uint8_t serial_port, struct modbus_frame *f, bool endianness)
{
    uint8_t response[MAX_RESPONSE_SIZE];
    int size, status;
    int n = 0; /* TODO: significant name */

    size = modbus_get_response(serial_port, response, sizeof(response));

    DEBUG(">>>RESP: ");
    for (int i = 0; i < size; i++) {
        watchdog_reset();
        DEBUG("%.2x ", response[i]);
    }
    DEBUG("  size: %i\n", size);

    if (size == 0) {
        return -E_NOT_DETECTED;
    }

    status = modbus_check_frame(response, size);
    if (status == -E_BAD_CHECKSUM) {
        return -E_BAD_CHECKSUM;
    }
    if (status == -E_INVALID) {
        return -E_INVALID;
    }

    switch (response[FUNC]) {
        case MODBUS_READ_COILS:
            /* Not implemented */
            break;
        case MODBUS_READ_DISCRETE_INPUTS:
            /* Not implemented */
            break;
        case MODBUS_READ_HOLDING_REGISTERS:
            n = modbus_read_holding_input_register(response, f, endianness);
            break;
        case MODBUS_READ_INPUT_REGISTERS:
            n = modbus_read_holding_input_register(response, f, endianness);
            break;
        case MODBUS_WRITE_SINGLE_COIL:
            n = f->n_coils;
            break;
        case MODBUS_WRITE_SINGLE_HOLDING_REGISTER:
            n = f->n_coils;
            break;
        case MODBUS_WRITE_MULTIPLE_COILS:
            /* Not implemented */
            break;
        case MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS:
            /* Not implemented */
            n = f->n_coils;
            break;
        default:
            break;
    }
    if (n != f->n_coils) {
        return -E_INVALID;
    }
    return 0;
}
