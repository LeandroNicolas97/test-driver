/***************************************************************************
 *   file                 : modbus.h                                       *
 *   begin                : Nov 27, 2019                                   *
 *   copyright            : (C) 2019 by Innovex Tecnologias Ltda.          *
 *   email                : development@innovex.cl                         *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2020. Innovex.                                          *
 ***************************************************************************/

#ifndef MODBUS_H_
#define MODBUS_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * MODBUS function codes
 */
#define MODBUS_READ_COILS                       1
#define MODBUS_READ_DISCRETE_INPUTS             2
#define MODBUS_READ_HOLDING_REGISTERS           3
#define MODBUS_READ_INPUT_REGISTERS             4
#define MODBUS_WRITE_SINGLE_COIL                5
#define MODBUS_WRITE_SINGLE_HOLDING_REGISTER    6
#define MODBUS_WRITE_MULTIPLE_COILS             15
#define MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS 16

#define MODBUS_MAX_BUFFER_SIZE 128

#define MSB_FIRST 1
#define LSB_FIRST 0

#define BIG_ENDIAN    0
#define LITTLE_ENDIAN 1

/**
 * Modbus structure
 */
struct modbus_frame {
    uint8_t slave_address;
    uint8_t function_code;
    uint16_t register_address;
    uint16_t n_coils;
    uint16_t data[30];
};

union floatingPointIEEE754 {
    struct {
        uint16_t lsb: 16; /* lsb */
        uint16_t msb: 16; /* msb */
    } raw;
    float f;
};

union DoubleIEEE754 {
    struct {
        uint16_t byte1: 16;
        uint16_t byte2: 16;
        uint16_t byte3: 16;
        uint16_t byte4: 16;
    } raw;
    double d;
};

/**
 * Modbus data fields
 */
enum modbus_message {
    ID,        /* ID field */
    FUNC,      /* Function code position */
    ADDR_HI,   /* Address high bytes */
    ADDR_LO,   /* Address low byte */
    NCOILS_HI, /* Number of coils or registers high byte */
    NCOILS_LO, /* Number of coils or registers low byte */
    BYTE_CNT,  /* byte counter */
    DATA
};

/**
 * Make a modbus query
 * @param serial_port The serial port to be used
 * @param frame From where the modbus data is extracted to be sended over serial interface
 */
int modbus_query(const uint8_t serial_port, const struct modbus_frame *frame);

/**
 * Fill the modbus response structure with the data received from serial, and act accordingly to this.
 * @param serial_port The serial port to be used
 * @param frame The modbus structure to be filled
 * @return 0 if Ok, negative on error
 */
int modbus_poll(const uint8_t serial_port, struct modbus_frame *frame, bool endian);

/**
 * Converting floating-point number to IEEE754
 * @param data_sb Data pointer most significant bit
 * @return Floating-point number converted
 */
float modbus_get_float(const uint16_t *data_sb);

#endif /* MODBUS_H_ */
