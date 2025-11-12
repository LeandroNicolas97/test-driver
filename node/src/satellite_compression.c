/***************************************************************************
 *   file                 : satellite_compression.c                        *
 *   begin                : Sep  09, 2022                                  *
 *   copyright            : (C) 2022 by Innovex Tecnologias Ltda.          *
 *   email                : development@innovex.cl                         *
 *                                                                         *
 *   This program is property of Innovex Tecnologias SpA. Chile.           *
 *   Copyright (C) 2022. Innovex.                                          *
 ***************************************************************************/

#include "satellite_compression.h"
#include "flowquest.h"
#include "measurement.h"
#include <math.h>

/**
 * Range and resolution of all the variables used.
 * The same table must be used for compression and decompression.
 */
const struct compression_param c_meas[] = {
    /* Variable             MIN      MAX    Resolution */

    [BATTERY_VOLTAGE] = {0.0,  25.0,  0.1}, /* Volt */
    [ADCP_TEMPERATURE] = {0.0,  25.0,  0.1}, /* Celsius */
    [ADCP_PRESSURE] = {0.0,  250.0, 0.1}, /* deciBar */
    [ADCP_ANGLE] = {-180, 180.0, 1  }, /* Pitch, roll */
    [ADCP_HEADING] = {0.0,  360.0, 1  }, /* Heading */
    [ADCP_SPEED] = {0.0,  500.0, 0.5}, /* cm/s */
    [ADCP_DIRECTION] = {0.0,  360.0, 5  },
    [ADCP_CELLS] = {0.0,  250.0, 1.0},
};

/**
 * Get the number of bits required to store a compressed variable
 * plus two extra values representing under and over range.
 * For example: range -50 to 150 with resolution of 0.1
 * requires 2000 counts plus 2 counts to keep over/under range
 * This is 2002 counts. This fits into a 12 bits word.
 */
int number_of_bits(enum variable_name name)
{
    int bits;
    const struct compression_param *p = &(c_meas[name]);
    int ext_range = floor((p->max - p->min + 2) / p->resolution);

    for (bits = 15; bits > 0; bits--) {
        if ((ext_range & (1 << bits)) != 0) {
            /* Found a 1 */
            break;
        }
    }
    /* If we found the bit at position 10, it has 11 bits */
    return bits + 1;
}

/**
 * Under-range value for the specified number of bits.
 */
uint16_t under_range_value(int bits)
{
    return (1 << bits) - 2;
}

/**
 * Over-range value for the specified number of bits.
 */
uint16_t over_range_value(int bits)
{
    return (1 << bits) - 1;
}

/**
 * Compress a variable into a 16 bit word with the specified resolution.
 * @return the compressed variable
 */
uint16_t compress_variable(enum variable_name name, float value)
{
    uint16_t c;
    int bits = number_of_bits(name);
    const struct compression_param *p = &(c_meas[name]);

    if (value < p->min) {
        c = under_range_value(bits);
    } else if (value > p->max) {
        c = over_range_value(bits);
    } else {
        c = (uint16_t)lround((value - p->min) / p->resolution);
    }
    return c;
}

/**
 * Decompress a variable from a 16 bit word into a float.
 * If the values was over or under-range, set the maximum
 * or minimum value.
 * @return the decompressed variable
 */
float decompress_variable(enum variable_name name, uint16_t c)
{
    float v;
    int bits = number_of_bits(name);
    const struct compression_param *p = &(c_meas[name]);

    if (c == under_range_value(bits)) {
        v = p->min;
    } else if (c == over_range_value(bits)) {
        v = p->max;
    } else {
        v = (c * p->resolution) + p->min;
    }
    return v;
}

/**
 * Pack a compressed variable into an array of bits using the specified
 * number of bits for every data word.
 *
 * +--------+--------+--------+--------+--------+--...
 * |        |        |        |        |        |    buffer
 * +--------+--------+--------+--------+--------+--...
 * +-----+------------+--------+-----+-------+...
 * |5bits| 12 bits    | 8 bits |5bits| 7bits |    data
 * +-----+------------+--------+-----+-------+...
 *
 *
 * @param buffer The buffer to pack all the compressed data
 * @param bit_position The bit position in the complete buffer
 * @param bits The size in bits of the compressed data
 * @param data The compressed data to pack. Only the amount of bits specified are packed.
 */
void pack_compressed_data(uint8_t *buffer, int bit_position, int bits, uint16_t data)
{
    uint32_t mask;
    uint32_t data_mask;
    uint32_t clean_mask;
    int byte_position = bit_position / 8;
    int byte_offset = bit_position % 8;
    int bits_to_shift = 24 - byte_offset - bits;

    mask = (1 << bits) - 1; /* Mask with all the used bits to 1 */
    data_mask = data;
    data_mask &= mask; /* Clean unused bits */

    clean_mask = ~(mask << bits_to_shift); /* Inverted bits */
    data_mask <<= bits_to_shift;

    /* Apply masks and copy data */
    *(buffer + byte_position + 0) &= ((clean_mask >> 16) & 0xFF);
    *(buffer + byte_position + 0) |= ((data_mask >> 16) & 0xFF);
    *(buffer + byte_position + 1) &= ((clean_mask >> 8) & 0xFF);
    *(buffer + byte_position + 1) |= ((data_mask >> 8) & 0xFF);
    *(buffer + byte_position + 2) &= ((clean_mask >> 0) & 0xFF);
    *(buffer + byte_position + 2) |= ((data_mask >> 0) & 0xFF);
}

/**
 * Unpack one word of the specified size from the buffer.
 * @param buffer The buffer with all the packed data
 * @param bit_position The bit position in the complete buffer
 * @param bits The size in bits of the data to extract
 * @return The data word of the specified bit size.
 */
uint16_t unpack_compressed_data(uint8_t *buffer, int bit_position, int bits)
{
    uint32_t data;
    uint32_t clean_mask;
    int byte_position = bit_position / 8;
    int byte_offset = bit_position % 8;
    int bits_to_shift = 24 - byte_offset - bits;

    data = (*(buffer + byte_position + 0) << 16) | (*(buffer + byte_position + 1) << 8) | *(buffer + byte_position + 2);

    clean_mask = (1 << bits) - 1; /* Mask with all the used bits to 1 */
    data >>= bits_to_shift;
    data &= clean_mask;
    return (uint16_t)data;
}

/**
 * Compress and pack a variable
 * @param buffer The buffer where to store the variable
 * @param pos The position where to store the compressed variable
 * @param name The name of the variable
 * @param value The value of the variable
 * @return The number of bits used to store the variable
 */
int compress_and_pack_variable(uint8_t *buffer, int pos, enum variable_name name, float value)
{
    int bits = number_of_bits(name);
    uint16_t c16 = compress_variable(name, value);

    pack_compressed_data(buffer, pos, bits, c16);
    return bits;
}

/**
 * Extract and uncompress a variable
 * @param buffer The buffer where to store the variable
 * @param pos The position where to store the compressed variable
 * @param name The name of the variable
 * @param value A pointer to store the value of the variable
 * @return The number of bits used to store the variable
 */
int unpack_and_decompress_variable(uint8_t *buffer, int pos, enum variable_name name, float *value)
{
    int bits = number_of_bits(name);
    uint16_t c16 = unpack_compressed_data(buffer, pos, bits);
    *value = decompress_variable(name, c16);
    return bits;
}

/**
 * Pack a status value (use only 4 bits)
 * @param buffer The buffer where to store the variable
 * @param pos The position where to store the status
 * @param status The actual status value
 * @return The number of bits used to store the status
 */
int pack_measurement_status(uint8_t *buffer, int pos, enum measurement_status status)
{
    int bits = 4;
    uint16_t to_pack = (uint16_t)status;

    pack_compressed_data(buffer, pos, bits, to_pack);
    return bits;
}

/**
 * Extract the status of a measurement
 * @param buffer The buffer where the data is stored
 * @param pos The position where the status is stored
 * @param status A pointer to store the extracted status value
 * @return The number of bits used to store the variable
 */
int unpack_measurement_status(uint8_t *buffer, int pos, enum measurement_status *status)
{
    int bits = 4;
    uint16_t c16 = unpack_compressed_data(buffer, pos, bits);
    *status = (enum measurement_status)c16;
    return bits;
}

/**
 * Pack a single byte
 * @param buffer The buffer where to store the variable
 * @param pos The position where to store the status
 * @param byte The value to pack
 * @return The number of bits used to store the status
 */
int pack_byte(uint8_t *buffer, int pos, int byte)
{
    int bits = 8;
    uint16_t to_pack = (uint16_t)(byte & 0xFF);

    pack_compressed_data(buffer, pos, bits, to_pack);
    return bits;
}

/**
 * Extract a single byte
 * @param buffer The buffer where the data is stored
 * @param pos The position where the status is stored
 * @param byte A pointer to store the extracted byte
 * @return The number of bits used to store the variable
 */
int unpack_byte(uint8_t *buffer, int pos, int *byte)
{
    int bits = 8;
    uint16_t c16 = unpack_compressed_data(buffer, pos, bits);
    *byte = c16;
    return bits;
}

uint32_t get_timestamp_from_bytes(const uint8_t *bytes, uint32_t *timestamp)
{
    uint32_t data;

    data = ((uint32_t)bytes[3] << 24) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[1] << 8) | (uint32_t)bytes[0];
    *timestamp = data;

    return 32; /* 32 bits, 4 bytes */
}

/**
 * Compress an ADCP measurement. Pack everything in an array.
 * @param adcp A pointer to the adcp measurement
 * @param compressed A pointer to store the compressed buffer
 * @return the size in bits of the compressed measurement
 */
int compress_adcp_measurement(struct adcp_data *adcp, uint8_t *compressed)
{
    int pos = 0;

    pos += compress_and_pack_variable(compressed, pos, ADCP_PRESSURE, adcp->pressure);
    pos += compress_and_pack_variable(compressed, pos, ADCP_TEMPERATURE, adcp->temperature);
    pos += compress_and_pack_variable(compressed, pos, ADCP_ANGLE, adcp->pitch);
    pos += compress_and_pack_variable(compressed, pos, ADCP_ANGLE, adcp->roll);
    pos += compress_and_pack_variable(compressed, pos, ADCP_HEADING, adcp->heading);
    pos += compress_and_pack_variable(compressed, pos, BATTERY_VOLTAGE, adcp->battery_voltage);
    pos += pack_byte(compressed, pos, adcp->blanking);
    pos += pack_byte(compressed, pos, adcp->cells);
    for (int i = 0; i < adcp->cells; i++) {
        pos += compress_and_pack_variable(compressed, pos, ADCP_SPEED, adcp->vel[i]);
        pos += compress_and_pack_variable(compressed, pos, ADCP_DIRECTION, adcp->dir[i]);
    }
    return pos;
}

/**
 * Unpack and decompress an ADCP measurement.
 * @param compressed A pointer with the compressed buffer
 * @param adcp A pointer to extract the adcp measurement
 * @return the size in bits of the compressed measurement.
 */
int uncompress_adcp_measurement(uint8_t *compressed, struct adcp_data *adcp)
{
    int pos = 0;

    pos += unpack_and_decompress_variable(compressed, pos, ADCP_PRESSURE, &(adcp->pressure));
    pos += unpack_and_decompress_variable(compressed, pos, ADCP_TEMPERATURE, &(adcp->temperature));
    pos += unpack_and_decompress_variable(compressed, pos, ADCP_ANGLE, &(adcp->pitch));
    pos += unpack_and_decompress_variable(compressed, pos, ADCP_ANGLE, &(adcp->roll));
    pos += unpack_and_decompress_variable(compressed, pos, ADCP_HEADING, &(adcp->heading));
    pos += unpack_and_decompress_variable(compressed, pos, BATTERY_VOLTAGE, &(adcp->battery_voltage));
    int temp_blanking;

    pos += unpack_byte(compressed, pos, &temp_blanking);
    adcp->blanking = (float)temp_blanking;
    pos += unpack_byte(compressed, pos, &(adcp->cells));
    for (int i = 0; i < adcp->cells; i++) {
        pos += unpack_and_decompress_variable(compressed, pos, ADCP_SPEED, &(adcp->vel[i]));
        pos += unpack_and_decompress_variable(compressed, pos, ADCP_DIRECTION, &(adcp->dir[i]));
    }
    return pos;
}
