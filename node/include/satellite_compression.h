/***************************************************************************
 *   file                 : satellite_compression.h                        *
 *   begin                : Sep  09, 2022                                  *
 *   copyright            : (C) 2022 by Innovex Tecnologias Ltda.          *
 *   email                : development@innovex.cl                         *
 *                                                                         *
 *   This program is property of Innovex Tecnologias SpA. Chile.           *
 *   Copyright (C) 2022. Innovex.                                          *
 ***************************************************************************/

#ifndef SATELLITE_COMPRESSION_H
#define SATELLITE_COMPRESSION_H

#include "measurement.h"
#include "adcp.h"

/*
 * Functions and structures to compress and uncompress measurements into
 * a packet to be send via satellite. Simply reduce every measurement to
 * a fixed length word achieving the required resolution and range.
 */

/**
 * Structure used to create a table with the compression parameters
 * for every variable.
 */
struct compression_param {
    float min;
    float max;
    float resolution;
};

/**
 * Names of the variables that can be compressed. Every
 * variable has an entry in the compression parameter table.
 */
enum variable_name {
    /* ADCP specific variables */
    BATTERY_VOLTAGE,
    ADCP_TEMPERATURE,
    ADCP_PRESSURE,
    ADCP_ANGLE,   /* Pitch, roll */
    ADCP_HEADING, /* Heading */
    ADCP_SPEED,
    ADCP_DIRECTION,
    ADCP_CELLS,
};

/**
 * Compress a variable into a 16 bit word with the specified resolution.
 * @return the compressed variable
 */
uint16_t compress_variable(enum variable_name name, float value);

/**
 * De-ompress a variable from a 16 bit word into a float.
 * If the values was over or under-range, set the maximum
 * or minimum value.
 * @return the decompressed variable
 */
float decompress_variable(enum variable_name name, uint16_t c);

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
void pack_compressed_data(uint8_t *buffer, int bit_position, int bits, uint16_t data);

/**
 * Unpack one word of the specified size from the buffer.
 * @param buffer The buffer with all the packed data
 * @param bit_position The bit position in the complete buffer
 * @param bits The size in bits of the data to extract
 * @return The data word of the specified bit size.
 */
uint16_t unpack_compressed_data(uint8_t *buffer, int bit_position, int bits);

/**
 * Pack a status value (use only 4 bits)
 * @param buffer The buffer where to store the variable
 * @param pos The position where to store the status
 * @param status The actual status value
 * @return The number of bits used to store the status
 */
int pack_measurement_status(uint8_t *buffer, int pos, enum measurement_status status);

/**
 * Extract the status of a measurement
 * @param buffer The buffer where the data is stored
 * @param pos The position where the status is stored
 * @param status A pointer to store the extracted status value
 * @return The number of bits used to store the variable
 */
int unpack_measurement_status(uint8_t *buffer, int pos, enum measurement_status *status);

/**
 * Compress an ADCP measurement. Pack everything in an array.
 * @param adcp A pointer to the adcp measurement
 * @param compressed A pointer to store the compressed buffer
 * @return the size in bits of the compressed measurement
 */
int compress_adcp_measurement(struct adcp_data *adcp, uint8_t *compressed);

/**
 * Unpack and decompress an ADCP measurement.
 * @param compressed A pointer with the compressed buffer
 * @param adcp A pointer to extract the adcp measurement
 * @return the size in bits of the compressed measurement.
 */
int uncompress_adcp_measurement(uint8_t *compressed, struct adcp_data *adcp);

#endif /* SATELLITE_COMPRESSION_H */
