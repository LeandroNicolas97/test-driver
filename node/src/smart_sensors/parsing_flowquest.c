/**
 *
 *  \file parsing_flowquest.c
 *  \brief Driver for parsing FlowQuest sensor data.
 *
 *  \author Raúl Baigorri Morales
 *  \author Sebastián Angulo Oyarzún
 *
 *  \date 18.02.2025
 *
 *  Copyright 2025 Innovex Tecnologías Ltda. All rights reserved.
 */
#include "debug.h"
#include "flowquest.h"
#include "nortek_signature.h"
#include <math.h>
#include <stdint.h>

static int adcp_cell_count;

int adcp_cell_count_get(void)
{
    return adcp_cell_count;
}

void adcp_cell_count_set(int count)
{
    adcp_cell_count = count;
}

/**
 * @brief  Compute the direction angle from velocity components.
 */
static float direction(float vel_east, float vel_north)
{
    float degrees;

    degrees = atan2(-1 * (vel_east), -1 * (vel_north));
    return (180.0f + 180.0f / 3.1416f * degrees);
}

/**
 * @brief  Compute the speed magnitude from velocity components.
 */
static float speed(float vel_east, float vel_north)
{
    float total;

    total = (vel_east * vel_east) + (vel_north * vel_north);
    return sqrt(total);
}

static int16_t le_to_i16(uint8_t *p)
{
    return (int16_t)(p[0] | (p[1] << 8));
}

/**
 * @brief  Parse a FlowQuest data frame into an ADCP raw data structure.
 */
int parse_flowquest_data_frame(int count_receive, uint8_t *frame, struct adcp_raw_data_flowquest *r)
{
    if (!frame || !r) {
        return -1;
    }

    r->pressure = 0.0f;
    r->cells = 0;

    if (count_receive < FLOWQUEST_HEADER_SIZE + MIN_COMMON_DATA_SIZE) {
        DEBUG("\nError: Trama demasiado corta (Recibido: %d bytes)\n", count_receive);
        return -1;
    }

    uint8_t *common_data = frame + FLOWQUEST_HEADER_SIZE;

    r->ensemble_number = le_to_u32(common_data + OFFSET_ENSEMBLE_NUMBER);
    r->temperature = le_to_u16(common_data + OFFSET_TEMPERATURE);
    r->battery_voltage = le_to_u16(common_data + OFFSET_BATTERY_VOLTAGE);
    r->num_pings = le_to_u16(common_data + OFFSET_NUM_PINGS);
    r->heading = le_to_u16(common_data + OFFSET_HEADING);
    r->pitch = le_to_u16(common_data + OFFSET_PITCH);
    r->roll = le_to_u16(common_data + OFFSET_ROLL);
    r->ensemble_output_switch = le_to_u16(common_data + OFFSET_ENSEMBLE_OUTPUT_SWITCH);
    r->bin_length = le_to_u16(common_data + OFFSET_BIN_LENGTH);
    r->blank_distance = le_to_u16(common_data + OFFSET_BLANK_DISTANCE);
    r->transducer_depth = le_to_u16(common_data + OFFSET_TRANSDUCER_DEPTH);
    r->error_code = le_to_u16(common_data + OFFSET_ERROR_CODE);
    r->rph_abnormal = le_to_u16(common_data + OFFSET_RPH_ABNORMAL);

    /* Search for blocks after the header */
    uint8_t *ptr = common_data + COMMON_HEADER_PAYLOAD_SIZE;
    int blocks_found = 0;

    while (ptr < frame + count_receive - BLOCK_HEADER_SIZE) {
        /* Checks if a block is found (blocks start with E) */
        if (ptr[0] == 'E' && ptr[1] >= '0' && ptr[1] <= '8') {
            char block_id = ptr[1];
            uint16_t length = le_to_u16(ptr + 2);
            uint8_t *data = ptr + BLOCK_HEADER_SIZE;

            blocks_found++;

            switch (block_id) {
                case '0': {
                    int16_t mr = (int16_t)le_to_i16(data);
                    int16_t mp = (int16_t)le_to_i16(data + 4);
                    uint16_t mh = (int16_t)le_to_i16(data + 8);

                    r->roll = mr;
                    r->pitch = mp;
                    r->heading = mh;
                    break;
                }

                case '4': /* E4: Earth Coordinates */
                    int bins_detected = length / VALUES_PER_VELOCITY_CELL;

                    if (bins_detected > 200) {
                        for (int i = -8; i < 16; i++) {
                            if (ptr + i >= frame && ptr + i < frame + count_receive) {
                            }
                        }
                        break;
                    }
                    /* The number of cells is set with respect to the detected header bins */
                    r->cells = bins_detected;
                    if (r->cells > MAX_CELLS) {
                        r->cells = MAX_CELLS;
                    }
                    uint8_t *search_ptr = ptr + BLOCK_HEADER_SIZE;
                    int available_bytes = 0;

                    while (search_ptr < frame + count_receive - 1) {
                        if (search_ptr[0] == 'E' && search_ptr[1] >= '0' && search_ptr[1] <= '8') {
                            break;
                        }
                        search_ptr++;
                        available_bytes++;
                    }

                    for (int bin = 0; bin < r->cells; bin++) {
                        int byte_index = bin * BYTES_PER_TRIPLET_DATA; /* 6 bytes por bin */

                        if (byte_index + (BYTES_PER_TRIPLET_DATA - 1) < available_bytes) {
                            r->vel_earth[bin].vx = (int16_t)le_to_u16(&data[byte_index]);     /* North (mm/s) */
                            r->vel_earth[bin].vy = (int16_t)le_to_u16(&data[byte_index + 2]); /* East (mm/s) */
                            r->vel_earth[bin].vz = (int16_t)le_to_u16(&data[byte_index + 4]); /* Down (mm/s) */

                        } else {
                            r->cells = bin; /* Truncated with valid bins */
                            break;
                        }
                    }
                    break;

                case '8': /* E8: Pressure */
                    if (length >= 4) {
                        uint16_t raw_pressure = le_to_u16(data);

                        r->pressure = (float)raw_pressure;
                    }
                    break;

                default:
                    break;
            }
            ptr += BLOCK_HEADER_SIZE + length;

        } else {
            ptr++;
        }
    }
    return 0;
}

/**
 * @brief  Convert raw FlowQuest ADCP data into processed ADCP measurements.
 */
int process_flowquest_raw_data(struct adcp_raw_data_flowquest *r, struct adcp_data *d)
{
    if (r->cells == 0) {
        DEBUG("\nWarning: No velocity data available (Cells = 0).\n");
    }

    d->cells = r->cells;
    d->beams = MAX_BEAMS;
    d->pressure = r->pressure / SCALE_FACTOR_1000; /* From 0.001psi to psi */
    d->temperature = r->temperature / SCALE_FACTOR_10;
    d->battery_voltage = r->battery_voltage / SCALE_FACTOR_10;
    d->heading = r->heading / SCALE_FACTOR_10;
    d->pitch = r->pitch / SCALE_FACTOR_100;
    d->roll = r->roll / SCALE_FACTOR_100;
    d->depth = (r->cells * (r->bin_length * SCALE_FACTOR_0_01)); /* cm to m */
    d->blanking = r->blank_distance / SCALE_FACTOR_100;

    DEBUG("bin_length: %u cm", r->bin_length);
    DEBUG("CELLS: %u,", r->cells);
    DEBUG("DEPTH: %.2f m", (double)d->depth);

    if (r->cells > 0) {
        for (int bin = 0; bin < r->cells && bin < MAX_CELLS; bin++) {
            d->vel_earth[bin].vx = (float)r->vel_earth[bin].vx / SCALE_FACTOR_10; /* North (cm/s) */
            d->vel_earth[bin].vy = (float)r->vel_earth[bin].vy / SCALE_FACTOR_10; /* East (cm/s) */
            d->vel_earth[bin].vz = (float)r->vel_earth[bin].vz / SCALE_FACTOR_10; /* Down (cm/s) */
            d->vel[bin] = speed(d->vel_earth[bin].vy, d->vel_earth[bin].vx);
            d->dir[bin] = direction(d->vel_earth[bin].vy, d->vel_earth[bin].vx);
        }
    }
    return 0;
}
