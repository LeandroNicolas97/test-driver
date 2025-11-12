/**
 *  \file flowquest.h
 *  \brief Interface definitions for the FlowQuest sensor driver.
 *
 *  \author Raúl Baigorri Morales
 *  \author Sebastián Angulo Oyarzún
 *
 *  \date 18.02.2025
 *
 *  Copyright 2025 Innovex Tecnologías Ltda. All rights reserved.
 */
#ifndef FLOWQUEST_H
#define FLOWQUEST_H

#include <stdint.h>
#include "adcp.h"

#define FLOWQUEST_HEADER_SIZE         8
#define OFFSET_ENSEMBLE_NUMBER        2
#define OFFSET_TEMPERATURE            6
#define OFFSET_BATTERY_VOLTAGE        8
#define OFFSET_NUM_PINGS              10
#define OFFSET_HEADING                18
#define OFFSET_PITCH                  20
#define OFFSET_ROLL                   22
#define OFFSET_ENSEMBLE_OUTPUT_SWITCH 14
#define OFFSET_BIN_LENGTH             32
#define OFFSET_BLANK_DISTANCE         52
#define OFFSET_TRANSDUCER_DEPTH       54
#define OFFSET_ERROR_CODE             56
#define OFFSET_RPH_ABNORMAL           58
#define VALUES_PER_VELOCITY_CELL      3
#define BYTES_PER_TRIPLET_DATA        6
#define BLOCK_HEADER_SIZE             4
#define MIN_COMMON_DATA_SIZE          24
#define COMMON_HEADER_PAYLOAD_SIZE    64

struct adcp_raw_data_flowquest {
    uint16_t ensemble_number;
    uint32_t timestamp;
    uint16_t num_pings;
    uint16_t ensemble_output_switch;
    uint16_t bin_length;
    uint16_t transducer_depth;
    uint16_t blank_distance;
    uint16_t data_length;
    uint16_t cells;
    uint16_t temperature;
    uint16_t heading;
    uint16_t error_code;
    uint16_t rph_abnormal;
    int16_t pitch;
    int16_t roll;
    float battery_voltage;
    float pressure;
    struct velocity_data vel_inst[MAX_CELLS];
    struct velocity_data vel_earth[MAX_CELLS];
};

/**********************************************************
 * Definitions for ADCP FlowQuest
 **********************************************************/

int parse_flowquest_data_frame(int count_receive, uint8_t *frame, struct adcp_raw_data_flowquest *r);

int process_flowquest_raw_data(struct adcp_raw_data_flowquest *r, struct adcp_data *d);

int adcp_cell_count_get(void);

void adcp_cell_count_set(int count);

#endif
