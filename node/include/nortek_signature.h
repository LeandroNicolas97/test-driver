/***************************************************************************
 *   file                 : nortek_signature.h                             *
 *   begin                : Sep 16, 2022                                   *
 *   copyright            : (C) 2022 by Innovex Tecnologias SpA.           *
 *   Email                : development@innovex.cl                         *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Spa. Chile.           *
 *   Copyright (C) 2022. Innovex.                                          *
 ***************************************************************************/

#ifndef NORTEK_SIGNATURE_H
#define NORTEK_SIGNATURE_H

/*
 * Constants and data structures for ADCP AQUAPRO NORTEK
 */

#include <stdint.h>
#include "adcp.h"

/* How many times we try to detect a sensor */
#define PD_CELL_SIZE 2.0 /**/

#define AQUADOPP_MAX_BEAMS 3
#define AQUADOPP_MAX_CELLS 70

#define HEAD_SIZE_AQUADOPP_PROFILER_VELOCITY_DATA 30
#define AQUADOPP_PROFILER_VELOCITY_DATA_SYNC      165
#define AQUADOPP_PROFILER_VELOCITY_DATA_ID        33

/**********************************************************
 * Definitions for ADCP Aquadopp Nortek
 **********************************************************/

/**
 * ADCP Aquadopp raw data structure.
 * Tis is more tha 1.5Kbytes long. Use with care in
 * embedded microcontrollers.
 */
struct adcp_raw_data {
    uint16_t heading;                  /* Compass heading (0.1 deg)  */
    int16_t pitch;                     /* Pitch (0.1 deg) */
    int16_t roll;                      /* Roll (0.1 deg) */
    int16_t temperature;               /* Temperature (0.01 deg C) */
    uint32_t pressure;                 /* Pressure 0.001dBar  */
    uint16_t cells;                    /* Number of cells */
    uint16_t beams;                    /* Number of beams */
    uint16_t cell_size;                /* Cell size in mm */
    uint16_t blanking;                 /* Blanking (cm or mm  depending on scaling */
    uint16_t battery_voltage;          /* Voltage 0.1V */
    uint16_t coordinates;              /* Coordinate system */
    int8_t velocity_scaling;           /* Velocity scaling */
    int16_t vel[MAX_BEAMS][MAX_CELLS]; /* Velocity ( 10^(velocity_scaling)m/s )*/
    uint8_t amp[MAX_BEAMS][MAX_CELLS]; /* Amplitude dB ( 0.5dB/count ) */
};

typedef struct {
    uint8_t cMinute; /* minute */
    uint8_t cSecond; /* second */
    uint8_t cDay;    /* day */
    uint8_t cHour;   /* hour */
    uint8_t cYear;   /* year */
    uint8_t cMonth;  /* month */
} PdClock;

typedef struct {
    uint8_t cSync;   /* sync = 0xa5 */
    uint8_t cId;     /* identification (0x21 = 3 beams, 0x22 = 2 beams, 0x21= 1 */
    uint16_t hSize;  /* size of structure (words) */
    PdClock clock;   /* date and time */
    uint16_t hError; /* error code */
    uint16_t hAnaIn1;
    uint16_t hBattery;                                    /* battery voltage (0.1 V) */
    uint16_t hSoundSpeed;                                 /* speed of sound (0.1 m/s) */
    int16_t hHeading;                                     /* compass heading (0.1 deg) */
    int16_t hPitch;                                       /* compass pitch (0.1 deg) */
    int16_t hRoll;                                        /* compass roll (0.1 deg) */
    uint8_t hPressureMSB;                                 /* pressure MSB */
    uint8_t cStatus;                                      /* status code */
    uint16_t hPressureLSW;                                /* pressure LSW */
    int16_t hTemperature;                                 /* temperature (0.01 deg C) */
    int16_t hVel[AQUADOPP_MAX_BEAMS][AQUADOPP_MAX_CELLS]; /* velocity */
    uint8_t cAmp[AQUADOPP_MAX_BEAMS][AQUADOPP_MAX_CELLS]; /* amplitude (counts) */
    unsigned char fill;
#if (AQUADOPP_MAX_CELLS * AQUADOPP_MAX_BEAMS) % 2 > 0
    uint16_t hChecksum; /* checksum */
#endif
} PdAqProf;

/**
 *Convert from BCD to char
 */
unsigned char BCDToChar(unsigned char cBCD);

/**
 * Parse the data coming from the Nortek ADCP. Take the frame and extract it
 * into a structure.
 * @param frame A pointer to the data frame
 * @param r A Pointer to the structure to store the parsed data.
 * @return negative on error.
 */
int parse_nortek_adcp_data_frame(uint8_t *frame, struct adcp_raw_data *r);

/**
 * Process an ADCP data frame and get normalized data
 * @param r A pointer to the raw data
 * @param d A pointer to the processed data
 * @return negative on error.
 */
int process_adcp_raw_data(struct adcp_raw_data *r, struct adcp_data *d);

int parse_aquadopp_data_frame(uint8_t *response, PdAqProf *aquadopp);

int process_aquadopp_raw_data(PdAqProf *aquadopp, struct adcp_data *d);
#endif /* NORTEK_SIGNATURE_H */
