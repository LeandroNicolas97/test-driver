/***************************************************************************
 *   file                 : parsing_signature_nortek.c                     *
 *   begin                : May 25, 2022                                   *
 *   copyright            : (C) 2022 by Innovex Tecnologias Ltda.          *
 *   Author               : Pablo Santamarina                              *
 *   Email                : development@innovex.cl                         *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2022. Innovex.                                          *
 ***************************************************************************/

/*
 * Parsing of the Nortek ADCP data
 */

#include <stdlib.h>
#include <math.h>
#include <zephyr/sys/printk.h>
#include "nortek_signature.h"
#include "watchdog.h"
#include "adcp.h"

#ifndef M_PI
/* TODO check why doesnt compile, pi should be in math.h */
#define M_PI 3.14159265358979323846
#endif

#define AQUADOPP_PROFILER_VELOCITY_DATA_SYNC 165
#define AQUADOPP_PROFILER_VELOCITY_DATA_ID   33

/**
 * Get a two byte word in Little-Endian into an unsigned integer.
 */
uint16_t le_to_u16(uint8_t *p)
{
    uint8_t lsb = *p;
    uint8_t msb = *(p + 1);

    return (msb << 8) | lsb;
}

/**
 * Get a four byte word in Little-Endian into an unsigned integer.
 */
uint32_t le_to_u32(uint8_t *p)
{
    uint32_t r = 0;

    p += 4;
    for (int i = 0; i < 4; i++) {
        p--;
        r <<= 8;
        r += *p;
    }
    return r;
}

/**
 * Parse the data coming from the Nortek ADCP. Take the frame and extract it
 * into a structure.
 * @param frame A pointer to the data frame
 * @param r A Pointer to the structure to store the parsed data.
 * @return negative on error.
 */
int parse_nortek_adcp_data_frame(uint8_t *frame, struct adcp_raw_data *r)
{
    uint8_t *header;      /* Start of header */
    uint8_t *common_data; /* Common data starts at this position */
    uint8_t *vel_data;    /* Velocity data starts at this position */

    header = frame;                          /* The first part of the frame is the header */
    common_data = frame + header[1];         /* Size of the header specified here */
    vel_data = common_data + common_data[1]; /* Offset of velocity data is here */

    /* The position of these data is specified in the data-sheet */
    r->temperature = le_to_u16(common_data + 18);
    r->pressure = le_to_u32(common_data + 20);
    r->heading = le_to_u16(common_data + 24);
    r->pitch = le_to_u16(common_data + 26);
    r->roll = le_to_u16(common_data + 28);
    r->cell_size = le_to_u16(common_data + 32);
    r->blanking = le_to_u16(common_data + 34);
    r->battery_voltage = le_to_u16(common_data + 38);
    r->velocity_scaling = (int8_t)common_data[58];

    /* These three fields are obtained from two bytes.  */
    uint16_t beams_coord_cells = le_to_u16(common_data + 30);

    r->cells = beams_coord_cells & 0x1FF;
    r->beams = (beams_coord_cells >> 12) % 0x0F;
    r->coordinates = (beams_coord_cells >> 10) & 0x03;

    /* Read the array with velocity data */
    for (int beam = 0; beam < r->beams; beam++) {
        if (beam >= MAX_BEAMS) {
            continue;
        }
        for (int cell = 0; cell < r->cells; cell++) {
            if (cell >= MAX_CELLS) {
                continue;
            }
            r->vel[beam][cell] = le_to_u16(vel_data + (beam * r->cells + cell) * sizeof(uint16_t));
        }
    }

    /* Read the array with Amplitude data. Amplitude
     * data is directly after velocity data
     */
    uint8_t *amplitude_data = (vel_data + (r->beams * r->cells * sizeof(uint16_t)));

    for (int beam = 0; beam < r->beams; beam++) {
        if (beam >= MAX_BEAMS) {
            continue;
        }
        for (int cell = 0; cell < r->cells; cell++) {
            if (cell >= MAX_CELLS) {
                continue;
            }
            r->amp[beam][cell] = *(amplitude_data + (beam * r->cells + cell));
        }
    }
    return 0;
}

/**
 * Square of an int16
 */
static float square16(int16_t a)
{
    float f = a * a;
    return f;
}

/**
 * Process an ADCP data frame and get normalised data
 * @param r A pointer to the raw data
 * @param d A pointer to the processed data
 * @return negative on error.
 */
int process_adcp_raw_data(struct adcp_raw_data *r, struct adcp_data *d)
{
    d->heading = (float)r->heading / 100;
    d->pitch = (float)r->pitch / 100;
    d->roll = (float)r->roll / 100;
    d->temperature = (float)r->temperature / 100;
    d->pressure = (float)r->pressure / 1000;
    d->battery_voltage = (float)r->battery_voltage / 10;
    d->blanking = (float)r->blanking / 100;
    d->cells = r->cells;
    d->beams = r->beams;
    d->first_cell = 3; /* TODO Set first cell fiexd to the blanking size of 3m */

    /*
     * Convert Vx,Vy into speed and direction for every cell.
     * Raw velocity is in mm/s, convert it to cm/s.
     * Raw velocity index are East = 0, North = 1, Up = 2
     */
    for (int i = 0; i < d->cells; i++) {
        d->vel[i] = 0.1 * sqrt(square16(r->vel[0][i]) + square16(r->vel[1][i]));
        float dir = atan2(r->vel[0][i], r->vel[1][i]) * 180 / M_PI;

        if (dir < 0) {
            dir += 360;
        }
        d->dir[i] = dir;
    }
    return 0;
}

int parse_aquadopp_data_frame(uint8_t *response, PdAqProf *aquadopp)
{
    int indice_vel = 30;
    int indice_anl = indice_vel + (AQUADOPP_MAX_BEAMS * AQUADOPP_MAX_CELLS * 2);

    aquadopp->cSync = response[0];
    aquadopp->cId = response[1];
    aquadopp->hSize = le_to_u16(&response[2]);
    aquadopp->clock.cMinute = BCDToChar(response[4]);
    aquadopp->clock.cSecond = BCDToChar(response[5]);
    aquadopp->clock.cDay = BCDToChar(response[6]);
    aquadopp->clock.cHour = BCDToChar(response[7]);
    aquadopp->clock.cYear = BCDToChar(response[8]);
    aquadopp->clock.cMonth = BCDToChar(response[9]);
    aquadopp->hError = le_to_u16(&response[10]);
    aquadopp->hAnaIn1 = le_to_u16(&response[12]);
    aquadopp->hBattery = le_to_u16(&response[14]);
    aquadopp->hSoundSpeed = le_to_u16(&response[16]);
    aquadopp->hHeading = le_to_u16(&response[18]);
    aquadopp->hPitch = le_to_u16(&response[20]);
    aquadopp->hRoll = le_to_u16(&response[22]);
    aquadopp->hPressureMSB = response[24];
    aquadopp->cStatus = response[25];
    aquadopp->hPressureLSW = le_to_u16(&response[26]);
    aquadopp->hTemperature = le_to_u16(&response[28]);
    watchdog_reset();
    for (int i = 0; i < AQUADOPP_MAX_BEAMS; i++) {
        for (int j = 0; j < AQUADOPP_MAX_CELLS; j++) {
            aquadopp->hVel[i][j] = le_to_u16(&response[indice_vel]); /* velocity */
            aquadopp->cAmp[i][j] = response[indice_anl];             /* amplitude (counts) */
            indice_vel = indice_vel + 2;
            indice_anl = indice_anl + 1;
        }
    }
    return 0;
}

int process_aquadopp_raw_data(PdAqProf *aquadopp, struct adcp_data *d)
{
    float pressure = (65536.0 * (double)(aquadopp->hPressureMSB) + (double)(aquadopp->hPressureLSW)) * 0.001;

    d->heading = (double)aquadopp->hHeading * 0.1;
    d->pitch = (double)aquadopp->hPitch * 0.1;
    d->roll = (double)aquadopp->hRoll * 0.1;
    d->temperature = (double)aquadopp->hTemperature * 0.01;
    d->pressure = pressure;
    d->battery_voltage = (double)aquadopp->hBattery * 0.1;
    d->blanking = 0;
    d->cells = AQUADOPP_MAX_CELLS;
    d->beams = AQUADOPP_MAX_BEAMS;
    d->first_cell = 0; /* TODO Set first cell fiexd to the blanking size of 3m */
    /*
     * Convert Vx,Vy into speed and direction for every cell.
     * Raw velocity is in mm/s, convert it to cm/s.
     * Raw velocity index are East = 0, North = 1, Up = 2
     */
    for (int i = 0; i < d->cells; i++) {
        d->vel[i] = 0.1 * sqrt((double)square16(aquadopp->hVel[0][i]) + (double)square16(aquadopp->hVel[1][i]) +
                               (double)square16(aquadopp->hVel[2][i]));
        float dir = atan2(aquadopp->hVel[0][i], aquadopp->hVel[1][i]) * 180 / M_PI;

        if (dir < 0) {
            dir += 360;
        }
        d->dir[i] = dir;
        printk("Celda: %i\n", i);
        printk("Velocidad: %.2f\n", (double)d->vel[i]);
        printk("Dir: %.2f\n", (double)d->dir[i]);
    }
    printk("Heading: %.1f\n", (double)d->heading);
    printk("Ptich: %.1f\n", (double)d->pitch);
    printk("Roll: %.1f\n", (double)d->roll);
    printk("Temperature: %.1f\n", (double)d->temperature);
    printk("Batery: %.1f\n", (double)d->battery_voltage);
    return 0;
}
