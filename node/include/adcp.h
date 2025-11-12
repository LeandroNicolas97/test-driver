#ifndef ADCP_h
#define ADCP_h
#define MAX_BEAMS         4
#define MAX_CELLS         129
#define SCALE_FACTOR_1000 1000
#define SCALE_FACTOR_100  100
#define SCALE_FACTOR_10   10
#define SCALE_FACTOR_0_01 0.01
#include <stdint.h>

struct velocity_data {
    int16_t vx;
    int16_t vy;
    int16_t vz;
};

/*
 * ADCP processed data structure. This is more than 1Kbyte long,
 * use with care in embedded microcontrollers.
 */
struct adcp_data {
    int cells;             /* Number of cells */
    int beams;             /* Number of beams */
    int first_cell;        /* First cell not in the blanking zone */
    int depth;             /* Measurement depth (meters) */
    float blanking;        /* Blanking distance in m*/
    float heading;         /* Compass heading (deg)  */
    float pitch;           /* Pitch (deg) */
    float roll;            /* Roll (deg) */
    float temperature;     /* Temperature (deg C) */
    float pressure;        /* Pressure dBar  */
    float cell_size;       /* Cell size in m */
    float battery_voltage; /* Voltage  */
    float vel[MAX_CELLS];  /* Velocity cm/s */
    float dir[MAX_CELLS];  /* Direction deg */
    struct velocity_data vel_earth[MAX_CELLS];
};

/* TODO find a better way to get this data (not global) */
extern struct adcp_data adcp_processed_data;

/**
 * Get a two byte word in Little-Endian into an unsigned integer.
 */
uint16_t le_to_u16(uint8_t *p);

/**
 * Get a four byte word in Little-Endian into an unsigned integer.
 */
uint32_t le_to_u32(uint8_t *p);
#endif /* ADCP_h */
