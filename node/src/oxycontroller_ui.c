/**
 *  \file oxycontroller_ui.c
 *  \brief User interface for concentrator
 *
 *  \author Pablo Santamarina Cuneo
 *
 *  Copyright 2016 Innovex Tecnologias Ltda. All rights reserved.
 */
#include <string.h>
#include <stdlib.h>
#include "bsp-config.h"
#include "configuration.h"
#include "microio.h"
#include "microtime.h"
#include "led.h"
#include "display_fb.h"
#include "font.h"
#include "debug.h"
#include "oxygen_saturation.h"
#include "measurement.h"
#include "measurement_operations.h"
#include "smart_sensor.h"
#include "errorcodes.h"
#include "version.h"
#include "userinterface.h"
#include "hardware.h"
#include "radio.h"
#include "actual_conditions.h"

#define DEPRECATED 0

/* The frame buffer for the screen */
uint8_t frame_buffer[DISPLAY_ROWS * DISPLAY_STRIDE];

/*
 * Local prototypes
 */
static void display_failure_text(enum sensor_status status);
static void display_visual_measurement_with_big_font(int n, enum measurement_status status,
                                                     struct visual_measurement *vm);
static void display_visual_measurement_with_medium_font(int sensor_nr, int n, enum measurement_status status,
                                                        struct visual_measurement *vm);
static void display_visual_measurement_with_small_font(int sensor_number, int n, enum measurement_status status,
                                                       struct visual_measurement *vm);
static void display_visual_measurement_with_small_font_two_columns(int sensor_number, int n,
                                                                   enum measurement_status status,
                                                                   struct visual_measurement *vm);
static void display_visual_measurement_current_small(int n, enum measurement_status status,
                                                     struct visual_measurement *vm);
static void display_visual_measurement_current_profiler_small(int n, enum measurement_status status,
                                                              struct visual_measurement *vm);
static void display_visual_measurement_current_ac_medium(int n, enum measurement_status status,
                                                         struct visual_measurement *vm);
static int convert_measurement_to_visual(uint8_t n_sensors, struct measurement *measurement,
                                         struct visual_measurement *vm, enum measurement_status *status,
                                         uint8_t use_sat);

/*
 * Fonts used in this project
 */
extern const struct font_description font_innovex_simple_8;
extern const struct font_description font_arial_narrow_bold_16;
extern const struct font_description font_arial_narrow_bold_24;
extern const struct font_description font_arial_narrow_bold_40;

enum {
    FONT_8_PIXEL,
    FONT_16_PIXEL,
    FONT_24_PIXEL,
    FONT_40_PIXEL
};

const struct font_description *fonts[] = {
    [FONT_8_PIXEL] = &font_innovex_simple_8,
    [FONT_16_PIXEL] = &font_arial_narrow_bold_16,
    [FONT_24_PIXEL] = &font_arial_narrow_bold_24,
    [FONT_40_PIXEL] = &font_arial_narrow_bold_40,
};

/**
 * Initialize and clear the LCD
 */
void init_and_clear_lcd(void)
{
    display_driver_init();
    display_init(frame_buffer);
    display_driver_set_contrast(cfg.lcd_contrast);
    display_clear();
}
/**
 * Display which memory will be in use
 */
void display_which_memory(void)
{
    if (actual_state.using_external_memory == 1) {
        display_printf("Using external memory\n");
    } else {
        display_printf("Using internal memory\n");
    }
}
void display_welcome_message(void)
{
    struct mac_address mac;
    char mac_string[48];

    display_clear();
    display_printf("Innovex Tecnologias\n");
    display_printf("Multitransmitter\n");
    display_printf("Version\n%s\n", VERSION_STRING);
    display_printf("%s\n", MICROLIB_VERSION_STRING);
    get_mac_address(&mac);
    mac_address_to_string(&mac.dev_id, mac.length, mac_string);
    display_printf("%s\n", mac_string);
    display_which_memory();
    display_flush();
    /*    lcd_printf("%.2x %.2x %.2x %.2x\n", (mac_address >> 48) & 0xFFFF, (mac_address >> 32) & 0xFFFF, (mac_address
     */
    /*    >> 16) & 0xFFFF,  mac_address & 0xFFFF ); */
}

void display_end_device_status(float battery_voltage)
{
    if (actual_state.coordinator_found) {
        display_printf("Ch: %i Mis: %i ", cfg.channel, actual_state.missed_conection);
        display_printf("Signal: %i%%", end_device_get_link_quality());
        actual_state.missed_conection = 0;
    } else {
        display_printf("Not associated\n");
    }
    display_move(0, 7 * 8);
    display_printf("Name: %s", cfg.name);
    display_move(38, 7 * 8);
    display_printf("Bat: %.2fV\n", (double)battery_voltage);
    display_move(86, 7 * 8);
    display_printf("D: %i", cfg.distance);
}

void display_failure_text(enum sensor_status status)
{
    DEBUG("Error: %s\n", sensor_status_to_string(status));
    display_printf("Error: %s\n", sensor_status_to_string(status));
}

void display_going_to_sleep(int seconds)
{
    display_clear();
    display_move(10, 10);
    display_set_font(FONT_16_PIXEL);
    display_printf("Sleeping in");
    display_move(10, 30);
    display_printf("%i seconds\n", seconds);
    display_flush();
}

void display_all_measurements(int n_of_sensors_active, struct measurement *measurement, uint8_t use_sat)
{
    DEBUG("Displaying all measurements..\n");
    struct measurement *measurement_ptr;

    display_move(0, 9);
    for (int i = 0; i < n_of_sensors_active; i++) {
        measurement_ptr = measurement + i;
        /* XXX If we have a salinity and an oxygen sensor we only display a single measurement */
        /* TODO Solve this in measurement_operations.c */
        /* if ((measurement_ptr->oxygen.salinity_status) == MEASUREMENT_OK && n_of_sensors_active <= 2) */
        /* n_of_sensors_active = 1; */
        /** XXX End */
        if (measurement_ptr->sensor_status == SENSOR_OK) {
            struct visual_measurement vm[9];
            enum measurement_status status;

            DEBUG("Sensor type: %i\n", measurement_ptr->type);
            int n = convert_measurement_to_visual(n_of_sensors_active, measurement_ptr, &(vm[0]), &status, use_sat);

            if (n == 0) {
                /* No conversion performed, sensor not supported */
                display_failure_text(SENSOR_NOT_SUPPORTED);
            } else if (n_of_sensors_active == 1) {
                if (measurement_ptr->type == CURRENT_AC_SENSOR) {
                    display_visual_measurement_current_ac_medium(n, status, vm);
                } else if (measurement_ptr->type == CURRENT_SENSOR) {
                    display_visual_measurement_current_small(n, status, vm);
                } else if (measurement_ptr->type == CURRENT_PROFILER_SENSOR) {
                    display_visual_measurement_current_profiler_small(n, status, vm);
                } else {
                    display_visual_measurement_with_big_font(n, status, vm);
                }
            } else if (n_of_sensors_active >= 2 && n_of_sensors_active <= 3) {
                display_visual_measurement_with_medium_font(i + 1, n, status, vm);
            } else if (n_of_sensors_active > 3 && n_of_sensors_active <= 6) {
                display_visual_measurement_with_small_font(i + 1, n, status, vm);
            } else if (n_of_sensors_active > 6) {
                display_visual_measurement_with_small_font_two_columns(i + 1, n, status, vm);
            }
        } else {
            DEBUG("--- Display s_status: %i\n", measurement_ptr->sensor_status);
            display_failure_text(measurement_ptr->sensor_status);
        }
    }
}

void display_associated_header(int channel, int missed_connections, int link_quality, float bat_lvl)
{
    ARG_UNUSED(channel);
    ARG_UNUSED(missed_connections);
    ARG_UNUSED(link_quality);

    display_set_font(FONT_8_PIXEL);
    display_printf("Bat: %.3fV\n", (double)bat_lvl);
    /* display_printf("Ch: %i Mis: %i ", channel, missed_connections); */
    /* display_printf("Link: %i\n", link_quality); */
}

void display_not_associated_header(void)
{
    display_set_font(FONT_8_PIXEL);
    display_printf("Not associated\n");
}

void display_footer(char *name, uint16_t channel)
{
    display_move(0, 7 * 8);
    display_set_font(FONT_8_PIXEL);
    display_printf("Name:%s  ", name);
    /*    display_printf("%s %li", name, get_time()); */
    display_move(54, 7 * 8);
    display_printf("Channel: %i ", channel);
}

static int oxygen_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->oxygen.concentration;
    vm[0].status = measurement->oxygen.concentration_status;
    vm[0].units = "mg/l";
    vm[0].format = "%.1f";

    vm[1].value = measurement->oxygen.temperature;
    vm[1].status = measurement->oxygen.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    vm[2].value = measurement->oxygen.saturation;
    vm[2].status = measurement->oxygen.saturation_status;
    vm[2].units = "%";
    vm[2].format = "%.0f";

    vm[3].value = measurement->oxygen.salinity;
    vm[3].status = measurement->oxygen.salinity_status;
    vm[3].units = "psu";
    vm[3].format = "Salin: %.1f";

    return 4; /* 4 variables to display */
}

static int saturation_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->oxygen.saturation;
    vm[0].status = measurement->oxygen.saturation_status;
    vm[0].units = "%%";
    vm[0].format = "%.0f";

    vm[1].value = measurement->oxygen.temperature;
    vm[1].status = measurement->oxygen.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    vm[2].value = measurement->oxygen.concentration;
    vm[2].status = measurement->oxygen.concentration_status;
    vm[2].units = "mg/l";
    vm[2].format = "%.1f";

    vm[3].value = measurement->oxygen.salinity;
    vm[3].status = measurement->oxygen.salinity_status;
    vm[3].units = "psu";
    vm[3].format = "Salin: %.1f";

    return 4; /* 4 variables to display */
}

static int oxygen_ponsel_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->oxygen.concentration;
    vm[0].status = measurement->oxygen.concentration_status;
    vm[0].units = "mg/l";
    vm[0].format = "%.1f";

    vm[1].value = measurement->oxygen.saturation;
    vm[1].status = measurement->oxygen.saturation_status;
    vm[1].units = "%";
    vm[1].format = "%.0f";

    vm[2].value = measurement->oxygen.temperature;
    vm[2].status = measurement->oxygen.temperature_status;
    vm[2].units = "\260C";
    vm[2].format = "%.1f";

    vm[3].value = measurement->oxygen.salinity;
    vm[3].status = measurement->oxygen.salinity_status;
    vm[3].units = "psu";
    vm[3].format = "Salin: %.1f";

    return 4; /* 4 variables to display */
}

static int ph_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->pH.pH;
    vm[0].status = measurement->pH.pH_status;
    vm[0].units = "pH";
    vm[0].format = "%.1f";

    return 1;
}

static int ph_measurements_to_visual_big(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->pH.pH;
    vm[0].status = measurement->pH.pH_status;
    vm[0].units = "pH";
    vm[0].format = "%.1f";

    vm[1].value = measurement->pH.temperature;
    vm[1].status = measurement->pH.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int salinity_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->conductivity.salinity;
    vm[0].status = measurement->conductivity.salinity_status;
    vm[0].units = "g/l";
    vm[0].format = "%.1f";

    vm[1].value = measurement->conductivity.temperature;
    vm[1].status = measurement->conductivity.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int conductivity_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->conductivity.conductivity; /* x100 to get uS/cm */
    vm[0].status = measurement->conductivity.conductivity_status;
    if (cfg.conductivity_freshwater == SEAWATER) {
        vm[0].units = "mS/cm";
        vm[0].format = "%.2f";
    } else if (cfg.conductivity_freshwater == FRESHWATER) {
        vm[0].units = "uS/cm";
        vm[0].format = "%.2f";
    }
/* yosemitech's sensors use this. And we dont want to show the temp with this */
/* sensors becouse of temp differences between them. */
#if DEPRECATED
    vm[1].value = measurement->conductivity.temperature;
    vm[1].status = measurement->conductivity.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
#endif
    return 1;
}

static int pressure_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
#if CONFIG_EXTERNAL_DATALOGGER
    float distance = ((measurement->pressure.pressure - 101320) / 98) / 100;

    vm[0].value = distance;
    vm[0].status = measurement->pressure.pressure_status;
    vm[0].units = "m";
    vm[0].format = "%.2f";
#else
    vm[0].value = measurement->pressure.pressure;
    vm[0].status = measurement->pressure.pressure_status;
    if (pass_pressure_unit() == KPA) {
        vm[0].units = "kPa";
        vm[0].format = "%.0f";
    } else if (pass_pressure_unit() == BAR) {
        vm[0].units = "bar";
        vm[0].format = "%.1f";
    }
#endif
    vm[1].value = measurement->pressure.temperature;
    vm[1].status = measurement->pressure.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int wave_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->wave.height;
    vm[0].status = measurement->wave.height_status;
    vm[0].units = "m";
    vm[0].format = "%.2f";

    vm[1].value = measurement->wave.temperature;
    vm[1].status = measurement->wave.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int radiation_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->radiation.radiation;
    vm[0].status = measurement->radiation.radiation_status;
    vm[0].units = "W/m2";
    vm[0].format = "%.2f";

    return 1; /* 1 variables to display */
}

static int radiation_uv_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->radiation_uv.energy_flow;
    vm[0].status = measurement->radiation_uv.energy_flow_status;
    vm[0].units = "W/m2";
    vm[0].format = "%.2f";

    return 1; /* 1 variables to display */
}

static int chlorophyll_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->chlorophyll.chlorophyll;
    vm[0].status = measurement->chlorophyll.chlorophyll_status;
    vm[0].units = "ug/l";
    vm[0].format = "%.1f";

    vm[1].value = measurement->chlorophyll.temperature;
    vm[1].status = measurement->chlorophyll.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int turbidity_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->turbidity.turbidity;
    vm[0].status = measurement->turbidity.turbidity_status;
    vm[0].units = "NTU";
    vm[0].format = "%.0f";

    vm[1].value = measurement->turbidity.temperature;
    vm[1].status = measurement->turbidity.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "T: %.1f";

    return 2; /* 2 variables to display */
}

static int suspended_solids_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->suspended_solids.suspended_solids;
    vm[0].status = measurement->suspended_solids.suspended_solids_status;
    vm[0].units = "mg/L";
    vm[0].format = "%.0f";

    vm[1].value = measurement->suspended_solids.temperature;
    vm[1].status = measurement->suspended_solids.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int water_potencial_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->water_potencial.water_potencial;
    vm[0].status = measurement->water_potencial.water_potencial_status;
    vm[0].units = "kPa";
    vm[0].format = "%.0f";

    vm[1].value = measurement->water_potencial.temperature;
    vm[1].status = measurement->water_potencial.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int distance_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->distance.mean_distance;
    vm[0].status = measurement->distance.mean_distance_status;
    vm[0].units = "cm";
    vm[0].format = "%.1f";

    return 1; /* 1 variables to display */
}

static int rain_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->rain.rain;
    vm[0].status = measurement->rain.rain_status;
    vm[0].units = "mm";
    vm[0].format = "%.2f";

    return 1; /* 1 variables to display */
}

static int watering_rate_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->watering_rate.watering_rate;
    vm[0].status = measurement->watering_rate.watering_rate_status;
    vm[0].units = "ml";
    vm[0].format = "%.1f";

    return 1; /* 1 variables to display */
}

static int phreatic_level_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
#if CONFIG_EXTERNAL_DATALOGGER
    vm[0].value = measurement->phreatic_level.phreatic_level;
    vm[0].status = measurement->phreatic_level.phreatic_level_status;
    vm[0].units = "m";
    vm[0].format = "%.1f";
#else
    vm[0].value = measurement->phreatic_level.phreatic_level;
    vm[0].status = measurement->phreatic_level.phreatic_level_status;
    if (pass_phreatic_unit() == METER) {
        vm[0].units = "m";
    } else if (pass_phreatic_unit() == CENTMETER) {
        vm[0].units = "cm";
    }
    vm[0].format = "%.1f";
#endif
    vm[1].value = measurement->phreatic_level.pressure;
    vm[1].status = measurement->phreatic_level.pressure_status;
    if (pass_pressure_unit() == KPA) {
        vm[1].units = "kPa";
    } else if (pass_pressure_unit() == BAR) {
        vm[1].units = "bar";
    }
    vm[1].format = "%.1f";

    vm[2].value = measurement->phreatic_level.temperature;
    vm[2].status = measurement->phreatic_level.temperature_status;
    vm[2].units = "\260C";
    vm[2].format = "%.1f";

    return 3; /* 3 variables to display */
}

static int line_pressure_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->line_pressure.line_pressure;
    vm[0].status = measurement->line_pressure.line_pressure_status;
    vm[0].units = "kPa";
    vm[0].format = "%.0f";

    vm[1].value = measurement->line_pressure.temperature;
    vm[1].status = measurement->line_pressure.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int co2_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->co2.co2;
    vm[0].status = measurement->co2.co2_status;
    vm[0].units = "%%";
    vm[0].format = "%.2f";

    return 1; /* 2 variables to display */
}

static int h2s_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->h2s.h2s;
    vm[0].status = measurement->h2s.h2s_status;
    vm[0].units = "ppm";
    vm[0].format = "%.2f";

    vm[1].value = measurement->h2s.temperature;
    vm[1].status = measurement->h2s.temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.0f";

    vm[2].value = (float)measurement->h2s.humidity;
    vm[2].status = MEASUREMENT_OK; /* there isn't a status for humidity */
    vm[2].units = "%";
    vm[2].format = "%.0f";

    return 3; /* 2 variables to display */
}

static int chelsea_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->chelsea.chlorophyll;
    vm[0].status = measurement->chelsea.chlorophyll_status;
    vm[0].units = "ug/l";
    vm[0].format = "%.1f";

    vm[1].value = measurement->chelsea.turbidity;
    vm[1].status = measurement->chelsea.turbidity_status;
    vm[1].units = "FTU";
    vm[1].format = "%.0f";

    vm[2].value = measurement->chelsea.phycocyanin;
    vm[2].status = measurement->chelsea.phycocyanin_status;
    vm[2].units = "";
    vm[2].format = "%.1f";

    return 3; /* 3 variables to display */
}

static int level_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->level.level_1;
    vm[0].status = measurement->level.level_1_status;
    vm[0].units = "cm";
    vm[0].format = "%.1f";

    vm[1].value = measurement->level.level_2;
    vm[1].status = measurement->level.level_2_status;
    vm[1].units = "cm";
    vm[1].format = "%.1f";

    return 2; /* 2 variables to display */
}

static int flow_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->flow.speed;
    vm[0].status = measurement->flow.speed_status;
    vm[0].units = "cm/s";
    vm[0].format = "%.0f";

    vm[1].value = measurement->flow.direction;
    vm[1].status = measurement->flow.direction_status;
    if ((measurement->flow.direction > 337.5f) || (measurement->flow.direction < 22.5f)) {
        vm[1].units = "N";
    } else if ((measurement->flow.direction > 22.5f) && (measurement->flow.direction < 67.5f)) {
        vm[1].units = "NE";
    } else if ((measurement->flow.direction > 67.5f) && (measurement->flow.direction < 112.5f)) {
        vm[1].units = "E";
    } else if ((measurement->flow.direction > 112.5f) && (measurement->flow.direction < 157.5f)) {
        vm[1].units = "SE";
    } else if ((measurement->flow.direction > 157.5f) && (measurement->flow.direction < 202.5f)) {
        vm[1].units = "S";
    } else if ((measurement->flow.direction > 202.5f) && (measurement->flow.direction < 257.5f)) {
        vm[1].units = "SO";
    } else if ((measurement->flow.direction > 257.5f) && (measurement->flow.direction < 292.5f)) {
        vm[1].units = "O";
    } else if ((measurement->flow.direction > 292.5f) && (measurement->flow.direction < 337.5f)) {
        vm[1].units = "NO";
    }
    vm[1].format = "%.0f";
    return 2; /* 2 variables to display */
}

static int current_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->current.current_1;
    vm[0].status = measurement->current.current_1_status;
    vm[0].units = "A";
    vm[0].format = "L1:%.1f";

    vm[1].value = measurement->current.current_2;
    vm[1].status = measurement->current.current_2_status;
    vm[1].units = "A";
    vm[1].format = "L2:%.1f";

    vm[2].value = measurement->current.current_3;
    vm[2].status = measurement->current.current_3_status;
    vm[2].units = "A";
    vm[2].format = "L3:%.1f";

    vm[3].value = measurement->current.current_4;
    vm[3].status = measurement->current.current_4_status;
    vm[3].units = "A";
    vm[3].format = "L4:%.1f";

    vm[4].value = measurement->current.current_5;
    vm[4].status = measurement->current.current_5_status;
    vm[4].units = "A";
    vm[4].format = "L5:%.1f";

    vm[5].value = measurement->current.current_6;
    vm[5].status = measurement->current.current_6_status;
    vm[5].units = "A";
    vm[5].format = "L6:%.1f";

    vm[6].value = measurement->current.current_7;
    vm[6].status = measurement->current.current_7_status;
    vm[6].units = "A";
    vm[6].format = "L7:%.1f";

    vm[7].value = measurement->current.current_8;
    vm[7].status = measurement->current.current_8_status;
    vm[7].units = "A";
    vm[7].format = "L8:%.1f";

    vm[8].value = measurement->current.temperature;
    vm[8].status = measurement->current.temperature_status;
    vm[8].units = "\260C";
    vm[8].format = "Temp: %.1f";

    return 9; /* 8 variables to display */
}

static int flow_water_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->flow_water.flow_water;
    vm[0].status = measurement->flow_water.flow_water_status;
    vm[0].units = "L/s";
    vm[0].format = "%.1f";

    vm[1].value = measurement->flow_water.frequency;
    vm[1].status = measurement->flow_water.frequency_status;
    vm[1].units = "m/s";
    vm[1].format = "%.1f";

    vm[2].value = measurement->flow_water.distance;
    vm[2].status = measurement->flow_water.distance_status;
    vm[2].units = "m";
    vm[2].format = "%.1f";

    vm[3].value = measurement->flow_water.accumulated;
    vm[3].status = measurement->flow_water.accumulated_status;
    vm[3].units = "m3";
    vm[3].format = "%.0f";
    return 4; /* 1 variables to display */
}

static int temperature_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->temperature.temperature;
    vm[0].status = measurement->temperature.temperature_status;
    vm[0].units = "\260C";
    vm[0].format = "%.1f";

    vm[1].value = measurement->temperature.depth;
    vm[1].status = measurement->temperature.depth_status;
    vm[1].units = "m";
    vm[1].format = "%.0f";

    return 2; /* 2 variables to display */
}

static int current_ac_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->current_ac.phase_1;
    vm[0].status = measurement->current_ac.phase_1_status;
    vm[0].units = "A";
    vm[0].format = "P1:%.1f";

    vm[1].value = measurement->current_ac.phase_2;
    vm[1].status = measurement->current_ac.phase_2_status;
    vm[1].units = "A";
    vm[1].format = "P2:%.1f";

    vm[2].value = measurement->current_ac.phase_3;
    vm[2].status = measurement->current_ac.phase_3_status;
    vm[2].units = "A";
    vm[2].format = "P3:%.1f";

    vm[3].value = measurement->current_ac.temperature;
    vm[3].status = measurement->current_ac.temperature_status;
    vm[3].units = "\260C";
    vm[3].format = "T: %.1f";

    return 4; /* 4 variables to display */
}

static int flow_ultrasonic_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{

    vm[0].value = measurement->flow_ultrasonic.rate;
    vm[0].status = measurement->flow_ultrasonic.rate_status;
    vm[0].units = "l/s";
    vm[0].format = "%.1f";

    vm[1].value = measurement->flow_ultrasonic.speed;
    vm[1].status = measurement->flow_ultrasonic.speed_status;
    vm[1].units = "m/s";
    vm[1].format = "%.1f";

    /* dummy measurement to show accumulated in the 4th slot */
    vm[2].value = measurement->flow_ultrasonic.depth;
    vm[2].status = measurement->flow_ultrasonic.depth_status;
    vm[2].units = "m";
    vm[2].format = "%.1f";

    vm[3].value = measurement->flow_ultrasonic.totalizer;
    vm[3].status = measurement->flow_ultrasonic.totalizer_status;
    vm[3].units = "m3";
    vm[3].format = "%.0f";

    return 4; /* 4 variable to display */
}

static int weather_station_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->weather_station.average_wind;
    vm[0].status = measurement->weather_station.average_wind_status;
    vm[0].units = "kn";
    vm[0].format = "%.1f";

    vm[1].value = measurement->weather_station.air_temperature;
    vm[1].status = measurement->weather_station.air_temperature_status;
    vm[1].units = "\260C";
    vm[1].format = "%.1f";

    vm[2].value = measurement->weather_station.relative_humidity;
    vm[2].status = measurement->weather_station.relative_humidity_status;
    vm[2].units = "%";
    vm[2].format = "%.0f";

    vm[3].value = measurement->weather_station.wind_gusts;
    vm[3].status = measurement->weather_station.wind_gusts_status;
    vm[3].units = "kn";
    vm[3].format = "Raf: %.0f";

    return 4; /* 4 variables to display */
}

static int wind_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->wind.average_wind;
    vm[0].status = measurement->wind.average_wind_status;
    vm[0].units = "kn";
    vm[0].format = "%.1f";

    vm[1].value = measurement->wind.average_direction;
    vm[1].status = measurement->wind.average_direction_status;
    vm[1].units = "\260";
    vm[1].format = "%.1f";

    vm[2].value = measurement->wind.wind_gusts;
    vm[2].status = measurement->wind.wind_gusts_status;
    vm[2].units = "kn";
    vm[2].format = "Raf: %.1f";

    return 3; /* 4 variables to display */
}

static int ysi_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->ctdo.conductivity;
    vm[0].status = measurement->ctdo.conductivity_status;
    vm[0].units = "uS/cm";
    vm[0].format = "%.2f";

    vm[1].value = measurement->ctdo.saturation;
    vm[1].status = measurement->ctdo.saturation_status;
    vm[1].units = "%";
    vm[1].format = "%.1f";

    vm[2].value = measurement->ctdo.temperature;
    vm[2].status = measurement->ctdo.temperature_status;
    vm[2].units = "C";
    vm[2].format = "%.1f";

    return 3;
}

static int gps_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->gps.Latitude;
    vm[0].status = measurement->gps.gps_status;
    vm[0].units = "o";
    vm[0].format = "%.2f";

    vm[1].value = measurement->gps.Longitude;
    vm[1].status = measurement->gps.gps_status;
    vm[1].units = "o";
    vm[1].format = "%.2f";

    return 2; /* 2 variables to display */
}

static int oxygen_measurements_to_visual_seabird(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->oxygen.concentration;
    vm[0].status = measurement->oxygen.concentration_status;
    vm[0].units = "mg/l";
    vm[0].format = "O: %.1f";

    vm[1].value = measurement->oxygen.salinity;
    vm[1].status = measurement->oxygen.salinity_status;
    vm[1].units = "psu";
    vm[1].format = "S: %.2f";

    return 2; /* 2 variables to display */
}

static int pressure_measurements_to_visual_seabird(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->pressure.pressure;
    vm[0].status = measurement->pressure.pressure_status;
    vm[0].units = "dbar";
    vm[0].format = "Press: %.3f";

    return 1; /* 1 variables to display */
}

static int chlorophyll_measurements_to_visual_seabird(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->chlorophyll.chlorophyll;
    vm[0].status = measurement->chlorophyll.chlorophyll_status;
    vm[0].units = "ug/l";
    vm[0].format = "Chlo: %.3f";

    return 1; /* 1 variables to display */
}

static int ctdo_measurements_to_visual_seabird(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->ctdo.conductivity;
    vm[0].status = measurement->ctdo.conductivity_status;
    vm[0].units = "mS/cm";
    vm[0].format = "C: %.2f";

    vm[1].value = measurement->ctdo.saturation;
    vm[1].status = measurement->ctdo.saturation_status;
    vm[1].units = "%";
    vm[1].format = "%.1f";

    return 2;
}

static int volume_measurements_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->volume.volume;
    vm[0].status = measurement->volume.volume_status;
    vm[0].units = "m3";
    vm[0].format = "%.1f";

    vm[1].value = measurement->volume.porcentage;
    vm[1].status = measurement->volume.porcentage_status;
    vm[1].units = "%%";
    vm[1].format = "%.0f";

    vm[2].value = measurement->volume.distance;
    vm[2].status = measurement->volume.distance_status;
    vm[2].units = "cm";
    vm[2].format = "%.1f";

    return 3; /* 3 variables to display */
}

static int current_profiler_measurement_to_visual(struct measurement *measurement, struct visual_measurement *vm)
{
    vm[0].value = measurement->current_profiler_signature.speed;
    vm[0].status = measurement->current_profiler_signature.current_profiler_signature_status;
    vm[0].units = "cm/s";
    vm[0].format = "Vel: %.1f";

    vm[1].value = measurement->current_profiler_signature.direction;
    vm[1].status = measurement->current_profiler_signature.current_profiler_signature_status;
    vm[1].units = "cm/s";
    vm[1].format = "Dir: %.1f";

    vm[2].value = measurement->current_profiler_signature.Temperature;
    vm[2].status = measurement->current_profiler_signature.current_profiler_signature_status;
    vm[2].units = "\260C";
    vm[2].format = "T: %.1f";

    return 3;
}

/**
 * Convert a measurement to a visual measurement.
 * @param measurement a Pointer to the measurement
 * @param vm a pointer to a visual measurement
 * @param status A pointer to receive the status of the measurement
 * @return The number of parameters to display
 */
static int convert_measurement_to_visual(uint8_t n_sensors, struct measurement *measurement,
                                         struct visual_measurement *vm, enum measurement_status *status,
                                         uint8_t use_sat)
{
    struct smart_sensor s = *smart_sensor_get(n_sensors - 1);
    int n = 0;
    *status = MEASUREMENT_OK;
    if (measurement->type == OXYGEN_SENSOR) {
        if (s.manufacturer == SEABIRD) {
            n = oxygen_measurements_to_visual_seabird(measurement, vm);
        } else if (s.manufacturer == PONSEL) {
            n = oxygen_ponsel_measurements_to_visual(measurement, vm);
        } else {
            if (use_sat == 1) {
                n = saturation_measurements_to_visual(measurement, vm);
            } else {
                n = oxygen_measurements_to_visual(measurement, vm);
            }
        }
        *status = measurement->oxygen.saturation_status;
    } else if (measurement->type == PH_SENSOR) {
        if (n_sensors == 1) {
            n = ph_measurements_to_visual_big(measurement, vm);
        } else if (n_sensors > 1) {
            n = ph_measurements_to_visual(measurement, vm);
        }
        *status = measurement->pH.pH_status;
    } else if (measurement->type == CONDUCTIVITY_SENSOR) {
        if (measurement->conductivity.conductivity_status == MEASUREMENT_OK) {
            n = conductivity_measurements_to_visual(measurement, vm);
            *status = measurement->conductivity.conductivity_status;
        } else {
            n = salinity_measurements_to_visual(measurement, vm);
            *status = measurement->conductivity.salinity_status;
        }
    } else if (measurement->type == PRESSURE_SENSOR) {
        if (s.manufacturer == SEABIRD) {
            n = pressure_measurements_to_visual_seabird(measurement, vm);
        } else {
            n = pressure_measurements_to_visual(measurement, vm);
        }
        *status = measurement->pressure.pressure_status;
    } else if (measurement->type == WAVE_SENSOR) {
        n = wave_measurements_to_visual(measurement, vm);
        *status = measurement->wave.height_status;
    } else if (measurement->type == RADIATION_SENSOR) {
        n = radiation_measurements_to_visual(measurement, vm);
        *status = measurement->radiation.radiation_status;
    } else if (measurement->type == RADIATION_UV_SENSOR) {
        n = radiation_uv_measurements_to_visual(measurement, vm);
        *status = measurement->radiation_uv.energy_flow_status;
    } else if (measurement->type == CHLOROPHYLL_SENSOR) {
        if (s.manufacturer == SEABIRD) {
            n = chlorophyll_measurements_to_visual_seabird(measurement, vm);
        } else {
            n = chlorophyll_measurements_to_visual(measurement, vm);
        }
        *status = measurement->chlorophyll.chlorophyll_status;
    } else if (measurement->type == TURBIDITY_SENSOR) {
        n = turbidity_measurements_to_visual(measurement, vm);
        *status = measurement->turbidity.turbidity_status;
    } else if (measurement->type == SUSPENDED_SOLIDS_SENSOR) {
        n = suspended_solids_measurements_to_visual(measurement, vm);
        *status = measurement->suspended_solids.suspended_solids_status;
    } else if (measurement->type == WATER_POTENCIAL_SENSOR) {
        n = water_potencial_measurements_to_visual(measurement, vm);
        *status = measurement->water_potencial.water_potencial_status;
    } else if (measurement->type == DISTANCE_SENSOR) {
        n = distance_measurements_to_visual(measurement, vm);
        *status = measurement->distance.mean_distance_status;
    } else if (measurement->type == RAIN_SENSOR) {
        n = rain_measurements_to_visual(measurement, vm);
        *status = measurement->rain.rain_status;
    } else if (measurement->type == WATERING_RATE_SENSOR) {
        n = watering_rate_measurements_to_visual(measurement, vm);
        *status = measurement->watering_rate.watering_rate_status;
    } else if (measurement->type == PHREATIC_LEVEL_SENSOR) {
        n = phreatic_level_measurements_to_visual(measurement, vm);
        *status = measurement->phreatic_level.phreatic_level_status;
    } else if (measurement->type == LINE_PRESSURE_SENSOR) {
        n = line_pressure_measurements_to_visual(measurement, vm);
        *status = measurement->line_pressure.line_pressure_status;
    } else if (measurement->type == CO2_SENSOR) {
        n = co2_measurements_to_visual(measurement, vm);
        *status = measurement->co2.co2_status;
    } else if (measurement->type == CHELSEA_SENSOR) {
        n = chelsea_measurements_to_visual(measurement, vm);
        *status = measurement->chelsea.chlorophyll_status;
    } else if (measurement->type == H2S_SENSOR) {
        n = h2s_measurements_to_visual(measurement, vm);
        *status = measurement->h2s.h2s_status;
    } else if (measurement->type == LEVEL_SENSOR) {
        n = level_measurements_to_visual(measurement, vm);
        *status = measurement->level.level_1_status;
    } else if (measurement->type == FLOW_SENSOR) {
        n = flow_measurements_to_visual(measurement, vm);
        *status = measurement->flow.direction_status;
    } else if (measurement->type == CURRENT_SENSOR) {
        n = current_measurements_to_visual(measurement, vm);
        *status = measurement->current.current_8_status;
    } else if (measurement->type == FLOW_WATER_SENSOR) {
        n = flow_water_measurements_to_visual(measurement, vm);
        *status = measurement->flow_water.flow_water_status;
    } else if (measurement->type == TEMPERATURE_SENSOR) {
        n = temperature_measurements_to_visual(measurement, vm);
        *status = measurement->temperature.temperature_status;
    } else if (measurement->type == CURRENT_AC_SENSOR) {
        n = current_ac_measurements_to_visual(measurement, vm);
        *status = measurement->current_ac.phase_1_status;
    } else if (measurement->type == FLOW_ULTRASONIC_SENSOR) {
        n = flow_ultrasonic_measurements_to_visual(measurement, vm);
        *status = measurement->flow_ultrasonic.rate_status;
    } else if (measurement->type == WEATHER_STATION_SENSOR) {
        n = weather_station_measurements_to_visual(measurement, vm);
        *status = measurement->weather_station.wind_gusts_status;
    } else if (measurement->type == WIND_SENSOR) {
        n = wind_measurements_to_visual(measurement, vm);
        *status = measurement->wind.wind_gusts_status;
    } else if (measurement->type == CTDO_SENSOR) {
        if (s.manufacturer == SEABIRD) {
            n = ctdo_measurements_to_visual_seabird(measurement, vm);
        } else {
            n = ysi_measurements_to_visual(measurement, vm);
        }
        *status = measurement->ctdo.conductivity_status;
    } else if (measurement->type == GPS_SENSOR) {
        n = gps_measurements_to_visual(measurement, vm);
        *status = measurement->gps.gps_status;
    } else if (measurement->type == VOLUME_SENSOR) {
        n = volume_measurements_to_visual(measurement, vm);
        *status = measurement->volume.volume_status;
    } else if (measurement->type == CURRENT_PROFILER_SENSOR) {
        n = current_profiler_measurement_to_visual(measurement, vm);
        *status = measurement->current_profiler_signature.current_profiler_signature_status;
    }
    return n;
}

/**
 * Display a measurement with n parameters, the primary with big font, secondary and tertiary with medium font and 4th,
 * with small font.
 */
static void display_visual_measurement_with_big_font(int n, enum measurement_status status,
                                                     struct visual_measurement *vm)
{
    display_set_font(FONT_40_PIXEL);
    display_printf(vm[0].format, (double)vm[0].value);
    display_set_font(FONT_8_PIXEL);
    display_move_rel(0, 25);
    display_printf(vm[0].units);
    if (n < 2) {
        return;
    }
    display_set_font(FONT_16_PIXEL);
    display_move(56, 8);
    display_printf(vm[1].format, (double)vm[1].value);
    display_printf("%s\n", vm[1].units);
    if (n < 3) {
        return;
    }
    display_move_rel(70, 0);
    display_printf(vm[2].format, (double)vm[2].value);
    display_set_font(FONT_8_PIXEL);
    display_move_rel(0, 7);
    display_printf("%s\n", vm[2].units);
    if (n < 4) {
        return;
    }
    display_move_rel(0, 5);
    display_printf(vm[3].format, (double)vm[3].value);
    display_printf("%s", vm[3].units);
    display_printf("  %s", measurement_status_to_string(status));
}

static void display_visual_measurement_with_medium_font(int sensor_number, int n, enum measurement_status status,
                                                        struct visual_measurement *vm)
{
    display_set_font(FONT_16_PIXEL);
    display_printf("%i: ", sensor_number);
    display_printf(vm[0].format, (double)vm[0].value);
    display_set_font(FONT_8_PIXEL);
    display_move_rel(0, 6);
    display_printf(vm[0].units);
    if (n < 2) {
        goto finish;
    }
    display_set_font(FONT_16_PIXEL);
    display_move_rel(0, -6);
    display_printf(vm[1].format, (double)vm[1].value);
    display_set_font(FONT_8_PIXEL);
    display_move_rel(0, 6);
    display_printf("%s", vm[1].units);
#if DEPRECATED
    if (n < 3) {
        goto finish;
    }
    display_set_font(FONT_16_PIXEL);
    display_move_rel(0, -7);
    display_printf(vm[2].format, vm[2].value);
    display_set_font(FONT_8_PIXEL);
    display_move_rel(0, 7);
    display_printf("%s", vm[2].units);
#endif
finish:
    display_printf("\n");
}

static void display_visual_measurement_with_small_font(int sensor_number, int n, enum measurement_status status,
                                                       struct visual_measurement *vm)
{
    display_set_font(FONT_8_PIXEL);
    display_printf("%i:  ", sensor_number);
    if (n > 3) {
        n = 3; /* Maximum 3 parameters */
    }
    for (int i = 0; i < n; ++i) {
        display_printf(vm[i].format, (double)vm[i].value);
        display_printf("%s ", vm[i].units);
    }
    display_printf("\n");
}

static void display_visual_measurement_with_small_font_two_columns(int sensor_number, int n,
                                                                   enum measurement_status status,
                                                                   struct visual_measurement *vm)
{
    display_set_font(FONT_8_PIXEL);
    if (sensor_number == 7) {
        display_move_rel(50, -8 * 6);
    } else if (sensor_number > 7) {
        display_move_rel(50, 0);
    }
    display_printf("%i:  ", sensor_number);
    if (n > 1) {
        n = 1; /* Maximum 1 parameter */
    }
    display_printf(vm[0].format, (double)vm[0].value);
    display_printf("%s ", vm[0].units);
    display_printf("\n");
}

static void display_visual_measurement_current_small(int n, enum measurement_status status,
                                                     struct visual_measurement *vm)
{
    display_move_rel((0) * 50, (0) * -32);
    display_set_font(FONT_8_PIXEL);
    display_printf(vm[0].format, (double)vm[0].value);
    display_printf(vm[0].units);
    display_printf("\n");
    if (n < 2) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((0) * 50), (0) * -0);
    display_printf(vm[1].format, (double)vm[1].value);
    display_printf("%s", vm[1].units);
    display_printf("\n");
    if (n < 3) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((0) * 50), (0) * -0);
    display_printf(vm[2].format, (double)vm[2].value);
    display_printf("%s", vm[2].units);
    display_printf("\n");
    if (n < 4) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((0) * 50), (0) * -0);
    display_printf(vm[3].format, (double)vm[3].value);
    display_printf("%s", vm[3].units);
    if (n < 5) {
        goto finish;
    }
    display_printf("\n");
    display_move_rel((1) * 50, (1) * -32);
    display_set_font(FONT_8_PIXEL);
    display_printf(vm[4].format, (double)vm[4].value);
    display_printf(vm[4].units);
    display_printf("\n");
    if (n < 6) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((1) * 50), (1) * -0);
    display_printf(vm[5].format, (double)vm[5].value);
    display_printf("%s", vm[5].units);
    display_printf("\n");
    if (n < 7) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((1) * 50), (1) * -0);
    display_printf(vm[6].format, (double)vm[6].value);
    display_printf("%s", vm[6].units);
    display_printf("\n");
    if (n < 8) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((1) * 50), (1) * -0);
    display_printf(vm[7].format, (double)vm[7].value);
    display_printf("%s", vm[7].units);
    display_printf("\n");
    if (n < 9) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((0) * 50), (1) * -0);
    display_printf(vm[8].format, (double)vm[8].value);
    display_printf("%s", vm[8].units);
finish:
    display_printf("\n");
}

static void display_visual_measurement_current_profiler_small(int n, enum measurement_status status,
                                                              struct visual_measurement *vm)
{
    display_move_rel((0) * 50, (0) * -32);
    display_set_font(FONT_8_PIXEL);
    display_printf(vm[0].format, (double)vm[0].value);
    display_printf(vm[0].units);
    display_printf("\n");
    if (n < 2) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((0) * 50), (0) * -0);
    display_printf(vm[1].format, (double)vm[1].value);
    display_printf("%s", vm[1].units);
    display_printf("\n");
    if (n < 3) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((0) * 50), (0) * -0);
    display_printf(vm[2].format, (double)vm[2].value);
    display_printf("%s", vm[2].units);
    display_printf("\n");
    if (n < 4) {
        goto finish;
    }
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((0) * 50), (0) * -0);
    display_printf(vm[3].format, (double)vm[3].value);
    display_printf("%s", vm[3].units);
finish:
    display_printf("\n");
}

static void display_visual_measurement_current_ac_medium(int n, enum measurement_status status,
                                                         struct visual_measurement *vm)
{
    display_move_rel((0) * 50, (0) * -32);
    display_set_font(FONT_16_PIXEL);
    display_printf(vm[0].format, (double)vm[0].value);
    display_printf(vm[0].units);
    display_printf("\n");
    if (n < 2) {
        return;
    }
    display_set_font(FONT_16_PIXEL);
    display_move_rel(((0) * 50), (0) * -0);
    display_printf(vm[1].format, (double)vm[1].value);
    display_printf("%s\n", vm[1].units);
    if (n < 3) {
        return;
    }
    display_move_rel((0) * 50, (0) * -0);
    display_printf(vm[2].format, (double)vm[2].value);
    display_printf(vm[2].units);
    display_printf("\n");
    if (n < 4) {
        return;
    }
    display_printf("\n");
    display_set_font(FONT_8_PIXEL);
    display_move_rel(((1) * 54), (1) * -63);
    display_printf(vm[3].format, (double)vm[3].value);
    display_printf("%s", vm[3].units);
}
