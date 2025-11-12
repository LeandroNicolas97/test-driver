/**
 *  \file ui_valves.c
 *  \brief Visualization of the solenoid valves.
 *
 *  \author Pablo Santamarina Cuneo
 *  \date 02 septemeber 2019
 *
 *  Copyright 2018 Innovex Tecnologias Ltda. All rights reserved.
 */
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "microio.h"
#include "defaults.h"
#include "display_fb.h"
#include "hardware.h"
#include "adc.h"
#include "solenoid.h"
#include "debug.h"
#include "errorcodes.h"
#include "oxygen_control.h"
#include "watchdog.h"

/**
 * Detect all the valves connected to this device
 */
void detect_all_valves(void)
{
    enum solenoid_status status;
    int mv_charged;
    int mv_discharged;

    display_clear();
    display_move(3, 0);
    display_printf("Detecting valves\n");
    for (int valve_nr = 0; valve_nr < MAX_N_VALVES; ++valve_nr) {
        watchdog_reset();
        status = detect_valve(valve_nr, &mv_charged, &mv_discharged);
        display_move(0, (valve_nr + 1) * 8);
        display_printf("Valve %i %i %i %i\n", valve_nr + 1, status, mv_charged, mv_discharged);
    }
}
