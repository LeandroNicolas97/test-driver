#include <string.h>
#include <zephyr/pm/state.h>
#include "zephyr/sys/printk.h"
#include "hardware.h"
#include "microio.h"
#include "comunication.h"
#include "configuration.h"
#include "measurement_storage.h"
#include "watchdog.h"
#include "radio.h"
#include "debug.h"
#include "actual_conditions.h"
#include "satellite_compression.h"
#include "shell_commands.h"
#if CONFIG_EXTERNAL_DATALOGGER
#include "compressed_measurement.h"
#include "external_datalogger.h"
#endif

void send_data_from_storage(int time_of_last_measurement)
{
    size_t n_size = 110;
    uint16_t unsended_data;
    uint8_t try = 5; /* Number of transmission attempts. */
    uint32_t missed_conection = 0;
    char data[255];
    char *buffer;

    unsended_data = unsended_data_get();
    printk("%i datos para enviar\n", unsended_data);
    while (unsended_data > 0) { /* Send until unsended is 0. */
        watchdog_reset();
        measurement_storage_get(data, n_size, unsended_data - 1);
        if (!strcmp(data, "")) {
            measurement_storage_get(data, n_size, unsended_data - 2);
        }
        char str_cpy[strlen(data) + 1];

        strcpy(str_cpy, data);
        char *frame_name = strtok_r(str_cpy, ":", &buffer);

        frame_name = strtok(buffer, ":");
        for (int i = 0; i < try; i++) {
            send_frame(data, strlen(data) + 1);
            if (check_acknowledgment(data, frame_name, time_of_last_measurement)) {
                unsended_data_flush_last(); /* Delete the last frames sent. */
                unsended_data = unsended_data_get();
                actual_state.missed_conection = missed_conection;
                missed_conection = 0;
                break;
            }
            missed_conection++;
        }
        memset(data, '\0', strlen(data));
        if (missed_conection >= try) {
            printk("Not associated\n");
            actual_state.coordinator_found = 0;
            break;
        }
    }
    if (unsended_data <= 0) {
        char payload[10];

        watchdog_reset();
        sleep_microseconds(300000);
        usnprintf(payload, 10, "%s %s", cfg.name, "END");
        radio_send_str(payload, strlen(payload));
    }
    DEBUG("Fin envio de datos\n");
}

/*
 * Compresses and sends processed measurements from an ADCP (Acoustic Doppler Current Profiler) sensor.
 * Splits the compressed data into small packets and stores them for later transmission.
 *
 * @param time_of_last_measurement Timestamp of the last measurement (in seconds since UNIX epoch or similar)
 * @param sensor_manufacturer ADCP sensor type (e.g., NORTEK or FLOWQUEST), determines the compression method
 *
 * The function selects the appropriate compression method based on the manufacturer, splits the
 * compressed output into fixed-size fragments, encodes them as hexadecimal text, and appends them
 * to the outgoing storage queue using `measurement_storage_append()`. Finally, it calls
 * `send_data_from_storage()` to initiate transmission.
 */
void send_adcp_measurements(int time_of_last_measurement, enum sensor_manufacturer sensor_manufacturer)
{
    uint8_t compressed[512];
    uint16_t pos = 0;
    int n_bits = 0;

    if (sensor_manufacturer == NORTEK || sensor_manufacturer == FLOWQUEST) {
        DEBUG("Usando compresion para ADCP.\n");
        n_bits = compress_adcp_measurement(&adcp_processed_data, compressed);
    } else {
        return;
    }

    int size = n_bits / 8;

    if ((n_bits % 8) != 0) {
        size++;
    }

    const int NUM_PACKETS = 6; /* A fixed value of 6 packs per decompression item is left */
    const int MAX_BYTES_PER_PACKET = 43;
    uint8_t data[2 * MAX_BYTES_PER_PACKET + 24];

    /* Verify that the data is sufficient to fill 6 packages */
    const int MIN_BYTES_PER_PACKET = 1; /* Minimum byte per packet */

    if (size < NUM_PACKETS * MIN_BYTES_PER_PACKET) {
        DEBUG("Datos insuficientes para %i paquetes (tamaño: %i bytes). Enviando en menos paquetes.\n",
              NUM_PACKETS,
              size);
        /* Calculates the optimal number of packets based on packet size */
        int optimal_packets = (size + MIN_BYTES_PER_PACKET - 1) / MIN_BYTES_PER_PACKET;

        if (optimal_packets > NUM_PACKETS) {
            optimal_packets = NUM_PACKETS;
        }
        if (optimal_packets < 1) {
            optimal_packets = 1;
        }

        DEBUG("Enviando en %i paquetes en su lugar.\n", optimal_packets);

        for (int part = 0; part < optimal_packets; part++) {
            watchdog_reset();
            int part_number = optimal_packets - 1 - part;
            int start_idx = (part * size) / optimal_packets;
            int end_idx = ((part + 1) * size) / optimal_packets;

            if (part == optimal_packets - 1) {
                end_idx = size;
            }
            int current_part_size = end_idx - start_idx;

            pos = usnprintf(data, sizeof(data), ":%u:%s:%i:ADCP ", time_of_last_measurement, cfg.name, part_number);

            for (int j = start_idx; j < end_idx; j++) {
                watchdog_reset();
                pos += usnprintf(data + pos, sizeof(data) - pos, "%.2x", compressed[j]);
            }

            measurement_storage_append(data, sizeof(data));
            DEBUG("Paquete %i: bytes %i-%i (tamaño: %i)\n", part_number, start_idx, end_idx - 1, current_part_size);
        }
    } else {
        DEBUG("Compressed adcp_data size: %i, enviando en %i paquetes\n", size, NUM_PACKETS);

        /* The data are distributed in exactly 6 packages */
        for (int part = 0; part < NUM_PACKETS; part++) {
            watchdog_reset();
            int part_number = NUM_PACKETS - 1 - part;
            int start_idx = (part * size) / NUM_PACKETS;
            int end_idx = ((part + 1) * size) / NUM_PACKETS;
            /* Ensures that the last packet includes all remaining bytes */
            if (part == NUM_PACKETS - 1) {
                end_idx = size;
            }
            int current_part_size = end_idx - start_idx;

            if (current_part_size <= 0) {
                DEBUG("Advertencia: Paquete %i está vacío, saltando.\n", part_number);
                continue;
            }
            /* Package header is created */
            pos = usnprintf(data, sizeof(data), ":%u:%s:%i:ADCP ", time_of_last_measurement, cfg.name, part_number);
            for (int j = start_idx; j < end_idx; j++) {
                watchdog_reset();
                pos += usnprintf(data + pos, sizeof(data) - pos, "%.2x", compressed[j]);
            }
            measurement_storage_append(data, sizeof(data));
            DEBUG("Paquete %i: bytes %i-%i (tamaño: %i)\n", part_number, start_idx, end_idx - 1, current_part_size);
        }
    }
    send_data_from_storage(actual_state.n_of_sensors_detected);
}

int if_received_data(char *data)
{
    int ret = SLEEPING;
    uint8_t queue = 0;
    uint32_t init_time;
    struct received_command command[3];

    memset(data, '\0', strlen(data));
    init_time = k_uptime_get();
    while ((k_uptime_get() - init_time) < 2000) {
        watchdog_reset();
        if (radio_receive_str(data, 255, (2 * cfg.time_on_air), cfg.name) > 0) {
            if (strlen(data) > 2) { /* if len data > 2 it is a command. */
                init_time = k_uptime_get();
                strncpy(command[queue].command, data, SIZE_COMMAND);
                queue++;
                ret = RECEIVING;
                memset(data, '\0', strlen(data));
            }
        }
        if (queue > 3) {
            queue = 2;
            break;
        }
    }
    for (int i = 0; i < queue; i++) {
        data_reception(command[i].command);
    }
    memset(data, '\0', strlen(data));
    return ret;
}

/*
 * Receiving state
 */
int receiving_commands(char *data)
{
    uint32_t init_time = k_uptime_get();
    uint32_t actual_time = init_time;

    while ((actual_time - init_time) <= RECEPTION_TIME) {
        watchdog_reset();
        if (radio_receive_str(data, 255, (4 * cfg.time_on_air), cfg.name) > 0) {
            data_reception(data);
            init_time = k_uptime_get();
            memset(data, '\0', strlen(data));
        }
        actual_time = k_uptime_get();
    }
    watchdog_reset();
    return SLEEPING;
}

/*
 * Receiving state
 */
int data_reception(char *data)
{
    for (int i = 0; i < strlen(data); i++) {
        shell_char_received(data[i]);
    }
    shell_char_received('\r');
    return 1;
}

int is_channel_free(void)
{
    char data[255];
    int ret = 0;
    uint16_t timeout = 800;

    for (int i = 0; i < 3; i++) {
        if (radio_receive_str(data, 255, timeout, cfg.name) == 0) {
            DEBUG("Busy channel\n");
            ret = 0;
        } else {
            DEBUG("Free channel\n");
            ret = 1;
            break;
        }
    }
    return ret;
}

int check_acknowledgment(char *data, char *frame_name, int time_of_last_measurement)
{
    char name[10];
    int ret = 0;

    if (radio_receive_str(data, 255, 2 * cfg.time_on_air, frame_name) > 0) {
        usnprintf(name, 10, "%s %s", frame_name, "OK");
        if (!strncmp(name, data, 4)) {
            watchdog_reset();
            actual_state.coordinator_found = 1;
            time_of_last_measurement = get_timestamp(data);
            set_current_time(&time_of_last_measurement);
            ret = 1;
        }
    }
    return ret;
}

void send_ping(void)
{
    char payload[10];
    char data[255];
    uint8_t radio_state = SLEEPING;

    watchdog_reset();
    sleep_microseconds(300000);
    usnprintf(payload, 10, "%s %s", cfg.name, "PING");
    radio_send_str(payload, 10);
    radio_state = if_received_data(data);
    if (radio_state == RECEIVING) {
        radio_state = receiving_commands(data);
    }
    watchdog_reset();
}

/**
 * @brief Check if ADCP conditions are met
 *
 * This function verifies that:
 * - At least one ADCP driver (NORTEK, AQUADOPP, or FLOWQUEST) is present.
 * - Exactly one sensor is detected.
 * - The detected sensor is a CURRENT_PROFILER_SENSOR.
 * - The measurement status is MEASUREMENT_OK.
 *
 * @return true if all conditions are met, false otherwise.
 */
bool check_for_adcp(void)
{
    return ((sen_drv.sensor_driver[NORTEK] != NULL || sen_drv.sensor_driver[AQUADOPP] != NULL ||
             sen_drv.sensor_driver[FLOWQUEST] != NULL) &&
            actual_state.n_of_sensors_detected == 1 && actual_measurements[0].type == CURRENT_PROFILER_SENSOR &&
            actual_measurements[0].current_profiler_signature.current_profiler_signature_status == MEASUREMENT_OK);
}
