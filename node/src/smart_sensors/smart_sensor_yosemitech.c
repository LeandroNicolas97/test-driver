/*
 * Communication with the smart sensors
 */

#include <string.h>
#include <stdlib.h>
#include "bsp-config.h"
#include "smart_sensor.h"
#include "serial.h"
#include "errorcodes.h"
#include "microio.h"
#include "crc16.h"
#include "hardware.h"
#include "timeutils.h"
#include "modbus.h"
#include "debug.h"
#include "configuration.h"

#define DETECTION_TRIES   2
#define MAX_SENSORS       6
#define MAX_RESPONSE_SIZE 128

#define TURBIDITY_SENSOR_SLAVE_ADDR        0x15
#define SUSPENDED_SOLIDS_SENSOR_SLAVE_ADDR 0x16
#define CHLOROPHYLL_SENSOR_SLAVE_ADDR      0x0B
#define CONDUCTIVITY_SENSOR_SLAVE_ADDR     0x1F
#define CONDUCTIVITY_2_SENSOR_SLAVE_ADDR   0x20
#define DISSOLVED_OXYGEN_SENSOR_SLAVE_ADDR 0x01
#define pH_SENSOR_SLAVE_ADDR               0x04

#define DEPRECATED 0

/**
 * Enum for diferent type of sensors
 */
enum yosemitech_sensor_type {
    YOSEMITECH_pH,
    YOSEMITECH_TURBIDITY,
    YOSEMITECH_SUSPENDED_SOLIDS,
    YOSEMITECH_CHLOROPHYLL,
    YOSEMITECH_CONDUCTIVITY,
    YOSEMITECH_DISSOLVED_OXYGEN,
    YOSEMITECH_END
};

/**
 * Driver function prototypes
 */
static int max_sensors(void);                                      /* Maximum number of sensors for this driver */
static const char *name(void);                                     /* Name of the driver */
static int init_driver(void);                                      /* Initialize the driver */
static int finish_driver(void);                                    /* Finish the operations with the driver */
static int detect(int sensor_number, struct smart_sensor *sensor); /* Detect a sensor */
static int prepare(struct smart_sensor *sensor);                   /* Prepare the sensor */
/* static int finish(struct smart_sensor *sensor); // Finish the communication with the sensor */
/* static int calibrate_zero(struct smart_sensor *sensor); // Calibrate the zero of the sensor */
/* static int calibrate_full(struct smart_sensor *sensor); // Calibrate the full scale of the sensor */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *m);
/* static int pass_command(struct smart_sensor *sensor, char *command); */
static int needs_external_voltage(void);

/**
 * Smart sensor driver structure for the Yosemitech sensors
 * This structure keeps a list of all the functions a smart sensor can have.
 */
const struct smart_sensor_driver smart_sensor_driver_yosemitech = {
    .max_sensors = max_sensors,
    .init_driver = init_driver,
    .finish_driver = finish_driver,
    .detect = detect,
    .prepare = prepare,
    .finish = NULL,         /* finish, */
    .calibrate_zero = NULL, /* calibrate_zero, */
    .calibrate_full = NULL,
    .acquire = acquire,
    .pass_command = NULL,
    .name = name,
    .needs_external_voltage = needs_external_voltage,
};

/*
 * Local prototypes
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg,
                                 uint16_t coils);
static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement);

#if DEPRECATED
/*
 * Local variables
 */
struct modbus modbus_response, modbus_request;
uint16_t modbus_data_buffer[MAX_RESPONSE_SIZE];
#endif

/*
 * Get the maximum number of sensors of this type this driver can handle
 */
static int max_sensors(void)
{
    return MAX_SENSORS;
}

/**
 * Get the name of the driver.
 */
static const char *name(void)
{
    return "Yosemitech";
}

/*
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
static int init_driver(void)
{
    /* serial_set_baudrate(UART_SMART_SENSOR, B9600); */
    return 0;
}

/*
 *
 */
static int prepare(struct smart_sensor *sensor)
{
    int response_status;
    struct modbus_frame f;

    serial_flush(UART_SMART_SENSOR);
    switch (sensor->type) {
        case YOSEMITECH_TURBIDITY:
        case YOSEMITECH_SUSPENDED_SOLIDS:
        case YOSEMITECH_CHLOROPHYLL:
        case YOSEMITECH_DISSOLVED_OXYGEN:
            prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, 0x2500, 2);
            break;
        case YOSEMITECH_CONDUCTIVITY:
            prepare_modbus_frame(&f, sensor, MODBUS_WRITE_MULTIPLE_HOLDING_REGISTERS, 0x1C00, 0);
            break;
        case YOSEMITECH_pH:
            prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, 0x2400, 2);
            break;
        default:
            prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, 0x2500, 1);
            break;
    }
    modbus_query(UART_SMART_SENSOR, &f);
    serial_flush(UART_SMART_SENSOR);
    response_status = modbus_poll(UART_SMART_SENSOR, &f, LITTLE_ENDIAN);
    if (response_status == -E_NOT_DETECTED) {
        return response_status;
    }
    if (response_status == -E_BAD_CHECKSUM) {
        return response_status;
    }
    if (response_status == -E_INVALID) {
        return response_status;
    }

    return 0;
}

/*
 * Finish the operation with the smart sensors
 */
static int finish_driver(void)
{
    return 0;
}

/**
 * Check for a single sensor with the specified number on the RS-485 bus
 * @param sensor_number The number of the sensor to check for.
 * @param measurement A pointer to return a measurement from the sensor
 * @param sensor A pointer to store the sensor information
 * @return True if a sensor was detected
 */
static int detect(int sensor_number, struct smart_sensor *sensor)
{
    int tries;

    DEBUG("Checking Yosemitech %i...\n", sensor_number);
    for (tries = 0; tries < DETECTION_TRIES; tries++) {
        sensor->number = sensor_number;
        switch (sensor->number) {
            case 0:
            case 1:
            case 2:
                sensor->type = YOSEMITECH_pH;
                break;
            case 3:
                sensor->type = YOSEMITECH_TURBIDITY;
                break;
            case 4:
                sensor->type = YOSEMITECH_SUSPENDED_SOLIDS;
                break;
            case 5:
                sensor->type = YOSEMITECH_CHLOROPHYLL;
                break;
            case 6:
            case 7:
                sensor->type = YOSEMITECH_CONDUCTIVITY;
                break;
            case 8:
                sensor->type = YOSEMITECH_DISSOLVED_OXYGEN;
                break;
            default:
                sensor->type = YOSEMITECH_END;
                break;
        }
        if (prepare(sensor) == 0) {
            sensor->manufacturer = YOSEMITECH;
            sensor->power_up_time = YOSEMITECH_POWERUP_TIME;
            sensor->channel = 0;           /* Currently we have only one channel */
            strcpy(sensor->name, "YOSEM"); /* TODO: make this string from the sensor number */
            DEBUG("OK\n");
            return 1;
        } else {
            DEBUG("NO\n");
        }
    }
    return 0;
}

/**
 * Acquire one smart sensor attached to this device.
 * @param tries The number of retries if there are problems with the sensor
 * @param sensor The sensor to read
 * @param measurements A Pointer to store the result measurement
 * @return 1 if OK, 0 on error // TODO Better return the error code
 */
static int acquire(int tries, struct smart_sensor *sensor, struct measurement *measurement)
{
    while (tries > 0) {
        DEBUG(" Trying\n");
        if (modbus_request_measurement(sensor, measurement)) {
            break;
        }
        DEBUG("Error reading sensor %s\n", sensor->name);
        tries--;
    }
    if (tries > 0) {
        return 1;
    } else {
        return 0;
    }
}

/*
 *
 */
static void prepare_modbus_frame(struct modbus_frame *f, struct smart_sensor *sensor, uint8_t function, uint16_t reg,
                                 uint16_t coils)
{
    switch (sensor->number) {
        case 0:
            f->slave_address = pH_SENSOR_SLAVE_ADDR;
            break;
        case 1:
            f->slave_address = pH_SENSOR_SLAVE_ADDR + 1;
            break;
        case 2:
            f->slave_address = pH_SENSOR_SLAVE_ADDR + 2;
            break;
        case 3:
            f->slave_address = TURBIDITY_SENSOR_SLAVE_ADDR;
            break;
        case 4:
            f->slave_address = SUSPENDED_SOLIDS_SENSOR_SLAVE_ADDR;
            break;
        case 5:
            f->slave_address = CHLOROPHYLL_SENSOR_SLAVE_ADDR;
            break;
        case 6:
            f->slave_address = CONDUCTIVITY_SENSOR_SLAVE_ADDR;
            break;
        case 7:
            f->slave_address = CONDUCTIVITY_2_SENSOR_SLAVE_ADDR;
            break;
        case 8:
            f->slave_address = DISSOLVED_OXYGEN_SENSOR_SLAVE_ADDR;
            break;
        default:
            f->slave_address = 1;
            break;
    }
    f->function_code = function;
    f->register_address = reg;
    f->n_coils = coils;
}

/*
 *
 */
static int modbus_request_measurement(struct smart_sensor *sensor, struct measurement *measurement)
{
    struct modbus_frame f;
    int response_status;
    float param1 = 0.0;
    float param2 = 0.0;
    float param3 = 0.0;
    int number_registers;
    int start_address;
    /* serial_flush(UART_SMART_SENSOR); */
    /* modbus_query(UART_SMART_SENSOR, &f); */
    sleep_microseconds(20000);
    if (sensor->number == 3) {
        number_registers = 4;
        start_address = 0x2600;
    } else if (sensor->number == 8) {
        number_registers = 6;
        start_address = 0x2600;
    } else if (sensor->number >= 0 && sensor->number < 3) {
        number_registers = 2;
        start_address = 0x2400;
    } else if (sensor->number == 6 || sensor->number == 7) {
        number_registers = 5;
        start_address = 0x2600;
    } else {
        number_registers = 4;
        start_address = 0x2600;
    }
    DEBUG("Star Register: 0x%.4x Size Register: %i\n", start_address, number_registers);
    DEBUG("Nro Sensor: %i\n", sensor->number);
    prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, start_address, number_registers);
    serial_flush(UART_SMART_SENSOR);
    modbus_query(UART_SMART_SENSOR, &f);
    response_status = modbus_poll(UART_SMART_SENSOR, &f, LITTLE_ENDIAN);
    if (response_status == -E_NOT_DETECTED) {
        measurement->sensor_status = SENSOR_NOT_DETECTED;
        return 0;
    }
    if (response_status == -E_BAD_CHECKSUM) {
        measurement->sensor_status = SENSOR_COMMUNICATION_BAD_CRC;
        return 0;
    }
    if (response_status == -E_INVALID) {
        measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
        return 0;
    }
    /*frame is ok*/
    if (number_registers == 2) {
        param1 = modbus_get_float(&f.data[0]);
        DEBUG("T: %.2f\n", (double)param1);
    } else if (number_registers == 4 || number_registers == 5) {
        param1 = modbus_get_float(&f.data[0]);
        param2 = modbus_get_float(&f.data[2]);

        DEBUG("T: %.2f param2: %.2f\n", (double)param1, (double)param2);
    } else if (number_registers == 6) {
        param1 = modbus_get_float(&f.data[0]);
        param2 = modbus_get_float(&f.data[2]);
        param3 = modbus_get_float(&f.data[4]);
        DEBUG("T: %.2f param2: %.2f param3: %.2f\n", (double)param1, (double)param2, (double)param3);
    }

    if ((sensor->number) >= 0 && (sensor->number < 3)) {
        /*read pH*/
        number_registers = 2;
        start_address = 0x2800;
        prepare_modbus_frame(&f, sensor, MODBUS_READ_HOLDING_REGISTERS, start_address, number_registers);
        serial_flush(UART_SMART_SENSOR);
        modbus_query(UART_SMART_SENSOR, &f);
        response_status = modbus_poll(UART_SMART_SENSOR, &f, LITTLE_ENDIAN);
        if (response_status == -E_NOT_DETECTED) {
            measurement->sensor_status = SENSOR_NOT_DETECTED;
            return 0;
        }
        if (response_status == -E_BAD_CHECKSUM) {
            measurement->sensor_status = SENSOR_COMMUNICATION_BAD_CRC;
            return 0;
        }
        if (response_status == -E_INVALID) {
            measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
            return 0;
        }
        param2 = modbus_get_float(&f.data[0]);
        DEBUG("pH: %.2f\n", (double)param2);
    }
    /*put data in measurement struct*/
    if (f.slave_address == TURBIDITY_SENSOR_SLAVE_ADDR) { /*TODO: do this in a more apropiate way*/
        struct turbidity_measurement *m = &(measurement->turbidity);

        measurement->sensor_status = SENSOR_OK;
        measurement->type = TURBIDITY_SENSOR;
        m->turbidity = param2;
        m->turbidity_status = MEASUREMENT_OK;
        m->depth = 10;
        m->temperature = param1;
        m->humidity = 0;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = MEASUREMENT_OK;
    } else if (f.slave_address == SUSPENDED_SOLIDS_SENSOR_SLAVE_ADDR) {
        struct suspended_solids_measurement *m = &(measurement->suspended_solids);

        measurement->sensor_status = SENSOR_OK;
        measurement->type = SUSPENDED_SOLIDS_SENSOR;
        m->suspended_solids = param2;
        m->suspended_solids_status = MEASUREMENT_OK;
        m->depth = 10;
        m->temperature = param1;
        m->humidity = 0;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = MEASUREMENT_OK;
    } else if (f.slave_address == CHLOROPHYLL_SENSOR_SLAVE_ADDR) {
        struct chlorophyll_measurement *m = &(measurement->chlorophyll);

        measurement->sensor_status = SENSOR_OK;
        measurement->type = CHLOROPHYLL_SENSOR;
        m->chlorophyll = (param2 < 0) ? 0.0f : param2;
        m->chlorophyll_status = MEASUREMENT_OK;
        m->depth = 10;
        m->temperature = param1;
        m->humidity = 0;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = MEASUREMENT_OK;
    } else if (f.slave_address == CONDUCTIVITY_SENSOR_SLAVE_ADDR) {
        struct conductivity_measurement *m = &(measurement->conductivity);

        measurement->sensor_status = SENSOR_OK;
        measurement->type = CONDUCTIVITY_SENSOR;
        /* m->conductivity = (param2 < 0 ) ? 0.0 : param2; */
        /* m->salinity = (param2 < 0 ) ? 0.0 : param2; */
        if (cfg.conductivity_freshwater == SEAWATER) {
            DEBUG("SEAWATER COND: %.2f\n", (double)param2);
            m->conductivity = param2; /* mS/cm */
            DEBUG("Conductivity: %.2fmS/cm\n", (double)m->conductivity);
        } else if (cfg.conductivity_freshwater == FRESHWATER) {
            DEBUG("FRESHWATER COND: %.2f\n", (double)param2);
            m->conductivity = param2 * 1000; /* uS/cm */
            DEBUG("Conductivity: %.2fuS/cm\n", (double)m->conductivity);
        }

        m->depth = 100;
        m->temperature = param1;
        m->humidity = 0;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = MEASUREMENT_OK;
        m->conductance = 0; /* fixed, sensor doesn't give this value */
        m->conductance_status = MEASUREMENT_VALUE_FIXED;
    } else if (f.slave_address == CONDUCTIVITY_2_SENSOR_SLAVE_ADDR) {
        struct conductivity_measurement *m = &(measurement->conductivity);

        measurement->sensor_status = SENSOR_OK;
        measurement->type = CONDUCTIVITY_SENSOR;
        /* m->conductivity = (param2 < 0 ) ? 0.0 : param2; */
        /* m->salinity = (param2 < 0 ) ? 0.0 : param2; */
        if (cfg.conductivity_freshwater == SEAWATER) {
            DEBUG("SEAWATER COND: %.2f\n", (double)param2);
            m->conductivity = param2; /* mS/cm */
            DEBUG("Conductivity: %.2fmS/cm\n", (double)m->conductivity);
        } else if (cfg.conductivity_freshwater == FRESHWATER) {
            DEBUG("FRESHWATER COND: %.2f\n", (double)param2);
            m->conductivity = param2 * 1000; /* uS/cm */
            DEBUG("Conductivity: %.2fuS/cm\n", (double)m->conductivity);
        }

        m->depth = 100;
        m->temperature = param1;
        m->humidity = 0;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = MEASUREMENT_OK;
        m->conductance = 0; /* fixed, sensor doesn't give this value */
        m->conductance_status = MEASUREMENT_VALUE_FIXED;
    } else if (f.slave_address == DISSOLVED_OXYGEN_SENSOR_SLAVE_ADDR) {
        struct oxygen_measurement *m = &(measurement->oxygen);

        measurement->sensor_status = SENSOR_OK;
        measurement->type = OXYGEN_SENSOR;
        m->temperature = (param1 < 0) ? 0.0f : param1;
        m->saturation = (param2 < 0) ? 0.0f : param2;
        m->concentration = (param3 < 0) ? 0.0f : param3;
        m->concentration_status = MEASUREMENT_OK;
        m->saturation_status = MEASUREMENT_OK;
        m->temperature_status = MEASUREMENT_OK;
    } else if (f.slave_address >= pH_SENSOR_SLAVE_ADDR && f.slave_address < pH_SENSOR_SLAVE_ADDR + 3) {
        struct pH_measurement *m = &(measurement->pH);

        measurement->sensor_status = SENSOR_OK;
        measurement->type = PH_SENSOR;
        m->temperature = param1 + cfg.temp_offset;
        m->depth = 14;
        m->pH = param2;
        m->humidity = 0;
        m->depth_status = MEASUREMENT_VALUE_FIXED;
        m->temperature_status = MEASUREMENT_OK;
        m->pH_status = MEASUREMENT_OK;
    } else {
        measurement->sensor_status = SENSOR_COMMUNICATION_ERROR;
        return 0;
    }
    return 1;
}

static int needs_external_voltage(void)
{
    return 1;
}
