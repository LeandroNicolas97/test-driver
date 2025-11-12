#ifndef _SMART_SENSOR_H
#define _SMART_SENSOR_H

#include <stdint.h>
#include "measurement.h"

/* Defines */
#define SIZE_SMART_SENSOR_NAME 10 /* Max size of the sensor ID */
#define ACTIVATE               1  /*value to activate sensor driver*/
#define DEACTIVATE             0  /*value to deactivate sensor driver*/
#define BAR                    0
#define KPA                    1
#define CENTMETER              0
#define METER                  1

/**
 * Sensor manufacturers
 */
enum sensor_manufacturer {
    MANUFACTURER_NONE, /* Invalid manufacturer */
    NORTEK,
    LUFFT,             /* Weather Sensors sensor (Modbus)*/
    VAISALA,           /* Weather Sensors sensor (Modbus)*/
    INNOVEX,           /* Innovex sensor (Name at the beginning) */
    MAXBOTIX,          /* Maxbotix acoustic range sensors */
    PONSEL,            /* Ponsel sensor (Modbus) */
    TEXAS_INSTRUMENTS, /* Texas Instruments sensors (At the moment this means TMP107 sensors on 1-wire bus) */
    YOSEMITECH,        /* Yosemitech sensor (Modbus) */
    AQUAS,             /* Aquas sensor (Modbus)*/
    YSI,               /* ctdo sensor (Modbus)*/
    HUIZHONG,          /* huizhong flow sensor TUC-2000M*/
    TELEDYNE_ISCO,     /* Signature flow sensor */
    ANBSENSORS,        /* ANBsensors pH sensor*/
    TDS100,            /* Ultrasonic flow sensor */
    GPS,               /* GPS */
    CHEMINS,           /* Chemins Chrolophyll sensor (Modbus) */
    SEABIRD,           /* Seabird CTDO sensors */
    JIANGSU,           /* Flow sensor with accumulator (Modbus) */
    ACCONEER,          /* Accconer XM126 distance sensor */
    AQUADOPP,
    FLOWQUEST,
    WITMOTION,
    SENSOR_MANUFACTURER_END
};

/**
 * Structure to keep the information of a smart sensor
 */
struct smart_sensor {
    enum sensor_type type;
    enum sensor_manufacturer manufacturer;
    int channel;       /* The channel where the sensor is connected (different serial ports or multiplexed) */
    int number;        /* The number of the sensor. */
    int baudrate;      /* The baudrate to communicate with the sensor */
    int power_up_time; /* Time required before taking a measurement, in milliseconds. */
    int version;       /* A version number in case of different protocols */
    char name[SIZE_SMART_SENSOR_NAME];
};

/**
 * Smart sensor driver.
 * This structure keeps a list of all the functions a smart sensor can have.
 */
struct smart_sensor_driver {
    int (*max_sensors)(void);                                      /* Maximum number of sensors for this driver */
    int (*init_driver)(void);                                      /* Initialize the driver */
    int (*finish_driver)(void);                                    /* Finish the operations with the driver */
    int (*detect)(int sensor_number, struct smart_sensor *sensor); /* Detect a sensor */
    int (*prepare)(struct smart_sensor *sensor);                   /* Prepare the sensor */
    int (*finish)(struct smart_sensor *sensor);                    /* Finish the communication with the sensor */
    int (*calibrate_zero)(struct smart_sensor *sensor);            /* Calibrate the zero of the sensor */
    int (*calibrate_full)(struct smart_sensor *sensor);            /* Calibrate the full scale of the sensor */
    int (*acquire)(int tries, struct smart_sensor *sensor, struct measurement *m);
    int (*pass_command)(struct smart_sensor *sensor, char *command);
    const char *(*name)(void); /* Get a string with the name of the driver or family of sensors */
    int (*needs_external_voltage)(void);
};

/**
 * Drivers for every sensor type
 * TODO Do they need to be here?
 */
extern const struct smart_sensor_driver smart_sensor_driver_innovex;
extern const struct smart_sensor_driver smart_sensor_driver_maxbotix;
extern const struct smart_sensor_driver smart_sensor_driver_yosemitech;
extern const struct smart_sensor_driver smart_sensor_driver_ponsel;
extern const struct smart_sensor_driver smart_sensor_driver_aquas;
extern const struct smart_sensor_driver smart_sensor_driver_ysi;
extern const struct smart_sensor_driver smart_sensor_driver_gps_pa1010d;
extern const struct smart_sensor_driver smart_sensor_driver_vaisala;
extern const struct smart_sensor_driver smart_sensor_driver_lufft;
extern const struct smart_sensor_driver smart_sensor_driver_signature_nortek;
extern const struct smart_sensor_driver smart_sensor_driver_huizhong;
extern const struct smart_sensor_driver smart_sensor_driver_signature_flow;
extern const struct smart_sensor_driver smart_sensor_driver_anb;
extern const struct smart_sensor_driver smart_sensor_driver_tds100;
extern const struct smart_sensor_driver smart_sensor_driver_chemins;
extern const struct smart_sensor_driver smart_sensor_driver_seabird;
extern const struct smart_sensor_driver smart_sensor_driver_jiangsu_flow;
extern const struct smart_sensor_driver smart_sensor_driver_xm126;
extern const struct smart_sensor_driver smart_sensor_driver_aquadopp_nortek;
extern const struct smart_sensor_driver smart_sensor_driver_flowquest;
extern const struct smart_sensor_driver smart_sensor_driver_wtvb01;
/**
 * Pass a command to the smart sensor TODO put in another place maybe
 */
void smart_sensor_pass_command(uint8_t sensor_number, char *command);

/**
 * Read a measurement string from the smart sensor.
 * @param sensor_number The sensor to read
 * @param response A pointer to a buffer to store the sensor response
 * @param size The size of the buffer
 * @return The amount of bytes read from the sensor
 */
int smart_sensor_get_oxygen_measurement(uint8_t sensor_number, char *response, uint8_t size);

/**
 * Detect all the sensors connected to the serial port
 * @return the number of sensors detected, negative on error.
 */
int smart_sensors_detect_all(void);

/**
 * Prepare the smart-sensors to start a measurement, this means turning them on and
 * enabling the serial port to communicate with them
 */
void smart_sensors_prepare(void);

/**
 * Prepare all the smart sensor
 */
void smart_sensor_prepare_all(int n_of_sensors);

/**
 * Get the driver for the specified sensor
 * @param The sensor number
 * @return The driver for the sensor, or NULL if there is no driver
 */
const struct smart_sensor_driver *driver_for_sensor(int sensor_number);

/**
 * Get a pointer to a smart sensor which has been detected.
 */
struct smart_sensor *smart_sensor_get(uint8_t sensor_number);

/**
 * Check if the smart sensor can be calibrated.
 * If the driver provides functionality to calibrate the sensor, this return true
 * @return 0 if the driver cannot calibrate the sensor. 1 if it can.
 */
int smart_sensor_can_calibrate(int sensor_number);

/**
 * Acquire all the smart sensors and store the measuruemnts in the specified array.
 * @param n_of_sensor The number of sensors to read
 * @param measurement a pointer to store all the measurements
 * @return the number of measurements acquired
 */
int smart_sensors_aquire_all(int n_of_sensors, int communication_tries, struct measurement *measurement);

/**
 * Finish the operation with the smart sensors
 */
void smart_sensors_finish(void);

/**
 * Send the calibration command to the specified sensor.
 * The sensor must be already ON
 * @param sensor_number The sensor to calibrate
 * @param cal_value The value to set for the actual measurement
 * @return 1 if calibrated, 0 not, negative on error
 */
int smart_sensor_calibrate(int sensor_number, float cal_value);

/**
 * Get the driver for the specified manufacturer
 * @param manufacturer The manufacturer of the sensor
 * @return The driver for the sensor or NULL if there is no driver
 */
const struct smart_sensor_driver *driver_for_manufacturer(enum sensor_manufacturer manufacturer);

/**
 * Initialize the serial port used to communicate with the smart sensor
 */
void smart_sensor_init_serial_port(void);

/**
 * Strip the right part of a string, remove any CR or NL.
 */
void strip_right(char *s);

/**
 * Get the total number of sensors detected.
 */
int total_sensors_detected(void);

/**
 * Get the preheat time needed to powerup the slowest sensor detected
 */
int get_sensors_preheat_time_ms(void);

/**
 * Get as much as possible data from a sensor until the amount specified
 * is full or the sensor stopped sending data.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @return The number of bytes stored in the buffer
 */
int smart_sensor_receive_data(char *response, uint8_t size);

/**
 * Get a string from the sensors UART. Return immediately after receiving a
 * newline or after a timeout. The newline is not included in the buffer.
 * @param response A pointer to a buffer to store the received data
 * @param size Size of the receive buffer
 * @return The number of bytes stored in the buffer
 */
int smart_sensor_get_response(char *response, uint8_t size);

/**
 * Check if the configuration of the sensor has changed.
 * It is only important to have the same sequence of sensor types.
 * If the list of sensors detected is the same as the list of sensor types
 * provided, then the configuration has not changed.
 * @param n_of_sensors The number of sensor to check
 * @param configured_sensor An array with the types of sensors configured.
 * @return true if all configured sensors are the same as the detected sensors.
 */
int has_sensor_list_changed(int n_of_sensors, enum sensor_type *configured_sensor);

/**
 * Time elapsed since some event.
 * TODO Move to other place
 */
uint32_t tics_elapsed(uint32_t tic_stamp);

/**
 * Restores drivers saved in the NVS
 */
void configure_sensor_drivers(void);

/**
 * Function that changes maufacturers selected for sensor drivers
 */
void sensor_switch(enum sensor_manufacturer manufacturer, int state);

/**
 * Function that checks if external 12V are needed
 */
int smart_sensors_detect_voltage(void);

/*
 * @brief: passes pressure unit, only kPa or bar at the moment
 */
int pass_pressure_unit(void);

/*
 * @brief: passess phreatic level unit, meter or centimeter at the moment
 */
int pass_phreatic_unit(void);

/*
 * @brief: restores the flag to get the unit of the measurements again
 */
void restore_meas_unit_flag(void);

#endif /*_SMART_SENSOR_H*/
