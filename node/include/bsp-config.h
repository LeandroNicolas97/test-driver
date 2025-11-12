/**
 *                        bsp-config.h  -  description
 *                            -------------------
 *   begin                : Tue Nov 6 2001
 *   copyright            : (C) 2001 by Pablo Santamarina Cuneo
 *   email                : pablo@santamarina.de
 */

#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/**
 * \file
 * Hardware and configuration dependent data.
 * All defines that are hardware or configuration dependent should
 * be defined here in a central place.
 */

#define BOARD_NAME_STRING "Oxycontroller 4"

/* Debug configuration */
#define DEBUG_USE_PRINTK 1

/* General configuration */
#define ANALOG_VCC                 3.3
#define FLUORESCENCE_SAMPLING_RATE 921600.00 /* 1000000 */

/* ADC configuration*/
#define ADC_N_OF_INPUTS  6 /* 4 external + internal Temperature and battery */
#define ADC_MIN_VOLTAGE  0.0
#define ADC_MAX_VOLTAGE  3.3
#define ADC_SCALE_WIDTH  20
#define ADC_SCALE_HEIGHT 90
#define ADC_BITS         12

/* Led configuration */
#define N_OF_LEDS  5
#define LED_STATUS 0 /* The status LED on the board */

/**
 * Digital to analog converter
 */
#define DAC_MAX_VALUE 4095 /* 12 bit dac */

/* The reference resistor should be at least 0.1%. Usually 22.0KOhm */
#define DEFAULT_REFERENCE_RESISTOR 22000.00

/**
 * Serial ports
 */
#define UART_SMART_SENSOR     0 /* Uart to connect the smart sensors */
#define SERIAL_PORT_CABLE     1 /* Cable is connected to UART 3 */
/* Port used for the communication with the user */
#define COMM_UART             SERIAL_PORT_CABLE /* Serial cable port */
#define RS232_PORT            2                 /* Iridium port for RS232 protocol */
/*
 * EEPROM configuration
 */
/* If a magic-key is written here, the bootloader jumps immediately to the application */
#define EE_BOOTLOADER_DISABLE 0x00
/* Configuration variables are stored here */
#define EE_CONFIGURATION      0x04

/* Display configuration */
#define USE_INVERTED_LCD     1   /* Invert the LCD display direction */
#define DISPLAY_ROWS         64  /* Vertical pixels */
#define DISPLAY_COLUMNS      102 /* Horizontal pixels */
#define DISPLAY_STRIDE       13  /* Bytes per one pixel row */
#define DEFAULT_LCD_CONTRAST 40

/* The reference resistor should be at least 0.1%. Usually 22.0KOhm */
#define DEFAULT_REFERENCE_RESISTOR 22000.00

/** The maximum number of external sensors */
#define MAX_EXTERNAL_SENSORS 16
#define MAX_N_VALVES         2

#endif /* BSP_CONFIG_H */
