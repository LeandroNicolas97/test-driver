/*
 * Hardware control functions for the oxycontroller.
 * Zephyr specific implementation
 */

#include "sensor_power_hw.h"
#include "errorcodes.h"
#include <zephyr/kernel.h>
/* #include <sys/printk.h> */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include "solenoid.h"
#include "watchdog.h"
#include "hardware.h"
#include "smart_sensor.h"

/* The device-tree node identifier for the "sensorpower" alias. */
#define SENSORPOWER_NODE DT_NODELABEL(sensorpower0)

/* A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec power_pin = GPIO_DT_SPEC_GET(SENSORPOWER_NODE, gpios);

/**
 * Initialize the sensors' power control
 */
int sensor_power_init(void)
{
    int ret;
    /* check and configure */
    if (!gpio_is_ready_dt(&power_pin)) {
        return -E_NOT_DETECTED;
    }
    ret = gpio_pin_configure_dt(&power_pin, GPIO_OUTPUT);
    if (ret < 0) {
        return -E_NOT_DETECTED;
    }

    return 0;
}

/**
 * Turn the power of the external sensors ON.
 */
void sensor_power_on(int external_voltage)
{
    if (external_voltage) {
        solenoid_prepare();
        solenoid_activate_forward(0);
        watchdog_reset();
        sleep_microseconds(100000);
        solenoid_release();
    }
    gpio_pin_set_dt(&power_pin, 1);
}

/**
 * Turn the power of the external sensors OFF.
 */
void sensor_power_off(int external_voltage)
{
    if (external_voltage) {
        solenoid_prepare();
        solenoid_activate_reverse(0);
        watchdog_reset();
        sleep_microseconds(100000);
        solenoid_release();
    }
    gpio_pin_set_dt(&power_pin, 0);
}
