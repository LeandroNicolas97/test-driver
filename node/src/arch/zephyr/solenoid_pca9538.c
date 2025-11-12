
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "solenoid.h"
#include "hardware.h"
#include "adc.h"
#include "bsp-config.h"
#include "i2c-hw.h"
#include "debug.h"
#include "watchdog.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(solenoid_pca9538, CONFIG_GPIO_LOG_LEVEL);

static const struct gpio_dt_spec rin1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(rin1), gpios, {0});
static const struct gpio_dt_spec fin1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(fin1), gpios, {0});
static const struct gpio_dt_spec rin2 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(rin2), gpios, {0});
static const struct gpio_dt_spec fin2 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(fin2), gpios, {0});
static const struct gpio_dt_spec sw8v = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw8v), gpios, {0});

/**
 * Initialize the solenoid controller. The I2C bust have been initialized before.
 */
enum solenoid_status solenoid_init(void)
{
    const struct device *dev = DEVICE_DT_GET_ANY(ti_tca9538);

    if (dev == NULL) {
        LOG_ERR("PCA9538 not found");
        return SOLENOID_NOT_DETECTED;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("pca9538 not ready");
        return SOLENOID_NOT_DETECTED;
    }
    LOG_DBG("Found device \"%s\"", dev->name);

    gpio_pin_configure_dt(&rin1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&fin1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&rin2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&fin2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&sw8v, GPIO_OUTPUT_INACTIVE);

    return SOLENOID_OK;
}

/**
 * Activate the specified solenoid in the forward position
 */
enum solenoid_status solenoid_activate_forward(int solenoid_nr)
{
    switch (solenoid_nr) {
        case 1:
            gpio_pin_set_dt(&rin1, 1);
            gpio_pin_set_dt(&fin1, 0);
            LOG_DBG("Valve 1 Forward");
            break;
        case 0:
            gpio_pin_set_dt(&rin2, 1);
            gpio_pin_set_dt(&fin2, 0);
            LOG_DBG("Valve 0 Forward");
            break;
        default:
            LOG_ERR("Not valid valve number.");
            break;
    }
    return SOLENOID_OK;
}

/**
 * Activate the specified solenoid in the reverse position
 * This is only used with bistable solenoids
 */
enum solenoid_status solenoid_activate_reverse(int solenoid_nr)
{
    switch (solenoid_nr) {
        case 1:
            gpio_pin_set_dt(&rin1, 0);
            gpio_pin_set_dt(&fin1, 1);
            LOG_DBG("Valve 1 Reverse");
            break;
        case 0:
            gpio_pin_set_dt(&rin2, 0);
            gpio_pin_set_dt(&fin2, 1);
            LOG_DBG("Valve 0 Reverse");
            break;
        default:
            LOG_ERR("Not valid valve number.");
            break;
    }
    return SOLENOID_OK;
}

/**
 * Release the solenoid. Disconnect all power from the solenoid.
 * This also disables the solenoid power supply.
 */
enum solenoid_status solenoid_release(void)
{
    gpio_pin_set_dt(&rin1, 0);
    gpio_pin_set_dt(&fin1, 0);
    gpio_pin_set_dt(&rin2, 0);
    gpio_pin_set_dt(&fin2, 0);
    gpio_pin_set_dt(&sw8v, 1); /* this pin is active low */
    LOG_DBG("Solenoid release all");
    return SOLENOID_OK;
}

/**
 * Enable the boost power supply for the solenoid
 * After powering the supply, one has to wait until the capacitor has been charged.
 */
enum solenoid_status solenoid_power_on(void)
{
    gpio_pin_set_dt(&sw8v, 0);
    LOG_DBG("Solenoid power ON");
    return SOLENOID_OK;
}

/**
 * Disable the boost power supply for the solenoid
 * TODO remove Should be enough with solenoid release
 */
enum solenoid_status solenoid_power_off(void)
{
    gpio_pin_set_dt(&sw8v, 1);
    LOG_DBG("Solenoid power OFF");
    return SOLENOID_OK;
}

/**
 * Prepare the solenoid.
 * First activate the solenoid power and check the voltage
 */
enum solenoid_status solenoid_prepare(void)
{
    int i;
    int adc;
    enum solenoid_status ret;

    adc_init();
    solenoid_power_on();
    for (i = 0; i < 20; i++) {
        watchdog_reset();
        sleep_microseconds(50000);
        adc = adc_read_solenoid_supply();
        LOG_DBG("Charging cap: %d", adc);
        if (adc > 7500) {
            /* Capacitor is charged */
            ret = SOLENOID_OK;
            break;
        }
    }
    if (adc_read_solenoid_supply() < 7200) {
        ret = SOLENOID_POWER_LOW;
    }
    LOG_DBG("Solenoid ADC: %i  i: %i", adc, i);
    return ret;
}
