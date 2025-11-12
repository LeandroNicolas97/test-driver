
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>
#include "local_sensors.h"
#include "debug.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(local_sensors, CONFIG_SENSOR_LOG_LEVEL);

static const struct device *humidity_dev;

static const struct device *get_sht21_device(void)
{
    const struct device *dev = DEVICE_DT_GET_ANY(sensirion_sht21);

    if (dev == NULL) {
        LOG_ERR("sht21 dev not found");
        return NULL;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("sht21 dev not ready");
        return NULL;
    }
    LOG_DBG("Found device \"%s\"", dev->name);
    return dev;
}

int local_sensors_init(void)
{
    humidity_dev = get_sht21_device();
    if (humidity_dev == NULL) {
        return -1; /* return adecuate error code */
    }
    return 0;
}

int local_sensors_get_hum_and_temp(int16_t *h, float *t)
{
    int ret;
    struct sensor_value humidity, temperature;

    /* We need to prevent entering stanby, couse ic2 hang */
    /* TODO check in zephyr why i2c hangs in stanby so we dont need this lock */
    pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
    ret = sensor_sample_fetch(humidity_dev);
    pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
    if (ret < 0) {
        LOG_ERR("Error fetching sample (code:%d)", ret);
        return ret; /* return adecuate error code */
    }
    ret = sensor_channel_get(humidity_dev, SENSOR_CHAN_HUMIDITY, &humidity);
    if (ret < 0) {
        LOG_ERR("Error getting humidity channel (code:%d)", ret);
        return ret;
    }
    ret = sensor_channel_get(humidity_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
    if (ret < 0) {
        LOG_ERR("Error getting temperature channel (code:%d)", ret);
        return ret;
    }
    LOG_DBG("temp: %d.%06d; humidity: %d.%06d", temperature.val1, temperature.val2, humidity.val1, humidity.val2);
    *h = humidity.val1;
    *t = (float)temperature.val1 + (float)temperature.val2 / 1000000;

    return ret;
}
