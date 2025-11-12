#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <stdint.h>
#include "adc.h"
#include "bsp-config.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc, CONFIG_ADC_LOG_LEVEL);

/**
 * Check if the required nodes are present in the Devicetree
 */
#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

/**
 * Checks if the necessary nodes exist in the devicetree
 * Obtains the configuration of a specific ADC channel from a Devicetree node in Zephyr,
 * facilitating iteration over multiple ADC channels
 */
#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/**
 * Configuration of the ADC channels using devicetree
 */
static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

#define ADC_NUM_CHANNELS ARRAY_SIZE(adc_channels)
#define ADC_NODE         DT_PHANDLE(DT_PATH(zephyr_user), io_channels)

/**
 * Configures and reads a specific channel from the ADC, storing the result in a buffer.
 * @param adc_channel ADC channel specification.
 * @param sample_buffer Buffer to store the value read.
 * @param resolution ADC resolution.
 * @return Status of the read operation.
 */
int read_adc_channel(const struct adc_dt_spec *adc_channel, int32_t *sample_buffer, uint8_t resolution)
{
    struct adc_sequence sequence = {
        .resolution = resolution,
        .buffer = sample_buffer,
        .buffer_size = sizeof(*sample_buffer),
        .channels = BIT(adc_channel->channel_id),
    };

    return adc_read(adc_channel->dev, &sequence);
}

static const struct device *dev_adc;
/**
 * Initialize the analog to digital converter
 */
void adc_init(void)
{
    dev_adc = DEVICE_DT_GET(ADC_NODE);

    if (dev_adc == NULL) {
        LOG_ERR("ADC device not found");
        return;
    }
    if (!device_is_ready(dev_adc)) {
        LOG_ERR("ADC device not ready\n");
        return;
    }
}

/**
 * Read the supply voltage of the solenoid in mV.
 * The solenoid supply is passed trough a 5.6M/560K voltage divider first,
 * dividing by 11, this goes to a follower and then to the ADC2.
 * We have to multiply by 11 to have mV.
 */
int adc_read_solenoid_supply(void)
{
    int32_t sample_buffer = 0;
    /* Solenoid channel configuration */
    int err = adc_channel_setup_dt(&adc_channels[0]);

    if (err < 0) {
        printk("Error configuring the Solenoid channel: %d\n", err);
        return -1;
    }
    /* Read from the Solenoid channel */
    err = read_adc_channel(&adc_channels[0], &sample_buffer, adc_channels[0].resolution);
    if (err < 0) {
        printk("Error reading Solenoid channel: %d\n", err);
        return -1;
    }
    /* Calculate the voltage value */
    int mv_value = (sample_buffer * 1650 * 2) / (1 << adc_channels[0].resolution);

    LOG_DBG("ADC ch Solenoid raw: %d = %dmV x 11 = %dmV", sample_buffer, mv_value, mv_value * 11);
    return mv_value * 11;
}

/**
 * Read the voltage of the battery in mV.
 */
int adc_read_battery(void)
{
    int32_t sample_buffer = 0;
    /* Battery Channel configuration  */
    int err = adc_channel_setup_dt(&adc_channels[1]);

    if (err < 0) {
        printk("Error configuring the Battery channel: %d\n", err);
        return -1;
    }
    /* Read from the Battery Channel */
    err = read_adc_channel(&adc_channels[1], &sample_buffer, adc_channels[1].resolution);
    if (err < 0) {
        printk("Error reading the Battery channel: %d\n", err);
        return -1;
    }
    /* Calculate the voltage value in mV */
    int mv_value = (sample_buffer * 1650 * 2) / (1 << adc_channels[1].resolution);

    LOG_DBG("ADC ch Battery raw: %d = %dmV x 2 = %dmV", sample_buffer, mv_value, mv_value * 2);
    return mv_value * 2;
}

/**
 * Read the supply voltage of the sensor in mV.
 * The Vcc of the external sensor is passed trough a 1/2 divider first and this voltage
 * is passed to the ADC.
 */
int adc_read_sensor_supply(void)
{
    int32_t sample_buffer = 0;
    /* Sensor Supply Channel configuration */
    int err = adc_channel_setup_dt(&adc_channels[2]);

    if (err < 0) {
        printk("Error configuring the Sensor channel: %d\n", err);
        return -1;
    }
    /* Read from the Sensor Supply Channel */
    err = read_adc_channel(&adc_channels[2], &sample_buffer, adc_channels[2].resolution);
    if (err < 0) {
        printk("Error reading the Sensor channel: %d\n", err);
        return -1;
    }
    /* Calculate the voltage value in mV */
    int mv_value = (sample_buffer * 1650 * 2) / (1 << adc_channels[2].resolution);

    LOG_DBG("ADC ch Sensor Supply raw: %d = %dmV x 2 = %dmV", sample_buffer, mv_value, mv_value * 2);
    return mv_value * 2;
}
