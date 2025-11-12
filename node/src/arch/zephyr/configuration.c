
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/drivers/lora.h>
#include "configuration.h"
#include "bsp-config.h"
#include "defaults.h"
#include "radio.h"
#include "smart_sensor.h"

struct configuration cfg;
struct sensor_config sen_drv;
static struct nvs_fs fs;
static uint32_t _time_offset; /* zero-initialized by C */

#define NVS_PARTITION        storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define CONFIG_ID         1
#define SENSOR_DRIVERS_ID 2

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(configuration, CONFIG_NVS_LOG_LEVEL);

int read_nvs_data(void)
{
    int rc = 0;
    struct flash_pages_info info;

    /* this bellow init the nvs, maybe we can make a init function. */
    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("Flash device %s is not ready", fs.flash_device->name);
        return -1;
    }
    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        LOG_ERR("Unable to get page info");
        return -1;
    }
    fs.sector_size = info.size;
    fs.sector_count = 32U; /* In accordance with the dts */
    LOG_DBG("Flash Sector size : %d", info.size);
    LOG_DBG("Flash Start offset : %li", info.start_offset);
    LOG_DBG("flash_pages_index : %d", info.index);

    rc = nvs_mount(&fs);
    if (rc) {
        LOG_ERR("Flash Init failed");
        return -1;
    }
    /* ends init */

    rc = nvs_read(&fs, CONFIG_ID, (uint8_t *)&cfg, sizeof(cfg));
    if (rc > 0) {
        /* founded! */
        LOG_DBG("Configuration founded in nvs");
    } else {
        /* not founded */
        LOG_WRN("Configuration NOT founded in nvs");
        cfg.n_changes = -1;
    }

    rc = nvs_read(&fs, SENSOR_DRIVERS_ID, (uint8_t *)&sen_drv, sizeof(sen_drv));
    if (rc > 0) {
        /* founded! */
        LOG_DBG("Sensor drivers founded in nvs");
    } else {
        /* not founded */
        LOG_WRN("Sensor drivers NOT founded in nvs");
        sen_drv.n_changes = -1;
        return -1;
    }
    return 0;
}

/*
 * Save the configuration to persistent storage. Increment the configuration changed counter.
 */
int write_configuration(void)
{
    cfg.n_changes++;
    int rc = 0;

    rc = nvs_write(&fs, CONFIG_ID, (uint8_t *)&cfg, sizeof(cfg));
    if (rc < 0) {
        LOG_ERR("Error writing the configuration to the nvs");
        return -1;
    }

    LOG_INF("Configuration written OK\n");
    return 0;
}

int write_sensor_configuration(void)
{
    sen_drv.n_changes++;
    int rc = 0;

    rc = nvs_write(&fs, SENSOR_DRIVERS_ID, (uint8_t *)&sen_drv, sizeof(sen_drv));
    if (rc < 0) {
        LOG_ERR("Error writing the configuration to the nvs");
        return -1;
    }

    LOG_INF("Configuration written OK\n");
    return 0;
}

/**
 * Set default values for all the configuration parameters.
 */
void set_default_configuration(void)
{
    cfg.n_changes = 0;
    cfg.sampling_interval = DEFAULT_SAMPLING_INTERVAL;
    cfg.wake_interval = 1; /* DEFAULT_WAKE_INTERVAL; */
    cfg.log_interval = DEFAULT_LOG_INTERVAL;
    cfg.ping_interval = 60;
    cfg.salinity = DEFAULT_SALINITY;
    cfg.fixed_temperature = DEFAULT_FIXED_TEMPERATURE;
    cfg.v_ref = 1.2139; /* The real value is about 1.2V, it must be calibrated */
    cfg.battery_coefficient = 0.6666666666666;
    cfg.low_oxygen_alarm = DEFAULT_LOW_OXYGEN_ALARM_LEVEL;
    cfg.high_oxygen_alarm = DEFAULT_HIGH_OXYGEN_ALARM_LEVEL;
    /*    cfg.sensor_powerup_time = DEFAULT_SENSOR_POWER_UP_TIME; */
    cfg.sensor_communication_tries = 10;
    cfg.channel = 0;
    cfg.uplink_channel = CHANNEL_UPLINK_64;
    cfg.downlink_channel = CHANNEL_DOWNLINK_0;
    cfg.lcd_contrast = DEFAULT_LCD_CONTRAST;
    cfg.use_saturation = 0;
    cfg.conductivity_freshwater = FRESHWATER;
    cfg.command_state = 0;
    cfg.distance = 0;
    cfg.bandwidth = BW_500_KHZ;
    cfg.datarate = SF_7;
    cfg.time_on_air = 33;
    strcpy(cfg.name, "1");
    cfg.temp_offset = 0.0;
    cfg.current_sensor_status = 0;
    cfg.totalized_flow = 0;
    cfg.total_volume = 1000;
}

void set_driver_default(void)
{
    sen_drv.n_changes = 0;
    sen_drv.sensor_driver[CHEMINS] = NULL;
    sen_drv.sensor_driver[NORTEK] = NULL;
    sen_drv.sensor_driver[LUFFT] = NULL;
    sen_drv.sensor_driver[VAISALA] = NULL;
    sen_drv.sensor_driver[INNOVEX] = &smart_sensor_driver_innovex;
    sen_drv.sensor_driver[MAXBOTIX] = NULL;
    sen_drv.sensor_driver[PONSEL] = NULL;
    sen_drv.sensor_driver[TEXAS_INSTRUMENTS] = NULL;
    sen_drv.sensor_driver[YOSEMITECH] = NULL;
    sen_drv.sensor_driver[AQUAS] = NULL;
    sen_drv.sensor_driver[YSI] = NULL;
    sen_drv.sensor_driver[HUIZHONG] = NULL;
    sen_drv.sensor_driver[TELEDYNE_ISCO] = NULL;
    sen_drv.sensor_driver[ANBSENSORS] = NULL;
    sen_drv.sensor_driver[TDS100] = NULL;
    sen_drv.sensor_driver[GPS] = NULL;
    sen_drv.sensor_driver[CHEMINS] = NULL;
    sen_drv.sensor_driver[SEABIRD] = NULL;
    sen_drv.sensor_driver[JIANGSU] = NULL;
    sen_drv.sensor_driver[ACCONEER] = NULL;
    sen_drv.sensor_driver[AQUADOPP] = NULL;
    sen_drv.sensor_driver[FLOWQUEST] = NULL;
    sen_drv.sensor_driver[WITMOTION] = NULL;
}

void set_current_time(uint32_t *time)
{
    _time_offset = *time - (k_uptime_get() / MSEC_PER_SEC);
}

uint32_t get_current_time(void)
{
    return (k_uptime_get() / MSEC_PER_SEC) + _time_offset;
}

uint32_t get_timestamp(char *data)
{
    char *buffer = strrchr(data, ' ');
    uint32_t timestamp = atol(buffer);
    return timestamp;
}
