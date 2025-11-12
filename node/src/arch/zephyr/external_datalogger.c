
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "bsp-config.h"
#include "defaults.h"
#include "external_datalogger.h"
#include "watchdog.h"

static struct nvs_fs fs;
static uint16_t unsended_data;

#define MEAS_PARTITION        extstorage_partition
#define MEAS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(MEAS_PARTITION)
#define MEAS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(MEAS_PARTITION)
#define MEAS_FLASH_AREA_ID    FIXED_PARTITION_ID(MEAS_PARTITION)

#define MEAS_ID          1
#define UNSENDED_DATA_ID 2

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(datalogger, CONFIG_NVS_LOG_LEVEL);

int datalogger_mount(void)
{
    int rc = 0;
    uint32_t sec_cnt;
    struct flash_pages_info info;

    rc = flash_area_get_sectors(MEAS_FLASH_AREA_ID, &sec_cnt, NULL);
    fs.flash_device = MEAS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("Flash device %s is not ready", fs.flash_device->name);
        return -1;
    }
    fs.offset = MEAS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        LOG_ERR("Unable to get page info");
        return -1;
    }
    fs.sector_size = info.size;
    fs.sector_count = 770U; /* sec_cnt; // TODO check this number */
    printk("Flash Sector size : %d\n", info.size);
    printk("Flash Sector count : %d\n", fs.sector_count);
    printk("Flash Start offset : %li\n", info.start_offset);
    printk("flash_pages_index : %d\n", info.index);

    watchdog_disable();
    rc = nvs_mount(&fs);
    watchdog_init();
    if (rc) {
        LOG_ERR("Flash Init failed");
        return -1;
    }
    unsended_data = datalogger_unsended_data_get();
    return 0;
}

uint16_t datalogger_unsended_data_get(void)
{
    int rc;

    watchdog_disable();
    rc = nvs_read(&fs, UNSENDED_DATA_ID, &unsended_data, sizeof(unsended_data));
    if (rc <= 0) {
        /* not founded */
        unsended_data = 0;
        (void)nvs_write(&fs, UNSENDED_DATA_ID, &unsended_data, sizeof(unsended_data));
    }
    watchdog_init();
    return unsended_data;
}

void datalogger_unsended_data_flush_last(void)
{
    unsended_data--;
    watchdog_disable();
    (void)nvs_write(&fs, UNSENDED_DATA_ID, &unsended_data, sizeof(unsended_data));
    watchdog_init();
}
/*
 *
 */
int datalogger_append(uint8_t *meas_data, size_t size)
{
    int rc = 0;

    watchdog_disable();
    rc = nvs_write(&fs, MEAS_ID, meas_data, size);
    if (rc < 0) {
        LOG_ERR("Error writing the measurement to the external flash");
        return -1;
    }
    /* check to see if unsended_data is more than the max. */
    uint8_t test_data[size];

    rc = datalogger_get(test_data, size, unsended_data);
    if (rc == -1) {
        LOG_WRN("Unsended data is more than max.\n");
        unsended_data--;
    }
    unsended_data++;
    (void)nvs_write(&fs, UNSENDED_DATA_ID, &unsended_data, sizeof(unsended_data));
    watchdog_init();
    LOG_INF("measurement written OK\n");
    printk("measurement written OK\n");
    return 0;
}

uint32_t datalogger_get_free_space(void)
{
    watchdog_disable();
    uint32_t res = nvs_calc_free_space(&fs);

    watchdog_init();
    return res;
}

void datalogger_format(void)
{
    unsended_data = 0;
    watchdog_disable();
    (void)nvs_clear(&fs);
    watchdog_init();
}
/*
 *
 */
int datalogger_get(uint8_t *meas_data, size_t size, uint16_t n_from_last)
{
    int rc;

    watchdog_disable();
    rc = nvs_read_hist(&fs, MEAS_ID, meas_data, size, n_from_last);
    watchdog_init();
    if (rc < 0) {
        LOG_WRN("No more data");
        return -1;
    }
    return 0;
}
