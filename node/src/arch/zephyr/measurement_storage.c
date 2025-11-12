
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "bsp-config.h"
#include "defaults.h"
#include "measurement_storage.h"
#include "watchdog.h"
#include "actual_conditions.h"

static struct nvs_fs fs;
static uint16_t unsended_data;

#define MEAS_PARTITION        measurement_partition
#define MEAS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(MEAS_PARTITION)
#define MEAS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(MEAS_PARTITION)
/* #define MEAS_FLASH_AREA_ID       FIXED_PARTITION_ID(MEAS_PARTITION) */

#define EXT_STOR_PART DT_NODELABEL(sst25vf032b)

#define EXT_MEAS_PARTITION        extstorage_partition
#define EXT_MEAS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(EXT_MEAS_PARTITION)
#define EXT_MEAS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(EXT_MEAS_PARTITION)
/* #define EXT_MEAS_FLASH_AREA_ID       FIXED_PARTITION_ID(EXT_MEAS_PARTITION) */

#define MEAS_ID          1
#define UNSENDED_DATA_ID 2

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(measurement_storage, CONFIG_NVS_LOG_LEVEL);

int measurement_storage_mount(void)
{
    int rc = 0;
    /* uint32_t sec_cnt; */
    struct flash_pages_info info;

    /* Check if external memory is available */
    if (device_is_ready(EXT_MEAS_PARTITION_DEVICE)) {
        printk("External memory detected\n");
        fs.flash_device = EXT_MEAS_PARTITION_DEVICE;
        fs.offset = EXT_MEAS_PARTITION_OFFSET;
        actual_state.using_external_memory = 1; /* Sets the variable indicating that external memory is used. */
    } else {
        printk("Internal memory detected\n");
        fs.flash_device = MEAS_PARTITION_DEVICE;
        fs.offset = MEAS_PARTITION_OFFSET;
        actual_state.using_external_memory = 0; /* Sets the variable indicating that internal memory is used. */
    }

    /* rc = flash_area_get_sectors(MEAS_FLASH_AREA_ID, &sec_cnt, NULL); */
    /* printk("Flash area sectors cnt: %d\n", sec_cnt); */
#if CONFIG_EXTERNAL_DATALOGGER
    fs.flash_device = MEAS_PARTITION_DEVICE;
    if (device_is_ready(fs.flash_device)) {
        printk("Internal memory detected\n");
        fs.offset = MEAS_PARTITION_OFFSET;
        rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
        if (rc) {
            LOG_ERR("Unable to get page info");
            return -1;
        }
        fs.sector_size = info.size;
        fs.sector_count = 64; /* sec_cnt; // TODO check this number */
    } else {
        LOG_ERR("Flash device %s is not ready", fs.flash_device->name);
        return -1;
    }
#else

#if DT_NODE_HAS_STATUS(EXT_STOR_PART, okay)
    if (device_is_ready(EXT_MEAS_PARTITION_DEVICE)) {
        fs.flash_device = EXT_MEAS_PARTITION_DEVICE;
        printk("Using external memory\n");
        fs.offset = EXT_MEAS_PARTITION_OFFSET;
        rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
        if (rc) {
            LOG_ERR("Unable to get page info");
            return -1;
        }
        fs.sector_size = info.size;
        fs.sector_count = 770U; /* sec_cnt; // TODO check this number */
    } else if (device_is_ready(MEAS_PARTITION_DEVICE)) {
        /* we dont find the external flash, so we use the internal partition */
        fs.flash_device = MEAS_PARTITION_DEVICE;
        printk("Using internal memory\n");
        fs.offset = MEAS_PARTITION_OFFSET;
        rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
        if (rc) {
            LOG_ERR("Unable to get page info");
            return -1;
        }
        fs.sector_size = info.size;
        fs.sector_count = 64; /* sec_cnt; // TODO check this number */
    } else {
        LOG_ERR("Flash device is not ready");
        return -1;
    }
#else
    if (device_is_ready(MEAS_PARTITION_DEVICE)) {
        /* we dont find the external flash, so we use the internal partition */
        fs.flash_device = MEAS_PARTITION_DEVICE;
        printk("Internal memory detected\n");
        fs.offset = MEAS_PARTITION_OFFSET;
        rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
        if (rc) {
            LOG_ERR("Unable to get page info");
            return -1;
        }
        fs.sector_size = info.size;
        fs.sector_count = 64; /* sec_cnt; // TODO check this number */
    } else {
        LOG_ERR("Flash device is not ready");
        return -1;
    }
#endif
#endif
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
    unsended_data = unsended_data_get();
    return 0;
}

uint16_t unsended_data_get(void)
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

void unsended_data_flush_last(void)
{
    unsended_data--;
    watchdog_disable();
    (void)nvs_write(&fs, UNSENDED_DATA_ID, &unsended_data, sizeof(unsended_data));
    watchdog_init();
}
/*
 *
 */
int measurement_storage_append(uint8_t *meas_data, size_t size)
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

    unsended_data++;
    rc = measurement_storage_get(test_data, size, unsended_data);
    while (rc == -1) {
        LOG_WRN("Unsended data is more than max.\n");
        unsended_data--;
        rc = measurement_storage_get(test_data, size, unsended_data);
    }
    (void)nvs_write(&fs, UNSENDED_DATA_ID, &unsended_data, sizeof(unsended_data));
    watchdog_init();
    LOG_INF("measurement written OK\n");
    return 0;
}

uint32_t get_free_space(void)
{
    watchdog_disable();
    uint32_t res = nvs_calc_free_space(&fs);

    watchdog_init();
    return res;
}

void measurement_storage_format(void)
{
    unsended_data = 0;
    watchdog_disable();
    (void)nvs_clear(&fs);
    watchdog_init();
}
/*
 *
 */
int measurement_storage_get(uint8_t *meas_data, size_t size, uint16_t n_from_last)
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
