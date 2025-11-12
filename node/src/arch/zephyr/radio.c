
#include "radio.h"
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include "crc16.h"
#include "errorcodes.h"
#include "microio.h"
#include "configuration.h"
#include "watchdog.h"

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay), "No default LoRa radio specified in DT");

#define MAX_DATA_LEN 255
#define TRANSMITING  true
#define RECEIVING    false

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lora_radio, CONFIG_LORA_LOG_LEVEL);

static const struct device *lora_dev;
static int16_t rssi;

static int lora_configure(bool transmiting)
{
    struct lora_modem_config config;
    int ret;

    if (transmiting) { /* Transmitting */
        config.frequency = cfg.uplink_channel;
        config.tx = transmiting;
    } else { /* Receiving */
        config.frequency = cfg.downlink_channel;
        config.tx = transmiting;
    }
    config.bandwidth = cfg.bandwidth;
    config.datarate = cfg.datarate;
    config.preamble_len = 8;
    config.coding_rate = CR_4_5;
    config.tx_power = 20;
    config.iq_inverted = false;
    config.public_network = false;
    ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        LOG_ERR("LoRa config failed");
        return -E_INVALID;
    }
    return 0;
}

int radio_init(void)
{

    lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("%s Device not ready", lora_dev->name);
        return E_NOT_DETECTED;
    }
    rssi = 0;

    return 0;
}

int radio_send_str(char *str, uint32_t len)
{
    int ret;

    /* lora_send blocks 2times estimated air time. */
    printk("Send: %s\n", str);
    watchdog_disable();
    ret = lora_configure(TRANSMITING);
    if (ret < 0) {
        LOG_ERR("LoRa init failed");
        return -E_INVALID;
    }

    ret = lora_send(lora_dev, str, strlen(str));
    if (ret < 0) {
        LOG_ERR("LoRa send failed");
        return -E_TIMEDOUT;
    }
    watchdog_init();
    LOG_INF("LoRa data sent");
    ret = lora_configure(RECEIVING);
    if (ret < 0) {
        LOG_ERR("Lora failed\n");
        return -E_INVALID;
    }
    return 0;
}

int send_frame(char *str, uint32_t len)
{
    int ret;
    uint16_t crc = 0xFFFF;
    char *buffer;
    char payload[255];

    buffer = str;
    while (*buffer) {
        crc = crc16_update(crc, *buffer);
        buffer++;
    }
    usnprintf(payload, sizeof(payload), "%s %.4x", str, crc);
    /*lora_send blocks 2times estimated air time.*/
    ret = lora_configure(TRANSMITING);
    if (ret < 0) {
        LOG_ERR("LoRa init failed");
        printk("lora configure failed\n");
        return -E_INVALID;
    }
    watchdog_disable();
    ret = lora_send(lora_dev, payload, strlen(payload));
    if (ret < 0) {
        LOG_ERR("LoRa send failed");
        printk("Lora failed send\n");
        /* return -E_TIMEDOUT; */
    }
    watchdog_init();
    LOG_INF("LoRa data sent");
    ret = lora_configure(RECEIVING);
    if (ret < 0) {
        printk("Lora failed\n");
        return -E_INVALID;
    }
    return 0;
}

int radio_receive_str(char *str, uint32_t len, uint16_t time, char *name)
{
    int8_t snr;
    int ret = 0;
    uint8_t data[255] = {0};

    /* lora_send blocks 2times estimated air time. */

    watchdog_disable();
    ret = lora_configure(RECEIVING);
    if (ret < 0) {
        LOG_ERR("Lora failed\n");
        watchdog_init();
        return -E_INVALID;
    }
    ret = lora_recv(lora_dev, data, len, K_MSEC(time), &rssi, &snr);
    if (ret < 0) {
        LOG_ERR("LoRa received failed");
        watchdog_init();
        return -E_TIMEDOUT;
    }
    watchdog_init();
    LOG_INF("Received data: %s (RSSI:%ddBm, SNR:%ddBm)", data, rssi, snr);
    printk("Received %s\n", data);
    char buffer[strlen(data) + 1];

    strcpy(buffer, data);
    char *mote_name = strtok(buffer, " ");

    if (!strncmp(mote_name, name, 10)) { /* Valid data */
        memcpy(str, data, 255);
        memset(data, '\0', 255);
        ret = 1;
    } else { /* Invalid data. */
        ret = 0;
    }
    return ret;
}

int end_device_get_link_quality(void)
{
    uint16_t signal = 0;

    if (cfg.distance == 0) {
        signal = (rssi * 0.92) + 109.2;
    } else if (cfg.distance == 1) {
        signal = (rssi * 0.85) + 108.5;
    } else if (cfg.distance == 2) {
        signal = (rssi * 0.83) + 108.3;
    }
    return signal;
}

int get_mac_address(struct mac_address *mac)
{
    (void)memset(mac->dev_id, 0x0, sizeof(mac->dev_id));
    mac->length = hwinfo_get_device_id(mac->dev_id, sizeof(mac->dev_id));
    if (mac->length <= 0) {
        return -1;
    }

    return 0;
}
