#pragma once

#include <stdint.h>

/* Channel uplink for 500 kHz BW */
#define CHANNEL_UPLINK_64 903000000
#define CHANNEL_UPLINK_65 904600000
#define CHANNEL_UPLINK_66 906200000
#define CHANNEL_UPLINK_67 907800000
#define CHANNEL_UPLINK_68 909400000
#define CHANNEL_UPLINK_69 911000000
#define CHANNEL_UPLINK_70 912600000
#define CHANNEL_UPLINK_71 914200000

/* Channel downlink */
#define CHANNEL_DOWNLINK_0 923300000
#define CHANNEL_DOWNLINK_1 923900000
#define CHANNEL_DOWNLINK_2 924500000
#define CHANNEL_DOWNLINK_3 925100000
#define CHANNEL_DOWNLINK_4 925700000
#define CHANNEL_DOWNLINK_5 926300000
#define CHANNEL_DOWNLINK_6 926900000
#define CHANNEL_DOWNLINK_7 927500000

struct mac_address {
    uint8_t dev_id[16];
    uint8_t length;
};

int radio_init(void);
int radio_send_str(char *str, uint32_t len);
int send_frame(char *str, uint32_t len);
int radio_receive_str(char *str, uint32_t len, uint16_t time, char *name);
int end_device_get_link_quality(void);
int get_mac_address(struct mac_address *mac);
