/*
 * Communication with the smart sensors
 */

#include <string.h>
#include <zephyr/sys/printk.h>
#include "smart_sensor_protocol.h"
#include "errorcodes.h"
#include "crc16.h"

/*
 * Read a string of up to four bytes and convert the value to HEX
 */
uint16_t get_hex16(char *s)
{
    int i;
    char c;
    uint16_t r = 0;

    for (i = 0; i < 4; i++) {
        c = *(s + i);
        if (c == '\0') {
            break; /* End of line before 4 bytes of text */
        }
        r <<= 4;
        if (c >= '0' && c <= '9') {
            r |= (c - '0');
        }
        if (c >= 'A' && c <= 'F') {
            r |= (c - 'A' + 10);
        }
        if (c >= 'a' && c <= 'f') {
            r |= (c - 'a' + 10);
        }
    }
    return r;
}

/**
 * Check if a frame received from a smart sensor is valid. The frame must start with a colon
 * and end with a null and have no CR or NL at the end.
 * If frame is valid, return 0, negative otherwise
 */
int smart_sensor_check_frame(char *frame)
{
    uint16_t crc = 0xFFFF;
    uint16_t received_crc = 0;
    int len;
    int i;

    len = strlen(frame);
    if (len < 6) {
        return -E_INVALID;
    }

    if (frame[0] != ':') {
        return -E_INVALID;
    }
    /* Frame delimiters are OK, get the checksum */
    /* As the checksum is in ASCII, it can be 1 to 4 bytes long, find the last space before the checksum */
    int start_of_checksum;

    for (start_of_checksum = len - 1; start_of_checksum > (len - 6); start_of_checksum--) {
        if (frame[start_of_checksum - 1] == ' ') {
            break;
        }
    }
    received_crc = get_hex16(&(frame[start_of_checksum]));

    /* Calculate CRC of received data */
    for (i = 1; i < start_of_checksum - 1; i++) {
        crc = crc16_update(crc, frame[i]);
    }
    if (crc == received_crc) {
        return 0;
    }
    frame[strlen(frame) - 1] = '\0';
    received_crc = get_hex16(&(frame[start_of_checksum]));
    if (crc == received_crc) {
        return 0;
    } else {
        printk("frame: %s\n", frame);
        printk("%x != %x\n", crc, received_crc);
        return -E_BAD_CHECKSUM;
    }
}
