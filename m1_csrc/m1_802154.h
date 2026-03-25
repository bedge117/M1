/*
 * m1_802154.h
 *
 * IEEE 802.15.4 (Zigbee/Thread) scanning for M1
 */

#ifndef M1_802154_H_
#define M1_802154_H_

#include <stdint.h>
#include <stdbool.h>

/* Protocol classification */
#define IEEE802154_PROTO_ZIGBEE  'Z'
#define IEEE802154_PROTO_THREAD  'T'
#define IEEE802154_PROTO_UNKNOWN 'U'

/* Max devices tracked per scan */
#define IEEE802154_MAX_DEVICES  64

/* Address string size (16 hex chars for EUI-64 + null) */
#define IEEE802154_ADDR_STR_SIZE  20
#define IEEE802154_PAN_STR_SIZE   6

typedef struct {
    char     proto;                                  /* Z, T, or U */
    char     src_pan[IEEE802154_PAN_STR_SIZE];       /* Source PAN ID hex string */
    char     src_addr[IEEE802154_ADDR_STR_SIZE];     /* Source address hex string */
    char     dst_pan[IEEE802154_PAN_STR_SIZE];       /* Destination PAN ID hex string */
    int8_t   rssi;                                   /* Best RSSI seen */
    uint8_t  lqi;                                    /* Best LQI seen */
    uint8_t  channel;                                /* Channel where seen */
    uint16_t frame_count;                            /* Number of frames from this device */
    char     frame_types[16];                        /* Seen frame types: "BCN,DATA" etc */
} ieee802154_device_t;

/* Menu entry functions */
void zigbee_scan(void);
void thread_scan(void);

#endif /* M1_802154_H_ */
