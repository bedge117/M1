/* See COPYING.txt for license details. */

#ifndef M1_WIFI_RPC_H_
#define M1_WIFI_RPC_H_

#include <stdint.h>
#include <stdbool.h>
#include "m1_compile_cfg.h"

#if defined(M1_APP_RPC_ENABLE) && defined(M1_APP_WIFI_CONNECT_ENABLE)

#define WIFI_RPC_PORT           3333
#define WIFI_RPC_MDNS_HOST      "m1-device"
#define WIFI_RPC_MDNS_SERVICE   "_m1rpc"

void wifi_rpc_init(void);
bool wifi_rpc_start_server(void);
void wifi_rpc_stop_server(void);
bool wifi_rpc_server_running(void);
bool wifi_rpc_client_connected(void);
bool wifi_rpc_transmit(const uint8_t *data, uint16_t len);
void wifi_rpc_notify_wifi_connected(void);
bool wifi_rpc_spi_intercept(const uint8_t *data, uint16_t len);

#endif
#endif
