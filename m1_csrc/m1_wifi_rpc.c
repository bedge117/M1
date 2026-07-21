/* See COPYING.txt for license details. */

/*
 * m1_wifi_rpc.c
 *
 * WiFi RPC Bridge — TCP server over ESP32 AT commands.
 *
 * Architecture (active mode + safe intercept):
 *   - Active +IPD mode (default). ESP32 pushes +IPD with inline data.
 *   - SPI intercept catches +IPD only when it's the sole content of
 *     the SPI buffer (offset <= 4 from start). When +IPD is batched
 *     with AT responses, the intercept does NOT consume — keeping the
 *     AT pipeline clean at the cost of losing that +IPD (qMonstatek
 *     retries). CONNECT/CLOSED events always intercepted.
 *   - On new connection, a 3-second settle delay lets initial +IPD
 *     events arrive while no AT commands are in flight (clean SPI bus).
 *   - wifi_rpc_task reads from stream buffer and feeds RPC parser.
 *   - Responses sent via AT+CIPSEND from task context.
 *
 * M1 Project
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "app_freertos.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "main.h"
#include "m1_compile_cfg.h"
#include "m1_esp_rpc_cfg.h"   /* M1_USE_ESP_RPC — Option C gates the legacy AT task */

#if defined(M1_APP_RPC_ENABLE) && defined(M1_APP_WIFI_CONNECT_ENABLE)

#include "m1_wifi_rpc.h"

/* -------------------------------------------------------------------------
 * Option C: qMonstatek-over-WiFi is served natively by m1_qmon_relay over the
 * m1_link transport. This legacy AT-command TCP bridge (AT+CIPSERVER / AT+MDNS
 * / AT+CIPSEND) is NOT built — under Option C its AT transport is left
 * uninitialized and its framing would collide with m1_link. These inert stubs
 * keep the shared call sites (wifi_rpc_init / _notify_wifi_connected /
 * _spi_intercept) linking without pulling in any AT commands.
 * ------------------------------------------------------------------------- */
void wifi_rpc_init(void)                                  {}
bool wifi_rpc_start_server(void)                          { return false; }
void wifi_rpc_stop_server(void)                           {}
bool wifi_rpc_server_running(void)                        { return false; }
bool wifi_rpc_client_connected(void)                      { return false; }
bool wifi_rpc_transmit(const uint8_t *d, uint16_t n)      { (void)d; (void)n; return false; }
void wifi_rpc_notify_wifi_connected(void)                 {}
bool wifi_rpc_spi_intercept(const uint8_t *d, uint16_t n) { (void)d; (void)n; return false; }


#endif /* M1_APP_RPC_ENABLE && M1_APP_WIFI_CONNECT_ENABLE */
