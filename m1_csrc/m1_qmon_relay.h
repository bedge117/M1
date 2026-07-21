/* See COPYING.txt for license details. */
/*
 * m1_qmon_relay.h — bridges qMonstatek-over-WiFi to the on-device qMonstatek
 * protocol handler (m1_rpc.c). Registers the m1_rpc TCP-tx route so responses
 * go back over WiFi, and runs a task that pulls queued desktop command frames
 * from the ESP (M1ESP_QMON_POLL) and feeds them to m1_rpc_feed(). Routing to
 * TCP is enabled for the whole session while a desktop TCP client is connected
 * (so both inline and deferred/slow commands answer over WiFi).
 */
#ifndef M1_QMON_RELAY_H_
#define M1_QMON_RELAY_H_

#include <stdbool.h>

void m1_qmon_relay_init(void);

/* Pause/resume the relay's ESP poll. Called around an ESP flash so the poll
 * doesn't contend with the ROM bootloader on the shared SPI link. Safe to call
 * even when the relay is compiled out (no-op). */
void m1_qmon_relay_suspend(bool suspend);

/* True while a qMonstatek desktop TCP client is connected over WiFi (the relay's
 * last-seen link state). Callers use it to refuse an M1-initiated WiFi re-connect
 * that would tear down the STA link the desktop session is riding on. Returns
 * false when the relay is compiled out. */
bool m1_qmon_relay_session_active(void);

#endif /* M1_QMON_RELAY_H_ */
