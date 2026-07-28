/* See COPYING.txt for license details. */
/*
 * m1_qmon_relay.c — qMonstatek-over-WiFi bridge (M1 side).
 *
 * When the ESP has an active qMonstatek TCP client, this task pulls the client's
 * RPC frames off the ESP over m1_link, feeds them to the local RPC handler, and
 * routes the responses back out over TCP. It only talks to the ESP while WiFi is
 * actually up (no network -> no client -> no reason to poll), and is suspended
 * during an ESP flash so its 50ms poll can't contend with the ROM bootloader.
 */
#include "m1_qmon_relay.h"
#include "m1_esp_client.h"
#include "m1_rpc.h"
#include "m1_wifi.h"          /* m1_ble_direct — keep polling for a BLE client with WiFi down */
#include "FreeRTOS.h"
#include "task.h"

#define M1_QMON_RELAY_ENABLE 1

#if M1_QMON_RELAY_ENABLE

/* Set true around an ESP flash so the poll doesn't hit the shared SPI link while
 * the ESP is in its UART ROM bootloader. */
static volatile bool s_relay_suspend = false;

/* Last-seen desktop TCP client state (updated each poll). Exposed so an
 * M1-initiated WiFi connect can refuse to run while a desktop session is live. */
static volatile bool s_session_active = false;

/* m1_rpc response bytes -> forward to the ESP, which relays to the TCP client. */
static bool qmon_tcp_tx(const uint8_t *data, uint16_t len)
{
    return m1_esp_client_qmon_resp(data, len);
}

static void qmon_relay_task(void *arg)
{
    static uint8_t buf[1500];        /* one qMonstatek frame */
    bool routed = false;
    uint32_t last_wifi_poll = 0;
    (void)arg;

    for (;;) {
        if (s_relay_suspend) {           /* paused for an ESP flash */
            if (routed) { m1_rpc_route_to_tcp(false); routed = false; }
            s_session_active = false;
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        /* No WiFi => normally nothing can be connected over TCP, so don't poll the
         * ESP at the 50ms rate. EXCEPTION: with Bluetooth Direct on, a phone can be
         * connected over BLE with WiFi down — the ESP reports that client via
         * rpc_connected, so keep polling. Re-check WiFi status every 3s (cheap) so
         * we notice it coming up without a manual reconnect. */
        if (!m1_esp_client_wifi_is_connected() && !m1_ble_direct) {
            if (routed) { m1_rpc_route_to_tcp(false); routed = false; }
            s_session_active = false;
            uint32_t now = xTaskGetTickCount();
            if ((now - last_wifi_poll) >= pdMS_TO_TICKS(3000)) {
                last_wifi_poll = now;
                uint8_t c = 0;
                m1_esp_client_wifi_status(&c, NULL);   /* refreshes the WiFi flag */
            }
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        uint8_t connected = 0;
        if (!m1_esp_client_rpc_connected(&connected)) {
            /* Poll FAILED (SPI busy/contended — e.g. a device menu is also talking
             * to the ESP). Do NOT treat a failed poll as a disconnect: that would
             * wrongly tear down an active screen stream. Keep the current session
             * state and retry next tick. */
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        s_session_active = (connected != 0);

        /* Route m1_rpc responses to WiFi for the whole connected session, so
         * deferred/slow commands (file ops) also answer over the socket. */
        if (connected && !routed)      { m1_rpc_route_to_tcp(true);  routed = true; }
        else if (!connected && routed) {
            /* WiFi client just left — stop screen streaming so we don't keep
             * pushing frames into a dead socket, then drop TCP routing. */
            m1_rpc_stop_screen_stream();
            m1_rpc_route_to_tcp(false);
            routed = false;
        }

        if (connected) {
            int n = m1_esp_client_qmon_poll(buf, sizeof(buf));
            if (n > 0) {
                m1_rpc_feed_tcp(buf, (uint16_t)n);   /* TCP context; inline cmds respond via qmon_tcp_tx */
                continue;                         /* drain the queue quickly */
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif /* M1_QMON_RELAY_ENABLE */

void m1_qmon_relay_init(void)
{
#if M1_QMON_RELAY_ENABLE
    /* Create the relay task EXACTLY ONCE for the life of the program. This is
     * called from m1_esp32_init(), which re-runs every time a radio menu is
     * re-entered (m1_esp32_deinit clears esp32_init_done). Without this guard
     * every menu open→close leaked a permanent 8KB task hammering the ESP link
     * — heap exhaustion + link saturation after heavy use → M1 hang / flash
     * failure. Registering the tx callback again is harmless (just a fn ptr). */
    static bool s_relay_task_created = false;
    m1_rpc_register_tcp_tx(qmon_tcp_tx);
    if (!s_relay_task_created)
    {
        if (xTaskCreate(qmon_relay_task, "qmon_relay", 2048, NULL, 4, NULL) == pdPASS)
            s_relay_task_created = true;
    }
#endif
}

/* Defined unconditionally so callers (the ESP flash path) link whether or not the
 * relay is compiled in. */
void m1_qmon_relay_suspend(bool suspend)
{
#if M1_QMON_RELAY_ENABLE
    s_relay_suspend = suspend;
    if (suspend) s_session_active = false;
#else
    (void)suspend;
#endif
}

bool m1_qmon_relay_session_active(void)
{
#if M1_QMON_RELAY_ENABLE
    return s_session_active;
#else
    return false;
#endif
}
