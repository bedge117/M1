/* See COPYING.txt for license details. */

/*
*
* game_peer_ttt.c
*
* 2-player Tic-Tac-Toe over the M1<->M1 ESP-NOW peer link.
*
* A live demo of the peer link for developers: real-time bidirectional M1<->M1
* messaging with almost no code. Pairing reuses the peer-link discovery
* (announce + get_peers); gameplay is turn-based and loss-tolerant — every move
* broadcasts the FULL board state (12 bytes), so a dropped packet self-heals on
* the next one. Two devices, each running this, find each other and play.
*
* Wire format (inside an ESP-NOW DATA payload): ['G'][subtype][...]. The 'G'
* magic keeps it clear of the peer-link file-transfer tags (0..4).
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_system.h"
#include "m1_display.h"
#include "m1_esp_client.h"
#include "m1_games.h"

/*************************** D E F I N E S ************************************/

#define TTT_CHANNEL     1
#define TTT_MAX_PEERS   8

#define GM_MAGIC        'G'
enum { GM_INVITE = 1, GM_ACCEPT = 2, GM_STATE = 3, GM_QUIT = 4 };

enum { C_EMPTY = 0, C_X = 1, C_O = 2 };                 /* cell value */
enum { ST_PLAY = 0, ST_XWIN = 1, ST_OWIN = 2, ST_DRAW = 3 }; /* game status */

/***************************** V A R I A B L E S ******************************/

/* Buffered ESP RX so we can hand back one game message at a time. Single peer-
 * link task, so a file-scope cursor is safe (not reentrant). */
static uint8_t s_buf[1024];
static int     s_len, s_count, s_idx, s_off;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static bool    ttt_next(uint8_t from[6], uint8_t *sub, const uint8_t **body, int *blen);
static void    ttt_send_state(const uint8_t *mac, const uint8_t *board,
                              uint8_t turn, uint8_t mc, uint8_t status);
static uint8_t ttt_status(const uint8_t *b);
static int     ttt_mode_select(void);
static bool    ttt_lobby(const uint8_t my_mac[6], uint8_t mac[6], bool *is_x);
static void    ttt_play(const uint8_t *mac, bool is_x);
static void    ttt_local(void);
static void    ttt_draw(const uint8_t *board, int cursor, bool show_cursor,
                        const char *top, const char *mid, const char *result);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/* Return the next buffered game ('G') message, draining the ESP as needed.
 * true if one was returned; false when nothing is pending. */
static bool ttt_next(uint8_t from[6], uint8_t *sub, const uint8_t **body, int *blen)
{
    for (;;) {
        if (s_idx >= s_count) {
            s_len = m1_esp_client_now_recv(s_buf, sizeof(s_buf));
            if (s_len < 1 || s_buf[0] == 0) { s_count = 0; return false; }
            s_count = s_buf[0]; s_idx = 0; s_off = 1;
        }
        while (s_idx < s_count) {
            if (s_off + 8 > s_len) { s_idx = s_count; break; }
            uint8_t fm[6];
            memcpy(fm, &s_buf[s_off], 6); s_off += 6;
            int len = s_buf[s_off] | (s_buf[s_off + 1] << 8); s_off += 2;
            if (s_off + len > s_len) { s_idx = s_count; break; }
            const uint8_t *p = &s_buf[s_off];
            s_off += len; s_idx++;
            if (len >= 2 && p[0] == GM_MAGIC) {
                memcpy(from, fm, 6);
                *sub  = p[1];
                *body = p + 2;
                *blen = len - 2;
                return true;
            }
            /* not a game message — skip it */
        }
    }
}

/* Broadcast the full board state to the opponent. Sent a few times because a
 * single ESP-NOW frame can be lost; the move counter makes repeats idempotent. */
static void ttt_send_state(const uint8_t *mac, const uint8_t *board,
                           uint8_t turn, uint8_t mc, uint8_t status)
{
    uint8_t m[2 + 12];
    m[0] = GM_MAGIC; m[1] = GM_STATE;
    memcpy(&m[2], board, 9);
    m[11] = turn;
    m[12] = mc;
    m[13] = status;
    for (int i = 0; i < 3; i++) { m1_esp_client_now_send(mac, m, sizeof(m)); HAL_Delay(8); }
}

/* Win/draw check over the 8 lines. */
static uint8_t ttt_status(const uint8_t *b)
{
    static const uint8_t L[8][3] = {
        {0,1,2},{3,4,5},{6,7,8}, {0,3,6},{1,4,7},{2,5,8}, {0,4,8},{2,4,6}
    };
    for (int i = 0; i < 8; i++) {
        uint8_t a = b[L[i][0]];
        if (a != C_EMPTY && a == b[L[i][1]] && a == b[L[i][2]])
            return (a == C_X) ? ST_XWIN : ST_OWIN;
    }
    for (int i = 0; i < 9; i++) if (b[i] == C_EMPTY) return ST_PLAY;
    return ST_DRAW;
}

/* ---- lobby: discover + pair ---- */

/* Fills mac[6] and *is_x when paired; returns false if the user backed out.
 * Inviter (presses OK on a peer) plays X and moves first; invitee plays O.
 * If both invite at once, the lower MAC keeps X (deterministic tiebreak). */
static bool ttt_lobby(const uint8_t my_mac[6], uint8_t mac[6], bool *is_x)
{
    m1_now_peer_t peers[TTT_MAX_PEERS];
    int      npeers = 0, sel = 0;
    uint32_t last_poll = 0;

    while (1) {
        uint32_t now = HAL_GetTick();

        /* Someone invited us -> accept and play O. */
        uint8_t from[6], sub; const uint8_t *body; int blen;
        while (ttt_next(from, &sub, &body, &blen)) {
            if (sub == GM_INVITE) {
                uint8_t ack[2] = { GM_MAGIC, GM_ACCEPT };
                m1_esp_client_now_send(from, ack, sizeof(ack));
                memcpy(mac, from, 6);
                *is_x = false;
                return true;
            }
        }

        if ((now - last_poll) >= 500) {
            last_poll = now;
            m1_esp_client_now_announce();
            int n = m1_esp_client_now_get_peers(peers, TTT_MAX_PEERS);
            npeers = (n > 0) ? n : 0;
            if (sel >= npeers) sel = npeers ? npeers - 1 : 0;
        }

        m1_u8g2_firstpage();
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 0, 8, "TicTacToe: pick foe");
            char hdr[24];
            snprintf(hdr, sizeof(hdr), "Peers: %d", npeers);
            u8g2_DrawStr(&m1_u8g2, 0, 18, hdr);
            int y = 28;
            for (int i = 0; i < npeers && i < 3; i++) {
                char line[28];
                snprintf(line, sizeof(line), "%c%s %ddB",
                         (i == sel) ? '>' : ' ', peers[i].name, peers[i].rssi);
                u8g2_DrawStr(&m1_u8g2, 0, y, line);
                y += 10;
            }
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_DrawBox(&m1_u8g2, 0, 52, 128, 12);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            u8g2_DrawStr(&m1_u8g2, 4, 61, "OK:Invite");
            u8g2_DrawStr(&m1_u8g2, 96, 61, "Back");
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        } while (m1_u8g2_nextpage());

        S_M1_Main_Q_t q; S_M1_Buttons_Status b;
        if (xQueueReceive(main_q_hdl, &q, pdMS_TO_TICKS(80)) == pdTRUE
            && q.q_evt_type == Q_EVENT_KEYPAD) {
            xQueueReceive(button_events_q_hdl, &b, 0);
            if (b.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK) {
                return false;
            } else if (b.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK) {
                if (sel > 0) sel--;
            } else if (b.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK) {
                if (sel < npeers - 1) sel++;
            } else if (b.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK && npeers > 0) {
                /* Invite the selected peer; wait ~2.5s for an ACCEPT -> we're X. */
                memcpy(mac, peers[sel].mac, 6);
                uint8_t inv[2] = { GM_MAGIC, GM_INVITE };
                uint32_t t0 = HAL_GetTick();
                bool accepted = false;
                while (!accepted && (HAL_GetTick() - t0) < 2500) {
                    m1_esp_client_now_send(mac, inv, sizeof(inv));
                    uint32_t w = HAL_GetTick();
                    while (!accepted && (HAL_GetTick() - w) < 350) {
                        uint8_t fm[6], sb; const uint8_t *bd; int bl;
                        while (ttt_next(fm, &sb, &bd, &bl)) {
                            if (sb == GM_ACCEPT && memcmp(fm, mac, 6) == 0) {
                                accepted = true;
                            } else if (sb == GM_INVITE && memcmp(fm, mac, 6) == 0
                                       && memcmp(my_mac, fm, 6) > 0) {
                                /* Both invited each other: the higher MAC yields
                                 * and plays O. */
                                m1_esp_client_now_send(fm, (uint8_t[]){ GM_MAGIC, GM_ACCEPT }, 2);
                                *is_x = false;
                                return true;
                            }
                        }
                        HAL_Delay(10);
                    }
                }
                if (accepted) { *is_x = true; return true; }
                m1_message_box(&m1_u8g2, "TicTacToe", "No answer", peers[sel].name, " OK ");
            }
        }
    }
}

/* ---- board rendering ---- */

/* Presentation only. Grid on the left; a text column on the right shows `top`
 * / `mid` during play and `result` + a rematch hint when the game is over. */
static void ttt_draw(const uint8_t *board, int cursor, bool show_cursor,
                     const char *top, const char *mid, const char *result)
{
    const int ox = 2, oy = 8, cs = 16;    /* grid origin + cell size (48x48) */
    const int tx = 56;                    /* text column x */

    m1_u8g2_firstpage();
    do {
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);

        /* grid */
        for (int i = 0; i <= 3; i++) {
            u8g2_DrawHLine(&m1_u8g2, ox, oy + i * cs, cs * 3);
            u8g2_DrawVLine(&m1_u8g2, ox + i * cs, oy, cs * 3);
        }
        for (int i = 0; i < 9; i++) {
            int cx = ox + (i % 3) * cs, cy = oy + (i / 3) * cs;
            if (board[i] == C_X) {
                u8g2_DrawLine(&m1_u8g2, cx + 3, cy + 3, cx + cs - 3, cy + cs - 3);
                u8g2_DrawLine(&m1_u8g2, cx + cs - 3, cy + 3, cx + 3, cy + cs - 3);
            } else if (board[i] == C_O) {
                u8g2_DrawCircle(&m1_u8g2, cx + cs / 2, cy + cs / 2, cs / 2 - 3, U8G2_DRAW_ALL);
            }
        }
        if (show_cursor) {
            int cx = ox + (cursor % 3) * cs, cy = oy + (cursor / 3) * cs;
            u8g2_DrawFrame(&m1_u8g2, cx + 1, cy + 1, cs - 1, cs - 1);
        }

        /* text column */
        if (top)    u8g2_DrawStr(&m1_u8g2, tx, 12, top);
        if (mid)    u8g2_DrawStr(&m1_u8g2, tx, 26, mid);
        if (result) {
            u8g2_DrawStr(&m1_u8g2, tx, 46, result);
            u8g2_DrawStr(&m1_u8g2, tx, 60, "OK:again");
        }
    } while (m1_u8g2_nextpage());
}

/* ---- gameplay ---- */

static void ttt_play(const uint8_t *mac, bool is_x)
{
    uint8_t board[9]   = {0};
    uint8_t turn       = C_X;       /* X always moves first */
    uint8_t movecount  = 0;
    uint8_t status     = ST_PLAY;
    uint8_t my_mark    = is_x ? C_X : C_O;
    int     cursor     = 4;         /* center */

    if (is_x) ttt_send_state(mac, board, turn, movecount, status);  /* kick off */

    while (1) {
        /* --- ingest opponent messages --- */
        uint8_t from[6], sub; const uint8_t *body; int blen;
        while (ttt_next(from, &sub, &body, &blen)) {
            if (memcmp(from, mac, 6) != 0) continue;    /* only our opponent */
            if (sub == GM_QUIT) {
                m1_message_box(&m1_u8g2, "TicTacToe", "Opponent", "left", " OK ");
                return;
            }
            if (sub == GM_STATE && blen >= 12) {
                uint8_t mc = body[10];
                /* Adopt a newer state, or any reset (mc==0). */
                if (mc == 0 || mc > movecount) {
                    memcpy(board, body, 9);
                    turn      = body[9];
                    movecount = mc;
                    status    = body[11];
                    if (mc == 0) cursor = 4;
                }
            }
        }

        bool my_turn = (status == ST_PLAY && turn == my_mark);

        char top[12];
        snprintf(top, sizeof(top), "You: %c", (my_mark == C_X) ? 'X' : 'O');
        const char *mid = (status == ST_PLAY) ? (my_turn ? "Your turn" : "Waiting") : 0;
        const char *res = 0;
        if      (status == ST_XWIN) res = (my_mark == C_X) ? "You win!" : "You lose";
        else if (status == ST_OWIN) res = (my_mark == C_O) ? "You win!" : "You lose";
        else if (status == ST_DRAW) res = "Draw";
        ttt_draw(board, cursor, my_turn, top, mid, res);

        /* --- input --- */
        S_M1_Main_Q_t q; S_M1_Buttons_Status b;
        if (xQueueReceive(main_q_hdl, &q, pdMS_TO_TICKS(60)) != pdTRUE
            || q.q_evt_type != Q_EVENT_KEYPAD)
            continue;
        xQueueReceive(button_events_q_hdl, &b, 0);

        if (b.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK) {
            m1_esp_client_now_send(mac, (uint8_t[]){ GM_MAGIC, GM_QUIT }, 2);
            return;
        }

        if (status != ST_PLAY) {
            if (b.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK) {
                /* Rematch: reset to a fresh game (X first) and tell the peer. */
                memset(board, 0, sizeof(board));
                turn = C_X; movecount = 0; status = ST_PLAY; cursor = 4;
                ttt_send_state(mac, board, turn, movecount, status);
            }
            continue;
        }

        if (!my_turn) continue;

        int r = cursor / 3, c = cursor % 3;
        if      (b.event[BUTTON_UP_KP_ID]    == BUTTON_EVENT_CLICK) r = (r + 2) % 3;
        else if (b.event[BUTTON_DOWN_KP_ID]  == BUTTON_EVENT_CLICK) r = (r + 1) % 3;
        else if (b.event[BUTTON_LEFT_KP_ID]  == BUTTON_EVENT_CLICK) c = (c + 2) % 3;
        else if (b.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK) c = (c + 1) % 3;
        else if (b.event[BUTTON_OK_KP_ID]    == BUTTON_EVENT_CLICK) {
            if (board[cursor] == C_EMPTY) {
                board[cursor] = my_mark;
                movecount++;
                turn   = (my_mark == C_X) ? C_O : C_X;
                status = ttt_status(board);
                ttt_send_state(mac, board, turn, movecount, status);
            }
        }
        cursor = r * 3 + c;
    }
}

/* ---- mode select ---- */

/* Returns 0 = Two Players (peer link), 1 = Pass & Play (one device), -1 = back. */
static int ttt_mode_select(void)
{
    static const char *opt[2] = { "Two Players", "Pass & Play" };
    static const char *sub[2] = { "(2 M1s, link)", "(1 M1, share)" };
    int sel = 0;

    while (1) {
        m1_u8g2_firstpage();
        do {
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 2, 10, "Tic-Tac-Toe");
            for (int i = 0; i < 2; i++) {
                int y = 26 + i * 18;
                if (i == sel) {
                    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
                    u8g2_DrawBox(&m1_u8g2, 0, y - 9, 128, 18);
                    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
                }
                u8g2_DrawStr(&m1_u8g2, 4, y, opt[i]);
                u8g2_DrawStr(&m1_u8g2, 74, y, sub[i]);
                if (i == sel) u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            }
        } while (m1_u8g2_nextpage());

        S_M1_Main_Q_t q; S_M1_Buttons_Status b;
        if (xQueueReceive(main_q_hdl, &q, portMAX_DELAY) != pdTRUE
            || q.q_evt_type != Q_EVENT_KEYPAD)
            continue;
        xQueueReceive(button_events_q_hdl, &b, 0);
        if (b.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK) return -1;
        else if (b.event[BUTTON_UP_KP_ID] == BUTTON_EVENT_CLICK)   sel = 0;
        else if (b.event[BUTTON_DOWN_KP_ID] == BUTTON_EVENT_CLICK) sel = 1;
        else if (b.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK)   return sel;
    }
}

/* ---- local pass-and-play (one device, take turns) ---- */

static void ttt_local(void)
{
    uint8_t board[9] = {0};
    uint8_t turn = C_X, status = ST_PLAY;
    int     cursor = 4;

    while (1) {
        char mid[10];
        const char *res = 0;
        if (status == ST_PLAY) snprintf(mid, sizeof(mid), "Turn: %c", (turn == C_X) ? 'X' : 'O');
        else if (status == ST_XWIN) res = "X wins!";
        else if (status == ST_OWIN) res = "O wins!";
        else res = "Draw";
        ttt_draw(board, cursor, status == ST_PLAY, "Pass&Play",
                 (status == ST_PLAY) ? mid : 0, res);

        S_M1_Main_Q_t q; S_M1_Buttons_Status b;
        if (xQueueReceive(main_q_hdl, &q, portMAX_DELAY) != pdTRUE
            || q.q_evt_type != Q_EVENT_KEYPAD)
            continue;
        xQueueReceive(button_events_q_hdl, &b, 0);

        if (b.event[BUTTON_BACK_KP_ID] == BUTTON_EVENT_CLICK) return;

        if (status != ST_PLAY) {
            if (b.event[BUTTON_OK_KP_ID] == BUTTON_EVENT_CLICK) {
                memset(board, 0, sizeof(board));
                turn = C_X; status = ST_PLAY; cursor = 4;
            }
            continue;
        }

        int r = cursor / 3, c = cursor % 3;
        if      (b.event[BUTTON_UP_KP_ID]    == BUTTON_EVENT_CLICK) r = (r + 2) % 3;
        else if (b.event[BUTTON_DOWN_KP_ID]  == BUTTON_EVENT_CLICK) r = (r + 1) % 3;
        else if (b.event[BUTTON_LEFT_KP_ID]  == BUTTON_EVENT_CLICK) c = (c + 2) % 3;
        else if (b.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK) c = (c + 1) % 3;
        else if (b.event[BUTTON_OK_KP_ID]    == BUTTON_EVENT_CLICK) {
            if (board[cursor] == C_EMPTY) {
                board[cursor] = turn;
                status = ttt_status(board);
                turn = (turn == C_X) ? C_O : C_X;
            }
        }
        cursor = r * 3 + c;
    }
}

/* ---- entry point ---- */

void game_peer_ttt_run(void)
{
    int mode = ttt_mode_select();
    if (mode < 0) return;
    if (mode == 1) { ttt_local(); return; }   /* one device — no ESP needed */

    /* Two players over the peer link. */
    uint8_t my_mac[6] = {0};
    s_len = s_count = s_idx = s_off = 0;       /* fresh RX buffer */

    if (!m1_esp_client_now_start(TTT_CHANNEL, "M1", my_mac)) {
        m1_message_box(&m1_u8g2, "TicTacToe", "ESP not", "ready", " OK ");
        return;
    }
    char myname[24];
    snprintf(myname, sizeof(myname), "M1-%02X%02X", my_mac[4], my_mac[5]);
    m1_esp_client_now_start(TTT_CHANNEL, myname, my_mac);   /* re-announce with name */

    uint8_t mac[6]; bool is_x;
    if (ttt_lobby(my_mac, mac, &is_x))
        ttt_play(mac, is_x);

    m1_esp_client_now_stop();
    xQueueReset(main_q_hdl);
}
