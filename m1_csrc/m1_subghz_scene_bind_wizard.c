/* See COPYING.txt for license details. */

/**
 * @file   m1_subghz_scene_bind_wizard.c
 * @brief  Sub-GHz "Bind New Remote" wizard — guided rolling-code binding.
 *
 * Provides a step-by-step wizard for creating and binding a brand-new
 * rolling-code remote to a receiver, using the M1's on-board RF hardware.
 *
 * Supported protocols (all have SubGhzProtocolFlag_PwmKeyReplay — the M1
 * can bind AND replay them indefinitely from Sub-GHz → Saved):
 *   • CAME Atomo 433 MHz       (62 bit, OOK PWM)
 *   • Nice FloR-S 433 MHz      (52 bit, OOK PWM)
 *   • Alutech AT-4N 433 MHz    (64 bit, OOK PWM)
 *   • DITEC GOL4 433 MHz       (54 bit, OOK PWM)
 *   • KingGates Stylo4k 433 MHz (60 bit, OOK PWM)
 *
 * Flow:
 *   1. Protocol picker list  → user selects protocol
 *   2. Key generation + file save (auto-advances)
 *   3. Step-by-step binding instructions (1..N screens)
 *        – Each screen: header "Step N of M", instruction text, optional
 *          countdown bar, OK=Next / OK=Send (for TX steps)
 *   4. Done screen — confirms file saved, notes replay capability
 *
 * Navigation:
 *   OK     = primary action (select protocol, advance step, fire TX)
 *   BACK   = go to previous step / exit wizard
 *   UP/DOWN = scroll protocol list
 *
 * The countdown bar is informational only; it ticks with the Sub-GHz
 * hopper-tick (200 ms) and does not auto-advance.  This avoids forcing
 * users to restart the wizard if they're slightly slow.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "ff.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_scene.h"
#include "m1_submenu.h"
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_api.h"
#include "m1_subghz_scene.h"
#include "subghz_new_remote_gen.h"
#include "subghz_submenu_model.h"
#include "flipper_subghz.h"

/*============================================================================*/
/* External linkage                                                           */
/*============================================================================*/

/* (Radio lifecycle is now owned by SubGhzSceneTransmitter — no direct
 *  menu_sub_ghz_* calls needed in this scene.) */
extern uint32_t HAL_GetTick(void);

/*============================================================================*/
/* Binding step definition                                                    */
/*============================================================================*/

/**
 * A single instruction step displayed full-screen.
 *
 * @p text uses '\n' to break lines (handled by m1_draw_text_box).
 * Lines should be ≤ 22 characters for NokiaSmallPlain.
 * Maximum of 4 visible lines in the content area.
 *
 * If @p countdown_sec > 0 the step shows a countdown bar counting down
 * from that many seconds.  Bar is informational — the user still presses
 * OK to advance.
 *
 * If @p tx_step is true, OK fires a TX via sub_ghz_replay_flipper_file()
 * then auto-advances to the next step.
 */
typedef struct {
    const char *text;
    uint8_t     countdown_sec; /* 0 = no countdown */
    bool        tx_step;       /* true = OK fires TX before advancing */
} BindStep;

/*============================================================================*/
/* Per-protocol step arrays                                                   */
/*============================================================================*/

/* ── CAME Atomo ──────────────────────────────────────────────────────────── */
static const BindStep steps_came_atomo[] = {
    {
        .text          = "New remote generated\nand saved to SD.\nApproach receiver\nwithin 2 metres.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Hold your EXISTING\nremote button toward\nthe receiver for\n6-10 seconds.",
        .countdown_sec = 10,
        .tx_step       = false,
    },
    {
        .text          = "Within 20 seconds:\npress OK to transmit\nthe new remote code.\nPress again if needed.",
        .countdown_sec = 20,
        .tx_step       = true,
    },
    {
        .text          = "Done! Check receiver\nLED for confirmation.\nFile saved — use\nSaved to replay.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
};

/* ── Nice FloR-S ─────────────────────────────────────────────────────────── */
static const BindStep steps_nice_flors[] = {
    {
        .text          = "New remote generated\nand saved to SD.\nApproach receiver\nwithin 2 metres.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Press OK to send\nfor 5+ seconds.\nPress OK 5 times if\nneeded, then wait 2s.",
        .countdown_sec = 0,
        .tx_step       = true,
    },
    {
        .text          = "Press your EXISTING\nremote button 3 times\nslowly (1 sec apart).",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Press OK once to\nfinish binding.",
        .countdown_sec = 0,
        .tx_step       = true,
    },
    {
        .text          = "Done! Receiver LED\nshould flash 3 times.\nFile saved — use\nSaved to replay.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
};

/* ── Alutech AT-4N ───────────────────────────────────────────────────────── */
static const BindStep steps_alutech[] = {
    {
        .text          = "New remote generated\nand saved to SD.\nOpen the receiver\ncontrol box.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Press 'F' button on\nthe receiver board\nfor ~3 seconds until\ndisplay shows 'Pr'.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Press 'F' again until\ndisplay shows 'Lr'\n(learn mode).",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Use +/- buttons to\nfind an empty slot\n(no red dot shown).",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Press OK to transmit\nthe new remote code.\nWait for display to\nconfirm.",
        .countdown_sec = 0,
        .tx_step       = true,
    },
    {
        .text          = "Press 'F' for ~3 sec\nto exit programming.\nFile saved — use\nSaved to replay.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
};

/* ── DITEC GOL4 ──────────────────────────────────────────────────────────── */
static const BindStep steps_ditec_gol4[] = {
    {
        .text          = "New remote generated\nand saved to SD.\nOpen the receiver\ncontrol box.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Find the Learn button\non the receiver board.\nHold it until the\nLED turns on.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Press OK to transmit\nthe new remote code.\nWait for LED to\nconfirm then release.",
        .countdown_sec = 10,
        .tx_step       = true,
    },
    {
        .text          = "Done! File saved to\n0:/SUBGHZ/ — use\nSaved to replay this\nremote at any time.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
};

/* ── KingGates Stylo4k ───────────────────────────────────────────────────── */
static const BindStep steps_kinggates[] = {
    {
        .text          = "New remote generated\nand saved to SD.\nOpen the receiver\ncontrol box.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Find the P2 (Learn)\nbutton on the board.\nPress and hold until\nLED turns on.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
    {
        .text          = "Press OK to transmit\nthe new remote code.\nWait for LED to\nconfirm then release.",
        .countdown_sec = 10,
        .tx_step       = true,
    },
    {
        .text          = "Done! File saved to\n0:/SUBGHZ/ — use\nSaved to replay this\nremote at any time.",
        .countdown_sec = 0,
        .tx_step       = false,
    },
};

/*============================================================================*/
/* Protocol definition table                                                  */
/*============================================================================*/

typedef struct {
    BindWizardProto   proto_id;
    const BindStep   *steps;
    uint8_t           step_count;
} BindProtoDef;

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))
#endif

static const BindProtoDef bind_protos[] = {
    { BW_PROTO_CAME_ATOMO,        steps_came_atomo, ARRAY_SIZE(steps_came_atomo) },
    { BW_PROTO_NICE_FLOR_S,       steps_nice_flors, ARRAY_SIZE(steps_nice_flors) },
    { BW_PROTO_ALUTECH_AT4N,      steps_alutech,    ARRAY_SIZE(steps_alutech) },
    { BW_PROTO_DITEC_GOL4,        steps_ditec_gol4, ARRAY_SIZE(steps_ditec_gol4) },
    { BW_PROTO_KINGGATES_STYLO4K, steps_kinggates,  ARRAY_SIZE(steps_kinggates) },
};

#define BIND_PROTO_COUNT  ((uint8_t)(sizeof(bind_protos) / sizeof(bind_protos[0])))

/*============================================================================*/
/* Scene state (static — same lifetime pattern as other scenes)              */
/*============================================================================*/

typedef enum {
    BW_STATE_PROTO_SEL = 0, /* Protocol picker list   */
    BW_STATE_GENERATING,    /* "Generating…" (1 tick) */
    BW_STATE_STEP,          /* Instruction step N     */
    BW_STATE_DONE,          /* Final success screen   */
    BW_STATE_SAVE_ERROR,    /* SD/save failure screen */
} BwState;

static BwState   bw_state;
/* Phase 7c-4: scroll/selection math for the protocol picker comes from the
 * reusable pure-logic widget; selected index lives in s_proto_model.selected. */
static subghz_submenu_model_t s_proto_model;
static const char *s_proto_labels[BIND_PROTO_COUNT];
static uint8_t   bw_step;       /* current step index (0-based) */
static uint32_t  bw_countdown_start; /* HAL_GetTick() when countdown began */
static bool      bw_tx_failed;  /* true if last TX step failed */
static NewRemoteParams bw_params;
static char      bw_filepath[72]; /* "0:/SUBGHZ/<file_base>.sub" */

/*============================================================================*/
/* Helpers                                                                    */
/*============================================================================*/

/* STM32H5 unique device ID register addresses (see RM0481 §53) */
#define STM32H5_UID0_ADDR   (0x08FFF800UL)
#define STM32H5_UID1_ADDR   (0x08FFF804UL)
#define STM32H5_UID2_ADDR   (0x08FFF808UL)

/* LCG multiplier (Knuth MMIX) used to further mix tick entropy */
#define BW_LCG_MUL   (6364136223846793005ULL)

/* Line height in pixels for instruction text (NokiaSmallPlain ascent=7) */
#define STEP_TEXT_LINE_H    9

/* Populate the protocol-label pointer table and (re)initialise the picker
 * model.  Called from scene_on_enter (and from any future place that resets
 * the picker).  Labels are static strings owned by subghz_new_remote_gen.c,
 * so pointer lifetime is the firmware's. */
static void proto_picker_init(void)
{
    for (uint8_t i = 0; i < BIND_PROTO_COUNT; i++)
        s_proto_labels[i] = subghz_new_remote_proto_label(bind_protos[i].proto_id);
    subghz_submenu_model_init(&s_proto_model,
                              BIND_PROTO_COUNT,
                              M1_MENU_VIS(BIND_PROTO_COUNT));
}

/* Return the currently-selected protocol entry. */
static const BindProtoDef *current_proto(void)
{
    return &bind_protos[s_proto_model.selected];
}

static const BindStep *current_step(void)
{
    return &current_proto()->steps[bw_step];
}

/** Generate key and save .sub file.  Returns true on success. */
static bool bw_generate_and_save(void)
{
    /* Entropy: mix HAL_GetTick() with STM32H5 unique device ID words
     * (three 32-bit read-once OTP registers at 0x08FFF800..08) to ensure
     * each device produces different serials even at the same boot time.
     * This matches the seeding strategy in m1_crypto.c. */
    uint32_t t    = HAL_GetTick();
    uint32_t uid0 = *(volatile uint32_t *)(STM32H5_UID0_ADDR);
    uint32_t uid1 = *(volatile uint32_t *)(STM32H5_UID1_ADDR);
    uint32_t uid2 = *(volatile uint32_t *)(STM32H5_UID2_ADDR);
    uint64_t seed = ((uint64_t)(uid0 ^ t) << 32)
                  ^ ((uint64_t)(uid1 ^ uid2))
                  ^ ((uint64_t)t * BW_LCG_MUL);

    if (!subghz_new_remote_gen(current_proto()->proto_id, seed, &bw_params))
        return false;

    /* Build SD path.  file_base is produced by subghz_new_remote_gen which
     * is unit-tested to contain no path separators, spaces, or tabs; assert the
     * trust boundary at runtime to catch any future regressions. */
    if (strchr(bw_params.file_base, '/') || strchr(bw_params.file_base, '\\') ||
        strchr(bw_params.file_base, ' ')  || strchr(bw_params.file_base, '\t'))
        return false; /* Defensive: should never happen */
    /* Ensure the SUBGHZ directory exists (safe to call even if it already does) */
    f_mkdir("0:/SUBGHZ");
    snprintf(bw_filepath, sizeof(bw_filepath),
             "0:/SUBGHZ/%s.sub", bw_params.file_base);

    return flipper_subghz_save_key(bw_filepath,
               bw_params.freq_hz, "FuriHalSubGhzPresetOok650Async",
               bw_params.proto_name,
               bw_params.bit_count, bw_params.key, (uint32_t)bw_params.te);
}

/** Push the async Transmitter scene to fire a TX step.  Replaces the legacy
 *  blocking sub_ghz_replay_flipper_file() call; the Transmitter scene owns
 *  the radio lifecycle (init/exit + tx_completed_naturally flag) and pops
 *  back here on completion or BACK abort. */
static void bw_push_tx(SubGhzApp *app)
{
    /* tx_path is 72 bytes; bw_filepath is "0:/SUBGHZ/" + up to 39 chars
     * of file_base + ".sub" = at most 53 chars, safely within bounds. */
    strncpy(app->tx_path, bw_filepath, sizeof(app->tx_path) - 1);
    app->tx_path[sizeof(app->tx_path) - 1] = '\0';
    app->tx_repeat_count = 1U;                   /* static keys: 1 burst */
    app->tx_mode         = 0U;                   /* SUBGHZ_TX_MODE_SINGLE */
    app->tx_autostart    = true;                 /* 1-press fire UX */
    /* Phase 4b — pass the binding protocol name to the Transmitter
     * scene.  All five wizard protocols (CAME Atomo, Nice FloR-S,
     * Alutech AT-4N, DITEC GOL4, KingGates Stylo4k) support button
     * cycling, so the Transmitter will enable LEFT/RIGHT during the
     * step.  Until Phase 4c lands the key-override path, cycling
     * only updates the visible "Btn X/Y" indicator. */
    strncpy(app->tx_protocol_name, bw_params.proto_name,
            sizeof(app->tx_protocol_name) - 1);
    app->tx_protocol_name[sizeof(app->tx_protocol_name) - 1] = '\0';
    app->resume_from_child = true;
    subghz_scene_push(app, SubGhzSceneTransmitter);
}

/*============================================================================*/
/* Draw helpers                                                               */
/*============================================================================*/

static void draw_proto_sel(void)
{
    /* Re-sync visible_count in case the user changed the text-size setting
     * while a child scene (Transmitter, etc.) was on top — matches the
     * SavedMenu / MoreRAW pattern. */
    subghz_submenu_model_set_visible_count(&s_proto_model,
                                           M1_MENU_VIS(BIND_PROTO_COUNT));
    m1_submenu_draw(&s_proto_model, "Bind New Remote", s_proto_labels);
}

static void draw_generating(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 124, "Bind New Remote", TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 10, 36, "Generating key...");
    m1_u8g2_nextpage();
}

static void draw_save_error(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 124, "Save Failed", TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 4, 28, "Check SD card.");
    u8g2_DrawStr(&m1_u8g2, 4, 40, "0:/SUBGHZ/ must");
    u8g2_DrawStr(&m1_u8g2, 4, 52, "be writable.");
    u8g2_DrawStr(&m1_u8g2, 4, 63, "BACK=Retry");
    m1_u8g2_nextpage();
}

static void draw_step(void)
{
    const BindProtoDef *pd  = current_proto();
    const BindStep     *st  = current_step();

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Title: "Step N of M" */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    {
        char title[24];
        snprintf(title, sizeof(title), "Step %u of %u",
                 (unsigned)(bw_step + 1), (unsigned)pd->step_count);
        m1_draw_text(&m1_u8g2, 0, 9, 128, title, TEXT_ALIGN_CENTER);
    }
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);

    /* Instruction text */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    m1_draw_text_box(&m1_u8g2, 2, 20, 124, STEP_TEXT_LINE_H, st->text, TEXT_ALIGN_LEFT);

    /* Countdown bar + hints */
    if (st->countdown_sec > 0)
    {
        uint32_t elapsed_ms = HAL_GetTick() - bw_countdown_start;
        uint32_t total_ms   = (uint32_t)st->countdown_sec * 1000u;
        uint32_t remaining_ms = (elapsed_ms < total_ms) ? (total_ms - elapsed_ms) : 0u;

        /* Bar */
        uint8_t bar_w = (remaining_ms > 0)
                      ? (uint8_t)((uint32_t)120u * remaining_ms / total_ms)
                      : 0u;
        u8g2_DrawFrame(&m1_u8g2, 4, 51, 120, 5);
        if (bar_w > 0)
            u8g2_DrawBox(&m1_u8g2, 4, 51, bar_w, 5);

        /* Time remaining + action/failure hint */
        char cd[8];
        snprintf(cd, sizeof(cd), "%us", (unsigned)((remaining_ms + 999u) / 1000u));
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 4, 63, cd);
        if (bw_tx_failed && st->tx_step)
            u8g2_DrawStr(&m1_u8g2, 46, 63, "TX fail-retry OK");
        else
            u8g2_DrawStr(&m1_u8g2, 70, 63, st->tx_step ? "OK=Send" : "OK=Next");
    }
    else
    {
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        if (bw_tx_failed)
            u8g2_DrawStr(&m1_u8g2, 4, 63, "TX fail-retry OK");
        else
            u8g2_DrawStr(&m1_u8g2, 4, 63, st->tx_step ? "OK=Send" : "OK=Next");
    }

    m1_u8g2_nextpage();
}

static void draw_done(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 124, "Binding Complete!", TEXT_ALIGN_CENTER);
    u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

    /* Show the filename (just the base + extension, not the full path) */
    char fname[44];
    /* Truncate to sizeof(fname) - sizeof(".sub") = 39 chars of base */
    snprintf(fname, sizeof(fname), "%.*s.sub",
             (int)(sizeof(fname) - sizeof(".sub")), bw_params.file_base);
    u8g2_DrawStr(&m1_u8g2, 2, 22, fname);

    u8g2_DrawStr(&m1_u8g2, 2, 33, "Saved to 0:/SUBGHZ/");
    u8g2_DrawStr(&m1_u8g2, 2, 44, "Use Saved to replay");
    u8g2_DrawStr(&m1_u8g2, 2, 55, "with counter-incr.");

    u8g2_DrawStr(&m1_u8g2, 4, 63, "BACK=Exit");

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Scene callbacks                                                            */
/*============================================================================*/

static void scene_on_enter(SubGhzApp *app)
{
    /* Detect resume-from-child: we just popped back from the Transmitter
     * scene after a TX step.  Inspect tx_completed_naturally to decide
     * whether to advance to the next step or stay on the current step
     * (so the user can retry on a failure / abort). */
    if (app->resume_from_child)
    {
        app->resume_from_child = false;

        if (bw_state == BW_STATE_STEP)
        {
            if (app->tx_completed_naturally)
            {
                /* TX ran to completion — advance to next step or done. */
                const BindProtoDef *pd = current_proto();
                bw_tx_failed = false;
                app->hopper_active = false;

                if (bw_step + 1 < pd->step_count)
                {
                    bw_step++;
                    bw_countdown_start = HAL_GetTick();
                    if (current_step()->countdown_sec > 0)
                        app->hopper_active = true;
                }
                else
                {
                    bw_state = BW_STATE_DONE;
                }
            }
            else
            {
                /* User aborted via BACK in the Transmitter scene — stay on
                 * this TX step so they can press OK again to retry. */
                bw_tx_failed = false;
            }
            app->need_redraw = true;
            return;
        }
        /* Fall through to full reset if resume happened in an unexpected
         * state (defensive — should not occur in practice). */
    }

    bw_state        = BW_STATE_PROTO_SEL;
    proto_picker_init();
    bw_step         = 0;
    bw_tx_failed    = false;
    memset(&bw_params, 0, sizeof(bw_params));
    bw_filepath[0]  = '\0';
    app->need_redraw = true;
}

static bool scene_on_event(SubGhzApp *app, SubGhzEvent event)
{
    switch (bw_state)
    {
        /* ── Protocol selection list ─────────────────────────────────────── */
        case BW_STATE_PROTO_SEL:
            /* Re-sync visible_count so a text-size change picked up while a
             * child scene was on top is honoured by the model's scroll math. */
            subghz_submenu_model_set_visible_count(&s_proto_model,
                                                   M1_MENU_VIS(BIND_PROTO_COUNT));
            switch (event)
            {
                case SubGhzEventBack:
                    subghz_scene_pop(app);
                    return true;

                case SubGhzEventUp:
                    subghz_submenu_model_up(&s_proto_model);
                    app->need_redraw = true;
                    return true;

                case SubGhzEventDown:
                    subghz_submenu_model_down(&s_proto_model);
                    app->need_redraw = true;
                    return true;

                case SubGhzEventOk:
                    bw_state = BW_STATE_GENERATING;
                    app->need_redraw = true;
                    /* Enable hopper ticks so generating screen gets drawn,
                     * then we advance to steps on the first HopperTick. */
                    app->hopper_active = true;
                    return true;

                default:
                    break;
            }
            break;

        /* ── Generating (auto-advance on first HopperTick) ───────────────── */
        case BW_STATE_GENERATING:
            if (event == SubGhzEventHopperTick || event == SubGhzEventOk)
            {
                app->hopper_active = false;

                if (!bw_generate_and_save())
                {
                    /* Save failed — show error screen */
                    bw_state         = BW_STATE_SAVE_ERROR;
                    app->need_redraw = true;
                    return true;
                }

                bw_step             = 0;
                bw_countdown_start  = HAL_GetTick();
                bw_state            = BW_STATE_STEP;

                /* Enable hopper ticks if first step has a countdown */
                if (current_step()->countdown_sec > 0)
                    app->hopper_active = true;

                app->need_redraw = true;
                return true;
            }
            if (event == SubGhzEventBack)
            {
                app->hopper_active = false;
                bw_state           = BW_STATE_PROTO_SEL;
                app->need_redraw   = true;
                return true;
            }
            break;

        /* ── Step screens ────────────────────────────────────────────────── */
        case BW_STATE_STEP:
        {
            const BindProtoDef *pd = current_proto();
            const BindStep     *st = current_step();

            if (event == SubGhzEventHopperTick)
            {
                /* Redraw to update countdown bar */
                app->need_redraw = true;
                return true;
            }

            if (event == SubGhzEventBack)
            {
                /* Previous step, or exit to proto sel on step 0 */
                app->hopper_active = false;
                bw_tx_failed       = false;

                if (bw_step > 0)
                {
                    bw_step--;
                    bw_countdown_start = HAL_GetTick();
                    if (current_step()->countdown_sec > 0)
                        app->hopper_active = true;
                }
                else
                {
                    bw_state        = BW_STATE_PROTO_SEL;
                }
                app->need_redraw = true;
                return true;
            }

            if (event == SubGhzEventOk)
            {
                /* Fire TX if this is a TX step — push the async Transmitter
                 * scene.  Advancement to the next step happens in
                 * scene_on_enter after pop-back (see resume_from_child path). */
                if (st->tx_step)
                {
                    bw_tx_failed = false;
                    app->hopper_active = false;
                    bw_push_tx(app);
                    return true;
                }

                bw_tx_failed = false;

                /* Non-TX step: advance to next step or done screen */
                app->hopper_active = false;

                if (bw_step + 1 < pd->step_count)
                {
                    bw_step++;
                    bw_countdown_start = HAL_GetTick();
                    if (current_step()->countdown_sec > 0)
                        app->hopper_active = true;
                }
                else
                {
                    bw_state = BW_STATE_DONE;
                }
                app->need_redraw = true;
                return true;
            }
            break;
        }

        /* ── Done screen ─────────────────────────────────────────────────── */
        case BW_STATE_DONE:
            if (event == SubGhzEventBack || event == SubGhzEventOk)
            {
                subghz_scene_pop(app);
                return true;
            }
            break;

        /* ── Save error screen ───────────────────────────────────────────── */
        case BW_STATE_SAVE_ERROR:
            if (event == SubGhzEventBack || event == SubGhzEventOk)
            {
                /* Return to protocol selection so the user can retry */
                bw_state         = BW_STATE_PROTO_SEL;
                app->need_redraw = true;
                return true;
            }
            break;

        default:
            break;
    }

    return false;
}

static void scene_on_exit(SubGhzApp *app)
{
    /* Make sure we don't leave hopper ticks running */
    app->hopper_active = false;
}

static void scene_draw(SubGhzApp *app)
{
    (void)app;

    switch (bw_state)
    {
        case BW_STATE_PROTO_SEL:  draw_proto_sel();    break;
        case BW_STATE_GENERATING: draw_generating();   break;
        case BW_STATE_STEP:       draw_step();         break;
        case BW_STATE_DONE:       draw_done();         break;
        case BW_STATE_SAVE_ERROR: draw_save_error();   break;
        default:                  break;
    }
}

/*============================================================================*/
/* Handler table                                                              */
/*============================================================================*/

const SubGhzSceneHandlers subghz_scene_bind_wizard_handlers = {
    .on_enter = scene_on_enter,
    .on_event = scene_on_event,
    .on_exit  = scene_on_exit,
    .draw     = scene_draw,
};
