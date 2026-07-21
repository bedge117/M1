/* See COPYING.txt for license details. */

/*
 * game_coin_flip.c
 *
 * Coin Flip — press OK to flip a coin.
 *
 * Native port of the M1 SDK example app (m1-sdk/examples/coin_flip). Replaces
 * the ELF-app shell (m1app_* API + manifest) with a built-in game that runs its
 * own event loop and returns on BACK, mirroring game_snake.c.
 *
 * M1 Project
 */

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "m1_games.h"

/*************************** D E F I N E S ************************************/

/*
 * The SDK app used u8g2_font_helvB08_tr, which is NOT linked into the firmware
 * (it does not appear in any native game_*.c). Substitute the closest safe font
 * already used by the native games.
 */
#define COIN_LABEL_FONT     u8g2_font_6x10_tr
#define COIN_TEXT_FONT      u8g2_font_6x10_tr
#define COIN_SMALL_FONT     u8g2_font_5x8_tr

/*************************** V A R I A B L E S *******************************/

static int total_flips = 0;
static int heads_count = 0;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void draw_coin(int frame, int result);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Draw the coin. frame < 0 => static result coin; else spinning frame.
 */
/*============================================================================*/
static void draw_coin(int frame, int result)
{
    int cx = 64;
    int cy = 28;

    if (frame < 0)
    {
        /* Static coin showing result */
        u8g2_DrawCircle(&m1_u8g2, cx, cy, 18, U8G2_DRAW_ALL);
        u8g2_DrawCircle(&m1_u8g2, cx, cy, 16, U8G2_DRAW_ALL);

        u8g2_SetFont(&m1_u8g2, COIN_LABEL_FONT);
        if (result == 0)
            u8g2_DrawStr(&m1_u8g2, cx - 6, cy + 4, "H");
        else
            u8g2_DrawStr(&m1_u8g2, cx - 5, cy + 4, "T");

        u8g2_SetFont(&m1_u8g2, COIN_LABEL_FONT);
        if (result == 0)
            u8g2_DrawStr(&m1_u8g2, 40, 54, "HEADS");
        else
            u8g2_DrawStr(&m1_u8g2, 42, 54, "TAILS");
    }
    else
    {
        /* Animated spinning coin — squish horizontally */
        /* Spin phases: full -> thin -> full -> thin ... */
        static const int widths[] = {18, 14, 8, 2, 8, 14, 18, 14, 8, 2, 8, 14};
        int num_phases = 12;
        int phase = frame % num_phases;
        int w = widths[phase];

        /* Draw ellipse approximation using lines */
        int h = 18;
        int x0 = cx - w;
        int x1 = cx + w;

        /* Top and bottom arcs */
        u8g2_DrawLine(&m1_u8g2, x0 + 2, cy - h + 2, x1 - 2, cy - h + 2);
        u8g2_DrawLine(&m1_u8g2, x0 + 2, cy + h - 2, x1 - 2, cy + h - 2);
        /* Sides */
        u8g2_DrawLine(&m1_u8g2, x0, cy - h + 6, x0, cy + h - 6);
        u8g2_DrawLine(&m1_u8g2, x1, cy - h + 6, x1, cy + h - 6);
        /* Corners */
        u8g2_DrawLine(&m1_u8g2, x0, cy - h + 6, x0 + 2, cy - h + 2);
        u8g2_DrawLine(&m1_u8g2, x1, cy - h + 6, x1 - 2, cy - h + 2);
        u8g2_DrawLine(&m1_u8g2, x0, cy + h - 6, x0 + 2, cy + h - 2);
        u8g2_DrawLine(&m1_u8g2, x1, cy + h - 6, x1 - 2, cy + h - 2);

        /* Show H or T flashing during spin */
        if (w > 6)
        {
            u8g2_SetFont(&m1_u8g2, COIN_LABEL_FONT);
            char c = (phase < 6) ? 'H' : 'T';
            char s[2] = { c, 0 };
            u8g2_DrawStr(&m1_u8g2, cx - 4, cy + 4, s);
        }

        /* "Flipping..." text */
        u8g2_SetFont(&m1_u8g2, COIN_TEXT_FONT);
        u8g2_DrawStr(&m1_u8g2, 30, 60, "Flipping...");
    }
}


/*============================================================================*/
/*
 * @brief  Coin Flip game entry point. Runs own event loop, returns on BACK.
 */
/*============================================================================*/
void game_coin_flip_run(void)
{
    game_button_t btn;

    game_rand_seed();

    int result = -1;   /* -1 = no result yet */
    int animating = 0;
    int anim_frame = 0;

    while (1)
    {
        btn = game_poll_button(animating ? 60 : 200);

        if (btn == GAME_BTN_BACK)
            break;

        if (btn == GAME_BTN_OK && !animating)
        {
            animating = 1;
            anim_frame = 0;
            m1_buzzer_notification();
        }

        if (animating)
        {
            anim_frame++;
            if (anim_frame >= 18)
            {
                /* Done spinning — pick result */
                animating = 0;
                result = game_rand_range(0, 1);
                total_flips++;
                if (result == 0)
                    heads_count++;
                m1_buzzer_notification();
            }
        }

        /* Draw */
        m1_u8g2_firstpage();
        do {
            u8g2_SetDrawColor(&m1_u8g2, 1);

            /* Title bar */
            u8g2_DrawBox(&m1_u8g2, 0, 0, 128, 12);
            u8g2_SetDrawColor(&m1_u8g2, 0);
            u8g2_SetFont(&m1_u8g2, COIN_TEXT_FONT);
            u8g2_DrawStr(&m1_u8g2, 30, 10, "Coin Flip");
            u8g2_SetDrawColor(&m1_u8g2, 1);

            if (animating)
            {
                draw_coin(anim_frame, 0);
            }
            else if (result >= 0)
            {
                draw_coin(-1, result);
            }
            else
            {
                /* Initial screen */
                u8g2_SetFont(&m1_u8g2, COIN_TEXT_FONT);
                u8g2_DrawStr(&m1_u8g2, 16, 34, "Press OK to flip");

                /* Draw a static coin preview */
                u8g2_DrawCircle(&m1_u8g2, 64, 48, 8, U8G2_DRAW_ALL);
                u8g2_SetFont(&m1_u8g2, COIN_SMALL_FONT);
                u8g2_DrawStr(&m1_u8g2, 62, 51, "?");
            }

            /* Stats line in title bar */
            if (total_flips > 0)
            {
                char stats[16];
                u8g2_SetDrawColor(&m1_u8g2, 0);
                u8g2_SetFont(&m1_u8g2, COIN_SMALL_FONT);
                snprintf(stats, sizeof(stats), "H%d T%d",
                         heads_count, total_flips - heads_count);
                u8g2_DrawStr(&m1_u8g2, 2, 9, stats);
                u8g2_SetDrawColor(&m1_u8g2, 1);
            }

        } while (m1_u8g2_nextpage());
    }
}
