/* See COPYING.txt for license details. */

/*
*
* game_flappy.c
*
* Flappy Bird — Tap OK to flap, avoid the pipes
*
* Ported from the M1 SDK example app (examples/flappy_bird/main.c).
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "m1_games.h"

/*************************** D E F I N E S ************************************/

#define SCREEN_W      128
#define SCREEN_H      64
#define PIPE_W        8
#define PIPE_GAP      26
#define PIPE_SPEED    2
#define BIRD_X        20
#define BIRD_W        6
#define BIRD_H        5
#define MAX_PIPES     3
#define GROUND_H      4
#define PLAY_H        (SCREEN_H - GROUND_H)
#define FRAME_MS      50   /* ~20 fps */

/* Scaled physics: positions are multiplied by PHYS_SCALE.
 * This allows fractional gravity (1/4 pixel per frame²). */
#define PHYS_SCALE    4
#define GRAVITY       1           /* 0.25 px/frame² */
#define FLAP_FORCE    (-8)        /* -2.0 px/frame upward */
#define VY_MAX        (3 * PHYS_SCALE)  /* terminal velocity */

//************************** S T R U C T U R E S *******************************

typedef struct {
    int x;
    int gap_y;    /* top of the gap */
    int scored;   /* already counted for score */
    int active;
} pipe_t;

/***************************** V A R I A B L E S ******************************/

static int bird_y_s;   /* scaled position (multiply by PHYS_SCALE) */
static int bird_vy;    /* scaled velocity */
static pipe_t pipes[MAX_PIPES];
static int score;
static int best_score;
static int alive;
static int ground_scroll;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void flappy_reset_game(void);
static void flappy_update(int flap);
static void flappy_draw_bird(void);
static void flappy_draw_pipes(void);
static void flappy_draw_ground(void);
static int  flappy_append_int(char *buf, int pos, int val);
static int  flappy_append_str(char *buf, int pos, const char *s);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Append a decimal integer to buf at pos, return new pos
 */
/*============================================================================*/
static int flappy_append_int(char *buf, int pos, int val)
{
    if (val < 0)
    {
        buf[pos++] = '-';
        val = -val;
    }
    if (val == 0)
    {
        buf[pos++] = '0';
        return pos;
    }

    char tmp[12];
    int tpos = 0;
    while (val > 0 && tpos < 11)
    {
        tmp[tpos++] = (char)('0' + (val % 10));
        val /= 10;
    }
    for (int j = tpos - 1; j >= 0; j--)
    {
        buf[pos++] = tmp[j];
    }
    return pos;
}


/*============================================================================*/
/*
 * @brief  Append a string to buf at pos, return new pos
 */
/*============================================================================*/
static int flappy_append_str(char *buf, int pos, const char *s)
{
    while (*s)
    {
        buf[pos++] = *s++;
    }
    return pos;
}


/*============================================================================*/
/*
 * @brief  Reset the game state for a new round
 */
/*============================================================================*/
static void flappy_reset_game(void)
{
    bird_y_s = (PLAY_H / 2) * PHYS_SCALE;
    bird_vy = 0;
    score = 0;
    alive = 1;
    ground_scroll = 0;

    for (int i = 0; i < MAX_PIPES; i++)
    {
        pipes[i].x = SCREEN_W + i * 50;
        pipes[i].gap_y = game_rand_range(10, PLAY_H - PIPE_GAP - 10);
        pipes[i].scored = 0;
        pipes[i].active = 1;
    }
}


/*============================================================================*/
/*
 * @brief  Advance the game one frame
 */
/*============================================================================*/
static void flappy_update(int flap)
{
    /* Bird physics (scaled coordinates) */
    if (flap)
        bird_vy = FLAP_FORCE;

    bird_vy += GRAVITY;
    if (bird_vy > VY_MAX)
        bird_vy = VY_MAX;
    bird_y_s += bird_vy;

    /* Convert to screen pixels for checks */
    int bird_y = bird_y_s / PHYS_SCALE;

    /* Clamp to play area */
    if (bird_y < 0)
    {
        bird_y_s = 0;
        bird_vy = 0;
        bird_y = 0;
    }
    if (bird_y > PLAY_H - BIRD_H)
    {
        bird_y_s = (PLAY_H - BIRD_H) * PHYS_SCALE;
        bird_y = PLAY_H - BIRD_H;
        alive = 0;
    }

    /* Move pipes */
    for (int i = 0; i < MAX_PIPES; i++)
    {
        if (!pipes[i].active)
            continue;

        pipes[i].x -= PIPE_SPEED;

        /* Score check */
        if (!pipes[i].scored && pipes[i].x + PIPE_W < BIRD_X)
        {
            pipes[i].scored = 1;
            score++;
            m1_buzzer_notification();
        }

        /* Recycle pipe */
        if (pipes[i].x < -PIPE_W)
        {
            pipes[i].x = SCREEN_W + game_rand_range(10, 30);
            pipes[i].gap_y = game_rand_range(10, PLAY_H - PIPE_GAP - 10);
            pipes[i].scored = 0;
        }

        /* Collision check */
        if (pipes[i].x < BIRD_X + BIRD_W && pipes[i].x + PIPE_W > BIRD_X)
        {
            /* Bird overlaps pipe column horizontally */
            if (bird_y < pipes[i].gap_y || bird_y + BIRD_H > pipes[i].gap_y + PIPE_GAP)
            {
                alive = 0;
            }
        }
    }

    /* Ground scroll */
    ground_scroll = (ground_scroll + PIPE_SPEED) % 4;
}


/*============================================================================*/
/*
 * @brief  Draw the bird
 */
/*============================================================================*/
static void flappy_draw_bird(void)
{
    int by = bird_y_s / PHYS_SCALE;

    /* Simple bird shape */
    u8g2_DrawBox(&m1_u8g2, BIRD_X, by, BIRD_W, BIRD_H);

    /* Wing (flap animation based on velocity) */
    if (bird_vy < 0)
    {
        /* Wing up */
        u8g2_DrawPixel(&m1_u8g2, BIRD_X + 1, by - 1);
        u8g2_DrawPixel(&m1_u8g2, BIRD_X + 2, by - 1);
    }
    else
    {
        /* Wing down */
        u8g2_DrawPixel(&m1_u8g2, BIRD_X + 1, by + BIRD_H);
        u8g2_DrawPixel(&m1_u8g2, BIRD_X + 2, by + BIRD_H);
    }

    /* Eye */
    u8g2_SetDrawColor(&m1_u8g2, 0);
    u8g2_DrawPixel(&m1_u8g2, BIRD_X + BIRD_W - 2, by + 1);
    u8g2_SetDrawColor(&m1_u8g2, 1);

    /* Beak */
    u8g2_DrawPixel(&m1_u8g2, BIRD_X + BIRD_W, by + 2);
    u8g2_DrawPixel(&m1_u8g2, BIRD_X + BIRD_W + 1, by + 2);
}


/*============================================================================*/
/*
 * @brief  Draw the pipes
 */
/*============================================================================*/
static void flappy_draw_pipes(void)
{
    for (int i = 0; i < MAX_PIPES; i++)
    {
        if (!pipes[i].active)
            continue;

        int px = pipes[i].x;
        int gy = pipes[i].gap_y;

        if (px > SCREEN_W || px + PIPE_W < 0)
            continue;

        /* Top pipe */
        if (gy > 0)
            u8g2_DrawBox(&m1_u8g2, px, 0, PIPE_W, gy);

        /* Top pipe lip */
        if (gy >= 3)
        {
            u8g2_DrawBox(&m1_u8g2, px - 1, gy - 3, PIPE_W + 2, 3);
        }

        /* Bottom pipe */
        int bot_y = gy + PIPE_GAP;
        if (bot_y < PLAY_H)
            u8g2_DrawBox(&m1_u8g2, px, bot_y, PIPE_W, PLAY_H - bot_y);

        /* Bottom pipe lip */
        if (bot_y + 3 <= PLAY_H)
        {
            u8g2_DrawBox(&m1_u8g2, px - 1, bot_y, PIPE_W + 2, 3);
        }
    }
}


/*============================================================================*/
/*
 * @brief  Draw the scrolling ground
 */
/*============================================================================*/
static void flappy_draw_ground(void)
{
    /* Ground line */
    u8g2_DrawHLine(&m1_u8g2, 0, PLAY_H, SCREEN_W);

    /* Ground pattern */
    for (int x = -ground_scroll; x < SCREEN_W; x += 4)
    {
        u8g2_DrawPixel(&m1_u8g2, x, PLAY_H + 2);
    }
}


/*============================================================================*/
/*
 * @brief  Main flappy bird entry point. Runs own event loop, returns on BACK.
 */
/*============================================================================*/
void game_flappy_run(void)
{
    game_rand_seed();

    int game_started = 0;

    flappy_reset_game();

    while (1)
    {
        game_button_t btn = game_poll_button(FRAME_MS);

        if (btn == GAME_BTN_BACK)
            break;

        if (!game_started)
        {
            /* Title screen */
            if (btn == GAME_BTN_OK)
            {
                game_started = 1;
                flappy_reset_game();
                m1_buzzer_notification();
            }

            m1_u8g2_firstpage();
            do {
                u8g2_SetDrawColor(&m1_u8g2, 1);
                u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
                u8g2_DrawStr(&m1_u8g2, 18, 20, "Flappy Bird");

                u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
                u8g2_DrawStr(&m1_u8g2, 16, 38, "Press OK to fly");

                if (best_score > 0)
                {
                    char best[20];
                    int pos = 0;
                    u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
                    pos = flappy_append_str(best, pos, "Best: ");
                    pos = flappy_append_int(best, pos, best_score);
                    best[pos] = '\0';
                    u8g2_DrawStr(&m1_u8g2, 42, 52, best);
                }

                /* Draw a little bird preview */
                u8g2_DrawBox(&m1_u8g2, 60, 42, BIRD_W, BIRD_H);

                flappy_draw_ground();
            } while (m1_u8g2_nextpage());

            continue;
        }

        if (alive)
        {
            int flap = (btn == GAME_BTN_OK || btn == GAME_BTN_UP);
            flappy_update(flap);

            if (!alive)
            {
                if (score > best_score)
                    best_score = score;
                m1_buzzer_notification2();
            }
        }
        else
        {
            /* Dead — OK to restart */
            if (btn == GAME_BTN_OK)
            {
                flappy_reset_game();
                m1_buzzer_notification();
            }
        }

        /* Draw */
        m1_u8g2_firstpage();
        do {
            u8g2_SetDrawColor(&m1_u8g2, 1);

            flappy_draw_pipes();
            flappy_draw_bird();
            flappy_draw_ground();

            /* Score */
            char sc[8];
            int pos = flappy_append_int(sc, 0, score);
            sc[pos] = '\0';
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 60, 10, sc);

            if (!alive)
            {
                /* Game over overlay */
                u8g2_DrawFrame(&m1_u8g2, 20, 18, 88, 30);
                u8g2_SetDrawColor(&m1_u8g2, 0);
                u8g2_DrawBox(&m1_u8g2, 21, 19, 86, 28);
                u8g2_SetDrawColor(&m1_u8g2, 1);

                u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
                u8g2_DrawStr(&m1_u8g2, 30, 30, "Game Over!");

                char final_sc[24];
                int fpos = 0;
                fpos = flappy_append_str(final_sc, fpos, "Score:");
                fpos = flappy_append_int(final_sc, fpos, score);
                fpos = flappy_append_str(final_sc, fpos, " Best:");
                fpos = flappy_append_int(final_sc, fpos, best_score);
                final_sc[fpos] = '\0';
                u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
                u8g2_DrawStr(&m1_u8g2, 24, 42, final_sc);
            }

        } while (m1_u8g2_nextpage());
    }
}
