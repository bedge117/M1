/* See COPYING.txt for license details. */

/*
*
* game_dice.c
*
* Animated dice roll game
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

/* Die face size and rendering */
#define DIE_SIZE        24
#define DIE_CORNER_R    2
#define DOT_R           2       /* Dot radius for filled circles */

/* Positioning for 1 die (centered) */
#define DIE1_SINGLE_X   52
#define DIE1_SINGLE_Y   6

/* Positioning for 2 dice (side by side) */
#define DIE1_DUAL_X     20
#define DIE2_DUAL_X     84
#define DIE_DUAL_Y      6

/* Roll history */
#define HISTORY_MAX     5

/* Animation */
#define ANIM_FRAMES     20      /* Number of random faces shown during roll */
#define ANIM_FRAME_MS   50      /* Duration per animation frame */

/* Bounce animation offsets (pixels up/down from base Y) */
static const int8_t bounce_offsets[] = {
    -4, -3, -1, 0, -3, -2, 0, -2, -1, 0,
    -2, -1, 0, -1, 0, 0, -1, 0, 0, 0
};

//************************** S T R U C T U R E S *******************************

typedef struct {
    uint8_t num_dice;           /* 1 or 2 */
    uint8_t die_val[2];         /* Current face values 1-6 */
    uint8_t history[HISTORY_MAX];  /* History of totals */
    uint8_t history_count;
    bool    rolling;
    uint8_t anim_frame;
} dice_state_t;

/***************************** V A R I A B L E S ******************************/

static dice_state_t g_dice;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void dice_init(dice_state_t *st);
static void dice_draw_die(int16_t x, int16_t y, uint8_t value);
static void dice_draw_dot(int16_t cx, int16_t cy);
static void dice_draw(dice_state_t *st, int8_t bounce1, int8_t bounce2);
static void dice_add_history(dice_state_t *st, uint8_t total);
static void dice_roll_animation(dice_state_t *st);
static bool dice_title_screen(void);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Initialize dice game state
 */
/*============================================================================*/
static void dice_init(dice_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->num_dice = 1;
    st->die_val[0] = 1;
    st->die_val[1] = 1;
    st->history_count = 0;
    st->rolling = false;
}


/*============================================================================*/
/*
 * @brief  Draw a single dot (pip) at center position
 */
/*============================================================================*/
static void dice_draw_dot(int16_t cx, int16_t cy)
{
    u8g2_DrawDisc(&m1_u8g2, cx, cy, DOT_R, U8G2_DRAW_ALL);
}


/*============================================================================*/
/*
 * @brief  Draw one die face at the given position
 * @param  x      Top-left x of the die
 * @param  y      Top-left y of the die
 * @param  value  Face value 1-6
 */
/*============================================================================*/
static void dice_draw_die(int16_t x, int16_t y, uint8_t value)
{
    /* Draw die outline (rounded rectangle via frame + filled corners) */
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    u8g2_DrawFrame(&m1_u8g2, x, y, DIE_SIZE, DIE_SIZE);
    u8g2_DrawFrame(&m1_u8g2, x + 1, y + 1, DIE_SIZE - 2, DIE_SIZE - 2);

    /*
     * Dot positions within the die face:
     *   TL  TC  TR       (top-left, top-center, top-right)
     *   ML  MC  MR       (mid-left, mid-center, mid-right)
     *   BL  BC  BR       (bot-left, bot-center, bot-right)
     */
    int16_t cx = x + DIE_SIZE / 2;     /* Center x */
    int16_t cy = y + DIE_SIZE / 2;     /* Center y */
    int16_t off = 7;                    /* Offset from center to dot positions */

    int16_t tl_x = cx - off, tl_y = cy - off;
    int16_t tr_x = cx + off, tr_y = cy - off;
    int16_t ml_x = cx - off, ml_y = cy;
    int16_t mr_x = cx + off, mr_y = cy;
    int16_t bl_x = cx - off, bl_y = cy + off;
    int16_t br_x = cx + off, br_y = cy + off;

    switch (value)
    {
        case 1:
            dice_draw_dot(cx, cy);
            break;
        case 2:
            dice_draw_dot(tr_x, tr_y);
            dice_draw_dot(bl_x, bl_y);
            break;
        case 3:
            dice_draw_dot(tr_x, tr_y);
            dice_draw_dot(cx, cy);
            dice_draw_dot(bl_x, bl_y);
            break;
        case 4:
            dice_draw_dot(tl_x, tl_y);
            dice_draw_dot(tr_x, tr_y);
            dice_draw_dot(bl_x, bl_y);
            dice_draw_dot(br_x, br_y);
            break;
        case 5:
            dice_draw_dot(tl_x, tl_y);
            dice_draw_dot(tr_x, tr_y);
            dice_draw_dot(cx, cy);
            dice_draw_dot(bl_x, bl_y);
            dice_draw_dot(br_x, br_y);
            break;
        case 6:
            dice_draw_dot(tl_x, tl_y);
            dice_draw_dot(tr_x, tr_y);
            dice_draw_dot(ml_x, ml_y);
            dice_draw_dot(mr_x, mr_y);
            dice_draw_dot(bl_x, bl_y);
            dice_draw_dot(br_x, br_y);
            break;
        default:
            break;
    }
}


/*============================================================================*/
/*
 * @brief  Draw the full dice screen
 * @param  bounce1  Vertical offset for die 1 (animation bounce)
 * @param  bounce2  Vertical offset for die 2 (animation bounce)
 */
/*============================================================================*/
static void dice_draw(dice_state_t *st, int8_t bounce1, int8_t bounce2)
{
    char buf[32];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    if (st->num_dice == 1)
    {
        /* Single die centered */
        dice_draw_die(DIE1_SINGLE_X, DIE1_SINGLE_Y + bounce1, st->die_val[0]);

        /* Total below die */
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        buf[0] = '0' + st->die_val[0];
        buf[1] = '\0';
        u8g2_DrawStr(&m1_u8g2, 62, 42, buf);
    }
    else
    {
        /* Two dice side by side */
        dice_draw_die(DIE1_DUAL_X, DIE_DUAL_Y + bounce1, st->die_val[0]);
        dice_draw_die(DIE2_DUAL_X, DIE_DUAL_Y + bounce2, st->die_val[1]);

        /* Total between and below dice */
        uint8_t total = st->die_val[0] + st->die_val[1];
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        if (total >= 10)
        {
            buf[0] = '1';
            buf[1] = '0' + (total - 10);
            buf[2] = '\0';
        }
        else
        {
            buf[0] = '0' + total;
            buf[1] = '\0';
        }
        u8g2_DrawStr(&m1_u8g2, 60, 42, buf);
    }

    /* Dice count indicator */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    if (st->num_dice == 1)
    {
        u8g2_DrawStr(&m1_u8g2, 0, 8, "1 die");
    }
    else
    {
        u8g2_DrawStr(&m1_u8g2, 0, 8, "2 dice");
    }

    /* Roll history at the bottom */
    if (st->history_count > 0)
    {
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);

        /* Build history string */
        int pos = 0;
        buf[pos++] = 'H';
        buf[pos++] = ':';

        uint8_t start = 0;
        if (st->history_count > HISTORY_MAX)
        {
            start = st->history_count - HISTORY_MAX;
        }

        uint8_t shown = 0;
        for (uint8_t i = start; i < st->history_count && shown < HISTORY_MAX; i++, shown++)
        {
            if (shown > 0)
            {
                buf[pos++] = ' ';
            }
            uint8_t val = st->history[shown];
            if (val >= 10)
            {
                buf[pos++] = '1';
                buf[pos++] = '0' + (val - 10);
            }
            else
            {
                buf[pos++] = '0' + val;
            }
        }
        buf[pos] = '\0';
        u8g2_DrawStr(&m1_u8g2, 0, 54, buf);
    }

    /* Controls hint */
    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 0, 63, "OK:Roll");
    u8g2_DrawStr(&m1_u8g2, 70, 63, "L/R:#dice");

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Add a roll result to history (circular buffer)
 */
/*============================================================================*/
static void dice_add_history(dice_state_t *st, uint8_t total)
{
    if (st->history_count < HISTORY_MAX)
    {
        st->history[st->history_count] = total;
        st->history_count++;
    }
    else
    {
        /* Shift history left */
        for (uint8_t i = 0; i < HISTORY_MAX - 1; i++)
        {
            st->history[i] = st->history[i + 1];
        }
        st->history[HISTORY_MAX - 1] = total;
    }
}


/*============================================================================*/
/*
 * @brief  Perform the rolling animation with bouncing dice
 */
/*============================================================================*/
static void dice_roll_animation(dice_state_t *st)
{
    st->rolling = true;

    for (uint8_t frame = 0; frame < ANIM_FRAMES; frame++)
    {
        /* Random face values during animation */
        st->die_val[0] = (uint8_t)game_rand_range(1, 6);
        st->die_val[1] = (uint8_t)game_rand_range(1, 6);

        /* Get bounce offset for this frame */
        int8_t b1 = bounce_offsets[frame];
        int8_t b2 = (frame < ANIM_FRAMES - 1) ? bounce_offsets[frame + 1] : 0;

        dice_draw(st, b1, b2);

        /* Tick sound during animation — gets slower as we near the end */
        if (frame < ANIM_FRAMES - 4)
        {
            m1_buzzer_set(1200 + game_rand_range(-200, 200), 15);
        }

        /* Slow down toward the end */
        uint32_t delay = ANIM_FRAME_MS;
        if (frame > ANIM_FRAMES - 6)
        {
            delay += (uint32_t)(frame - (ANIM_FRAMES - 6)) * 30;
        }
        vTaskDelay(pdMS_TO_TICKS(delay));
    }

    /* Final values */
    st->die_val[0] = (uint8_t)game_rand_range(1, 6);
    st->die_val[1] = (uint8_t)game_rand_range(1, 6);

    /* Settle sound */
    m1_buzzer_notification();

    /* Add to history */
    uint8_t total = st->die_val[0];
    if (st->num_dice == 2)
    {
        total += st->die_val[1];
    }
    dice_add_history(st, total);

    st->rolling = false;
}


/*============================================================================*/
/*
 * @brief  Display the title screen
 * @retval true if user wants to play, false if BACK pressed
 */
/*============================================================================*/
static bool dice_title_screen(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 20, 20, "DICE");

    /* Draw a small sample die */
    dice_draw_die(90, 4, 5);

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 10, 38, "Roll 1 or 2 dice");

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 18, 52, "OK=Play  BACK=Exit");

    m1_u8g2_nextpage();

    for (;;)
    {
        game_button_t btn = game_poll_button(500);
        if (btn == GAME_BTN_OK)
        {
            return true;
        }
        if (btn == GAME_BTN_BACK)
        {
            return false;
        }
    }
}


/*============================================================================*/
/*
 * @brief  Main entry point for Dice game
 *         Runs its own event loop; returns when user presses BACK
 */
/*============================================================================*/
void game_dice_run(void)
{
    game_rand_seed();

    /* Title screen */
    if (!dice_title_screen())
    {
        return;
    }

    dice_init(&g_dice);

    /* Initial draw */
    dice_draw(&g_dice, 0, 0);

    for (;;)
    {
        game_button_t btn = game_poll_button(200);

        if (btn == GAME_BTN_BACK)
        {
            return;
        }

        if (btn == GAME_BTN_OK)
        {
            dice_roll_animation(&g_dice);
            dice_draw(&g_dice, 0, 0);
        }
        else if (btn == GAME_BTN_LEFT)
        {
            if (g_dice.num_dice > 1)
            {
                g_dice.num_dice = 1;
                dice_draw(&g_dice, 0, 0);
            }
        }
        else if (btn == GAME_BTN_RIGHT)
        {
            if (g_dice.num_dice < 2)
            {
                g_dice.num_dice = 2;
                dice_draw(&g_dice, 0, 0);
            }
        }
    }
}
