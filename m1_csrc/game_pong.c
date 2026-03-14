/* See COPYING.txt for license details. */

/*
*
* game_pong.c
*
* Pong game — player vs AI
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

/* Court dimensions */
#define COURT_X         0
#define COURT_Y         0
#define COURT_W         128
#define COURT_H         64
#define BORDER          1

/* Play area (inside border) */
#define PLAY_LEFT       (COURT_X + BORDER)
#define PLAY_RIGHT      (COURT_X + COURT_W - BORDER)
#define PLAY_TOP        (COURT_Y + BORDER + 8)   /* Leave room for score */
#define PLAY_BOTTOM     (COURT_Y + COURT_H - BORDER)

/* Paddle dimensions */
#define PADDLE_W        3
#define PADDLE_H        14
#define PADDLE_MARGIN   4      /* Distance from edge to paddle */
#define PADDLE_SPEED    5

/* Player paddle x position */
#define PLAYER_X        (PLAY_LEFT + PADDLE_MARGIN)
/* AI paddle x position */
#define AI_X            (PLAY_RIGHT - PADDLE_MARGIN - PADDLE_W)

/* Ball */
#define BALL_SIZE       6
#define BALL_SPEED_INIT 1
#define BALL_SPEED_MAX  3

/* Scoring */
#define WIN_SCORE       7

/* Frame timing */
#define FRAME_MS        40

//************************** S T R U C T U R E S *******************************

typedef struct {
    int16_t x;
    int16_t y;
    int16_t dx;
    int16_t dy;
    int16_t speed;
} pong_ball_t;

typedef struct {
    int16_t y;
} pong_paddle_t;

typedef struct {
    pong_ball_t    ball;
    pong_paddle_t  player;
    pong_paddle_t  ai;
    uint8_t        score_player;
    uint8_t        score_ai;
    bool           running;
    bool           serving;
    int8_t         serve_dir;   /* -1 = serve left (to player), +1 = serve right (to AI) */
    int16_t        ai_target_y;
    int16_t        ai_error;
} pong_state_t;

/***************************** V A R I A B L E S ******************************/

static pong_state_t g_pong;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void pong_reset_ball(pong_state_t *st);
static void pong_init(pong_state_t *st);
static void pong_update(pong_state_t *st, game_button_t btn);
static void pong_draw(pong_state_t *st);
static void pong_draw_court(void);
static void pong_draw_scores(pong_state_t *st);
static void pong_draw_center_line(void);
static bool pong_title_screen(void);
static bool pong_winner_screen(pong_state_t *st);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Reset the ball to center and prepare for a serve
 */
/*============================================================================*/
static void pong_reset_ball(pong_state_t *st)
{
    st->ball.x = COURT_W / 2 - BALL_SIZE / 2;
    st->ball.y = (PLAY_TOP + PLAY_BOTTOM) / 2 - BALL_SIZE / 2;
    st->ball.dx = 0;
    st->ball.dy = 0;
    st->serving = true;

    /* Alternate serve direction */
    st->serve_dir = -st->serve_dir;
}


/*============================================================================*/
/*
 * @brief  Launch the ball after a serve
 */
/*============================================================================*/
static void pong_serve(pong_state_t *st)
{
    st->ball.speed = BALL_SPEED_INIT;
    st->ball.dx = (st->serve_dir > 0) ? st->ball.speed : -st->ball.speed;
    st->ball.dy = game_rand_range(-2, 2);
    if (st->ball.dy == 0)
    {
        st->ball.dy = 1;
    }
    st->serving = false;
}


/*============================================================================*/
/*
 * @brief  Initialize a new game
 */
/*============================================================================*/
static void pong_init(pong_state_t *st)
{
    memset(st, 0, sizeof(*st));

    int16_t center_y = (PLAY_TOP + PLAY_BOTTOM) / 2 - PADDLE_H / 2;
    st->player.y = center_y;
    st->ai.y = center_y;
    st->score_player = 0;
    st->score_ai = 0;
    st->running = true;
    st->serve_dir = 1;
    st->ai_target_y = center_y;
    st->ai_error = 0;

    pong_reset_ball(st);
}


/*============================================================================*/
/*
 * @brief  Clamp a paddle Y position to stay within play area
 */
/*============================================================================*/
static int16_t pong_clamp_paddle(int16_t y)
{
    if (y < PLAY_TOP)
    {
        y = PLAY_TOP;
    }
    if (y > PLAY_BOTTOM - PADDLE_H)
    {
        y = PLAY_BOTTOM - PADDLE_H;
    }
    return y;
}


/*============================================================================*/
/*
 * @brief  AI logic — track the ball with slight delay and error
 */
/*============================================================================*/
static void pong_update_ai(pong_state_t *st)
{
    /* Only actively track when the ball is moving toward the AI */
    if (st->ball.dx > 0)
    {
        /* Recalculate target occasionally with some error */
        static uint8_t ai_tick = 0;
        ai_tick++;
        if (ai_tick >= 5)
        {
            ai_tick = 0;
            st->ai_error = (int16_t)game_rand_range(-4, 4);
            st->ai_target_y = st->ball.y + st->ai_error - PADDLE_H / 2;
        }

        /* Move toward target at slightly less than paddle speed */
        int16_t diff = st->ai_target_y - st->ai.y;
        if (diff > 2)
        {
            st->ai.y += 2;
        }
        else if (diff < -2)
        {
            st->ai.y -= 2;
        }
    }
    else
    {
        /* Ball moving away — drift toward center */
        int16_t center = (PLAY_TOP + PLAY_BOTTOM) / 2 - PADDLE_H / 2;
        int16_t diff = center - st->ai.y;
        if (diff > 1)
        {
            st->ai.y += 1;
        }
        else if (diff < -1)
        {
            st->ai.y -= 1;
        }
    }

    st->ai.y = pong_clamp_paddle(st->ai.y);
}


/*============================================================================*/
/*
 * @brief  Check if ball overlaps a paddle and bounce
 * @retval true if a collision occurred
 */
/*============================================================================*/
static bool pong_check_paddle_collision(pong_ball_t *ball, int16_t px, int16_t py)
{
    /* Ball bounding box */
    int16_t bx1 = ball->x;
    int16_t by1 = ball->y;
    int16_t bx2 = ball->x + BALL_SIZE;
    int16_t by2 = ball->y + BALL_SIZE;

    /* Paddle bounding box */
    int16_t px1 = px;
    int16_t py1 = py;
    int16_t px2 = px + PADDLE_W;
    int16_t py2 = py + PADDLE_H;

    if (bx2 >= px1 && bx1 <= px2 && by2 >= py1 && by1 <= py2)
    {
        /* Reverse horizontal direction */
        ball->dx = -ball->dx;

        /* Adjust vertical speed based on where ball hits paddle */
        int16_t paddle_center = py + PADDLE_H / 2;
        int16_t ball_center = ball->y + BALL_SIZE / 2;
        int16_t offset = ball_center - paddle_center;

        /* Map offset to dy: top of paddle = negative, bottom = positive */
        ball->dy = offset / 2;
        if (ball->dy == 0)
        {
            ball->dy = (game_rand_range(0, 1) == 0) ? 1 : -1;
        }

        /* Push ball out of paddle to prevent re-collision */
        if (ball->dx > 0)
        {
            ball->x = px2 + 1;
        }
        else
        {
            ball->x = px1 - BALL_SIZE - 1;
        }

        return true;
    }
    return false;
}


/*============================================================================*/
/*
 * @brief  Update game state for one frame
 */
/*============================================================================*/
static void pong_update(pong_state_t *st, game_button_t btn)
{
    /* Handle player paddle movement */
    if (btn == GAME_BTN_UP)
    {
        st->player.y -= PADDLE_SPEED;
    }
    else if (btn == GAME_BTN_DOWN)
    {
        st->player.y += PADDLE_SPEED;
    }
    else if (btn == GAME_BTN_OK && st->serving)
    {
        pong_serve(st);
    }
    st->player.y = pong_clamp_paddle(st->player.y);

    /* If serving, don't move ball */
    if (st->serving)
    {
        pong_update_ai(st);
        return;
    }

    /* Move ball */
    st->ball.x += st->ball.dx;
    st->ball.y += st->ball.dy;

    /* Bounce off top and bottom walls */
    if (st->ball.y <= PLAY_TOP)
    {
        st->ball.y = PLAY_TOP;
        st->ball.dy = -st->ball.dy;
    }
    if (st->ball.y + BALL_SIZE >= PLAY_BOTTOM)
    {
        st->ball.y = PLAY_BOTTOM - BALL_SIZE;
        st->ball.dy = -st->ball.dy;
    }

    /* Check paddle collisions */
    if (pong_check_paddle_collision(&st->ball, PLAYER_X, st->player.y))
    {
        /* Slight speed increase after player hit */
        if (st->ball.speed < BALL_SPEED_MAX)
        {
            st->ball.speed++;
        }
        /* Ensure ball moves right */
        if (st->ball.dx < 0)
        {
            st->ball.dx = -st->ball.dx;
        }
        m1_buzzer_set(800, 30);
    }

    if (pong_check_paddle_collision(&st->ball, AI_X, st->ai.y))
    {
        /* Ensure ball moves left */
        if (st->ball.dx > 0)
        {
            st->ball.dx = -st->ball.dx;
        }
        m1_buzzer_set(600, 30);
    }

    /* Check scoring — ball past left edge */
    if (st->ball.x + BALL_SIZE < PLAY_LEFT)
    {
        st->score_ai++;
        m1_buzzer_set(200, 150);
        if (st->score_ai >= WIN_SCORE)
        {
            st->running = false;
        }
        else
        {
            pong_reset_ball(st);
        }
    }

    /* Check scoring — ball past right edge */
    if (st->ball.x > PLAY_RIGHT)
    {
        st->score_player++;
        m1_buzzer_set(400, 150);
        if (st->score_player >= WIN_SCORE)
        {
            st->running = false;
        }
        else
        {
            pong_reset_ball(st);
        }
    }

    /* Update AI */
    pong_update_ai(st);
}


/*============================================================================*/
/*
 * @brief  Draw the dashed center line
 */
/*============================================================================*/
static void pong_draw_center_line(void)
{
    int16_t x = COURT_W / 2;
    for (int16_t y = PLAY_TOP; y < PLAY_BOTTOM; y += 4)
    {
        u8g2_DrawPixel(&m1_u8g2, x, y);
        u8g2_DrawPixel(&m1_u8g2, x, y + 1);
    }
}


/*============================================================================*/
/*
 * @brief  Draw the score at the top of the screen
 */
/*============================================================================*/
static void pong_draw_scores(pong_state_t *st)
{
    char buf[8];

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);

    /* Player score on left side */
    buf[0] = '0' + st->score_player;
    buf[1] = '\0';
    u8g2_DrawStr(&m1_u8g2, COURT_W / 2 - 16, 8, buf);

    /* AI score on right side */
    buf[0] = '0' + st->score_ai;
    buf[1] = '\0';
    u8g2_DrawStr(&m1_u8g2, COURT_W / 2 + 12, 8, buf);
}


/*============================================================================*/
/*
 * @brief  Draw the court borders
 */
/*============================================================================*/
static void pong_draw_court(void)
{
    /* Top border (below score area) */
    u8g2_DrawHLine(&m1_u8g2, COURT_X, PLAY_TOP - 1, COURT_W);
    /* Bottom border */
    u8g2_DrawHLine(&m1_u8g2, COURT_X, PLAY_BOTTOM, COURT_W);
}


/*============================================================================*/
/*
 * @brief  Draw the full game scene
 */
/*============================================================================*/
static void pong_draw(pong_state_t *st)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Scores */
    pong_draw_scores(st);

    /* Court */
    pong_draw_court();
    pong_draw_center_line();

    /* Player paddle */
    u8g2_DrawBox(&m1_u8g2, PLAYER_X, st->player.y, PADDLE_W, PADDLE_H);

    /* AI paddle */
    u8g2_DrawBox(&m1_u8g2, AI_X, st->ai.y, PADDLE_W, PADDLE_H);

    /* Ball */
    u8g2_DrawBox(&m1_u8g2, st->ball.x, st->ball.y, BALL_SIZE, BALL_SIZE);

    /* Serve prompt */
    if (st->serving)
    {
        u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 36, 38, "Press OK");
    }

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Display the title screen
 * @retval true if user wants to play, false if BACK pressed
 */
/*============================================================================*/
static bool pong_title_screen(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 30, 22, "PONG");

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 18, 38, "First to 7 wins");

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 18, 52, "OK=Play  BACK=Exit");

    m1_u8g2_nextpage();

    /* Wait for input */
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
 * @brief  Display the winner screen
 * @retval true if user wants to restart, false if BACK pressed
 */
/*============================================================================*/
static bool pong_winner_screen(pong_state_t *st)
{
    const char *msg;
    if (st->score_player >= WIN_SCORE)
    {
        msg = "YOU WIN!";
    }
    else
    {
        msg = "CPU WINS";
    }

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 14, 24, msg);

    /* Final score */
    char score_buf[16];
    score_buf[0] = '0' + st->score_player;
    score_buf[1] = ' ';
    score_buf[2] = '-';
    score_buf[3] = ' ';
    score_buf[4] = '0' + st->score_ai;
    score_buf[5] = '\0';
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 48, 40, score_buf);

    u8g2_SetFont(&m1_u8g2, M1_DISP_SUB_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 10, 56, "OK=Again  BACK=Exit");

    m1_u8g2_nextpage();

    if (st->score_player >= WIN_SCORE)
    {
        m1_buzzer_notification();
    }

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
 * @brief  Main entry point for Pong game
 *         Runs its own event loop; returns when user presses BACK
 */
/*============================================================================*/
void game_pong_run(void)
{
    game_rand_seed();

    /* Title screen */
    if (!pong_title_screen())
    {
        return;
    }

restart:
    pong_init(&g_pong);

    while (g_pong.running)
    {
        game_button_t btn = game_poll_button(FRAME_MS);

        if (btn == GAME_BTN_BACK)
        {
            return;
        }

        pong_update(&g_pong, btn);
        pong_draw(&g_pong);
    }

    /* Someone won — show winner screen */
    if (pong_winner_screen(&g_pong))
    {
        goto restart;
    }
}
