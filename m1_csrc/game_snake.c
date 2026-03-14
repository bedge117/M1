/* See COPYING.txt for license details. */

/*
*
* game_snake.c
*
* Classic snake game for M1
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

#define SNAKE_GRID_W        32
#define SNAKE_GRID_H        16
#define SNAKE_CELL_SIZE     4

/* Maximum snake length = entire grid */
#define SNAKE_MAX_LEN       (SNAKE_GRID_W * SNAKE_GRID_H)

#define SNAKE_INITIAL_LEN   3
#define SNAKE_BASE_DELAY_MS 120
#define SNAKE_MIN_DELAY_MS  50
#define SNAKE_SPEED_STEP    2    /* ms faster per food eaten */

/* Directions */
#define DIR_UP      0
#define DIR_DOWN    1
#define DIR_LEFT    2
#define DIR_RIGHT   3

//************************** S T R U C T U R E S *******************************

typedef struct {
    uint8_t x;
    uint8_t y;
} snake_pos_t;

typedef struct {
    snake_pos_t body[SNAKE_MAX_LEN];
    uint16_t    length;
    uint8_t     dir;
    snake_pos_t food;
    uint16_t    score;
    uint16_t    delay_ms;
    bool        game_over;
    bool        running;
} snake_state_t;

/***************************** V A R I A B L E S ******************************/

static snake_state_t snake;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void snake_init(void);
static void snake_place_food(void);
static void snake_update(void);
static void snake_draw(void);
static void snake_draw_title(void);
static void snake_draw_game_over(void);
static bool snake_cell_occupied(uint8_t x, uint8_t y);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Check if a grid cell is occupied by the snake body
 */
/*============================================================================*/
static bool snake_cell_occupied(uint8_t x, uint8_t y)
{
    for (uint16_t i = 0; i < snake.length; i++)
    {
        if (snake.body[i].x == x && snake.body[i].y == y)
        {
            return true;
        }
    }
    return false;
}


/*============================================================================*/
/*
 * @brief  Place food at a random unoccupied cell
 */
/*============================================================================*/
static void snake_place_food(void)
{
    uint8_t x, y;
    uint16_t attempts = 0;

    do {
        x = (uint8_t)game_rand_range(0, SNAKE_GRID_W - 1);
        y = (uint8_t)game_rand_range(0, SNAKE_GRID_H - 1);
        attempts++;
        if (attempts > 500)
        {
            /* Grid is nearly full, just place it somewhere */
            break;
        }
    } while (snake_cell_occupied(x, y));

    snake.food.x = x;
    snake.food.y = y;
}


/*============================================================================*/
/*
 * @brief  Initialize the snake game state
 */
/*============================================================================*/
static void snake_init(void)
{
    memset(&snake, 0, sizeof(snake));

    snake.length = SNAKE_INITIAL_LEN;
    snake.dir = DIR_RIGHT;
    snake.score = 0;
    snake.delay_ms = SNAKE_BASE_DELAY_MS;
    snake.game_over = false;
    snake.running = true;

    /* Place snake in center, going right */
    uint8_t start_x = SNAKE_GRID_W / 2 - SNAKE_INITIAL_LEN / 2;
    uint8_t start_y = SNAKE_GRID_H / 2;

    for (uint16_t i = 0; i < SNAKE_INITIAL_LEN; i++)
    {
        /* body[0] = head, body[len-1] = tail */
        snake.body[i].x = start_x + (SNAKE_INITIAL_LEN - 1 - i);
        snake.body[i].y = start_y;
    }

    game_rand_seed();
    snake_place_food();
}


/*============================================================================*/
/*
 * @brief  Update the snake position (one tick)
 */
/*============================================================================*/
static void snake_update(void)
{
    if (snake.game_over)
    {
        return;
    }

    /* Calculate new head position */
    snake_pos_t new_head;
    new_head.x = snake.body[0].x;
    new_head.y = snake.body[0].y;

    switch (snake.dir)
    {
        case DIR_UP:    new_head.y--; break;
        case DIR_DOWN:  new_head.y++; break;
        case DIR_LEFT:  new_head.x--; break;
        case DIR_RIGHT: new_head.x++; break;
    }

    /* Check wall collision */
    if (new_head.x >= SNAKE_GRID_W || new_head.y >= SNAKE_GRID_H)
    {
        snake.game_over = true;
        return;
    }

    /* Check self collision (skip tail since it will move) */
    for (uint16_t i = 0; i < snake.length - 1; i++)
    {
        if (snake.body[i].x == new_head.x && snake.body[i].y == new_head.y)
        {
            snake.game_over = true;
            return;
        }
    }

    /* Check food */
    bool ate_food = (new_head.x == snake.food.x && new_head.y == snake.food.y);

    if (ate_food)
    {
        /* Grow: don't remove tail */
        if (snake.length < SNAKE_MAX_LEN)
        {
            snake.length++;
        }
        snake.score += 10;

        /* Speed up */
        if (snake.delay_ms > SNAKE_MIN_DELAY_MS)
        {
            snake.delay_ms -= SNAKE_SPEED_STEP;
            if (snake.delay_ms < SNAKE_MIN_DELAY_MS)
            {
                snake.delay_ms = SNAKE_MIN_DELAY_MS;
            }
        }
    }

    /* Shift body: move each segment to the position of the one in front */
    for (uint16_t i = snake.length - 1; i > 0; i--)
    {
        snake.body[i] = snake.body[i - 1];
    }
    snake.body[0] = new_head;

    if (ate_food)
    {
        snake_place_food();
        m1_buzzer_set(BUZZER_FREQ_04_KHZ, 30);
    }
}


/*============================================================================*/
/*
 * @brief  Draw the snake game screen
 */
/*============================================================================*/
static void snake_draw(void)
{
    char buf[16];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Draw border */
    u8g2_DrawFrame(&m1_u8g2, 0, 0, GAME_SCREEN_W, GAME_SCREEN_H);

    /* Draw food as a filled square with a dot pattern */
    u8g2_DrawBox(&m1_u8g2,
                 snake.food.x * SNAKE_CELL_SIZE,
                 snake.food.y * SNAKE_CELL_SIZE,
                 SNAKE_CELL_SIZE, SNAKE_CELL_SIZE);

    /* Draw snake body — 3x3 boxes (smaller than cell = visible gaps) */
    for (uint16_t i = 1; i < snake.length; i++)
    {
        uint8_t px = snake.body[i].x * SNAKE_CELL_SIZE;
        uint8_t py = snake.body[i].y * SNAKE_CELL_SIZE;
        u8g2_DrawBox(&m1_u8g2, px, py, SNAKE_CELL_SIZE - 1, SNAKE_CELL_SIZE - 1);
    }

    /* Draw head — full 4x4 box (visibly larger than 3x3 body) */
    {
        uint8_t px = snake.body[0].x * SNAKE_CELL_SIZE;
        uint8_t py = snake.body[0].y * SNAKE_CELL_SIZE;
        u8g2_DrawBox(&m1_u8g2, px, py, SNAKE_CELL_SIZE, SNAKE_CELL_SIZE);
    }

    /* Draw score at top-right, small font so it doesn't overlap much */
    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);

    /* Use snprintf to format score */
    buf[0] = '\0';
    int val = snake.score;
    int pos = 0;
    if (val == 0)
    {
        buf[0] = '0';
        buf[1] = '\0';
    }
    else
    {
        char tmp[8];
        int tpos = 0;
        while (val > 0 && tpos < 7)
        {
            tmp[tpos++] = '0' + (val % 10);
            val /= 10;
        }
        for (int j = tpos - 1; j >= 0; j--)
        {
            buf[pos++] = tmp[j];
        }
        buf[pos] = '\0';
    }

    uint8_t sw = u8g2_GetStrWidth(&m1_u8g2, buf);
    u8g2_DrawStr(&m1_u8g2, GAME_SCREEN_W - sw - 2, 7, buf);

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Draw the title screen
 */
/*============================================================================*/
static void snake_draw_title(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 30, 25, "SNAKE");

    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    u8g2_DrawStr(&m1_u8g2, 22, 42, "Press OK to start");
    u8g2_DrawStr(&m1_u8g2, 25, 56, "BACK to go back");

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Draw the game over screen
 */
/*============================================================================*/
static void snake_draw_game_over(void)
{
    char buf[24];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 8, 20, "GAME OVER");

    /* Format score string manually */
    memcpy(buf, "Score: ", 7);
    int val = snake.score;
    int pos = 7;
    if (val == 0)
    {
        buf[pos++] = '0';
    }
    else
    {
        char tmp[8];
        int tpos = 0;
        while (val > 0 && tpos < 7)
        {
            tmp[tpos++] = '0' + (val % 10);
            val /= 10;
        }
        for (int j = tpos - 1; j >= 0; j--)
        {
            buf[pos++] = tmp[j];
        }
    }
    buf[pos] = '\0';

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    uint8_t sw = u8g2_GetStrWidth(&m1_u8g2, buf);
    u8g2_DrawStr(&m1_u8g2, (GAME_SCREEN_W - sw) / 2, 38, buf);

    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    u8g2_DrawStr(&m1_u8g2, 10, 56, "OK:Retry  BACK:Exit");

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Main snake game entry point. Runs own event loop, returns on BACK.
 */
/*============================================================================*/
void game_snake_run(void)
{
    game_button_t btn;

    /* Title screen */
    snake_draw_title();
    while (1)
    {
        btn = game_poll_button(500);
        if (btn == GAME_BTN_BACK)
        {
            return;
        }
        if (btn == GAME_BTN_OK)
        {
            break;
        }
    }

restart:
    snake_init();

    /* Main game loop */
    while (snake.running)
    {
        /* Poll input with game tick delay */
        btn = game_poll_button(snake.delay_ms);

        if (btn == GAME_BTN_BACK)
        {
            return;
        }

        /* Process direction changes (prevent 180-degree reversal) */
        switch (btn)
        {
            case GAME_BTN_UP:
                if (snake.dir != DIR_DOWN)  snake.dir = DIR_UP;
                break;
            case GAME_BTN_DOWN:
                if (snake.dir != DIR_UP)    snake.dir = DIR_DOWN;
                break;
            case GAME_BTN_LEFT:
                if (snake.dir != DIR_RIGHT) snake.dir = DIR_LEFT;
                break;
            case GAME_BTN_RIGHT:
                if (snake.dir != DIR_LEFT)  snake.dir = DIR_RIGHT;
                break;
            default:
                break;
        }

        /* Update game state */
        snake_update();

        /* Draw */
        if (snake.game_over)
        {
            m1_buzzer_set(BUZZER_FREQ_01_KHZ, 200);
            snake_draw_game_over();

            /* Wait for OK (restart) or BACK (exit) */
            while (1)
            {
                btn = game_poll_button(500);
                if (btn == GAME_BTN_BACK)
                {
                    return;
                }
                if (btn == GAME_BTN_OK)
                {
                    goto restart;
                }
            }
        }

        snake_draw();
    }
}
