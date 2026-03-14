/* See COPYING.txt for license details. */

/*
*
* game_trex.c
*
* T-Rex runner game (Chrome dino style) for M1
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

/* Ground line position (y from top) */
#define TREX_GROUND_Y       52
#define TREX_GROUND_LINE_Y  53

/* Dino dimensions and position */
#define DINO_W              10
#define DINO_H              12
#define DINO_DUCK_W         14
#define DINO_DUCK_H         7
#define DINO_X              12

/* Jump physics */
#define JUMP_VELOCITY       (-7)   /* initial upward velocity (negative = up) */
#define GRAVITY             1      /* per frame */

/* Cactus */
#define CACTUS_SMALL_W      6
#define CACTUS_SMALL_H      12
#define CACTUS_LARGE_W      8
#define CACTUS_LARGE_H      16

/* Bird */
#define BIRD_W              12
#define BIRD_H              8

/* Max obstacles on screen */
#define MAX_OBSTACLES       4

/* Game speed */
#define TREX_BASE_SPEED     2      /* pixels per frame */
#define TREX_MAX_SPEED      5
#define TREX_FRAME_MS       33     /* ~30 fps */
#define TREX_SPEED_INC      500    /* score interval to increase speed */

/* Min gap between obstacles in pixels */
#define OBSTACLE_MIN_GAP    40
#define OBSTACLE_MAX_GAP    80

//************************** S T R U C T U R E S *******************************

typedef enum {
    OBS_NONE = 0,
    OBS_CACTUS_SMALL,
    OBS_CACTUS_LARGE,
    OBS_BIRD_LOW,
    OBS_BIRD_HIGH
} obstacle_type_t;

typedef struct {
    obstacle_type_t type;
    int16_t x;
    int16_t y;
    uint8_t w;
    uint8_t h;
} obstacle_t;

typedef struct {
    /* Dino state */
    int16_t  dino_y;
    int8_t   dino_vy;        /* vertical velocity */
    bool     dino_jumping;
    bool     dino_ducking;

    /* Obstacles */
    obstacle_t obs[MAX_OBSTACLES];

    /* Game state */
    uint16_t score;
    uint16_t speed;           /* pixels per frame */
    uint16_t distance;        /* raw distance counter */
    uint16_t next_spawn_dist; /* distance until next obstacle */
    uint8_t  anim_frame;      /* for dino leg animation */
    bool     game_over;
    bool     running;
} trex_state_t;

/***************************** V A R I A B L E S ******************************/

static trex_state_t trex;

/*
 * Dino sprite: 10x12 pixels, standing pose
 * Simple pixel art of a T-Rex facing right
 */
static const uint8_t dino_sprite[] = {
    /* Row by row, 10 pixels wide, packed into bytes (MSB first, padded) */
    /* Encoding: 10 wide = 2 bytes per row, 12 rows */
    0x1F, 0x80,  /* ...XXXXX X....... */
    0x1F, 0xC0,  /* ...XXXXX XX...... */
    0x1F, 0x00,  /* ...XXXXX ........ */
    0x1F, 0x80,  /* ...XXXXX X....... */
    0x7F, 0x00,  /* .XXXXXXX ........ */
    0xFF, 0x00,  /* XXXXXXXX ........ */
    0xFF, 0x80,  /* XXXXXXXX X....... */
    0x7F, 0x00,  /* .XXXXXXX ........ */
    0x3E, 0x00,  /* ..XXXXX. ........ */
    0x1C, 0x00,  /* ...XXX.. ........ */
    0x1C, 0x00,  /* ...XXX.. ........ */
    0x14, 0x00,  /* ...X.X.. ........ */
};

/*
 * Dino ducking sprite: 14x7 pixels
 */
static const uint8_t dino_duck_sprite[] = {
    0x07, 0xE0,  /* .....XXXXXX..... */
    0x07, 0xF0,  /* .....XXXXXXX.... */
    0x07, 0xC0,  /* .....XXXXX...... */
    0x3F, 0xE0,  /* ..XXXXXXXXX..... */
    0x7F, 0xC0,  /* .XXXXXXXXX...... */
    0xFF, 0xC0,  /* XXXXXXXXXX...... */
    0x14, 0x00,  /* ...X.X.......... */
};

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void trex_init(void);
static void trex_update(game_button_t btn);
static void trex_spawn_obstacle(void);
static bool trex_check_collision(void);
static void trex_draw(void);
static void trex_draw_title(void);
static void trex_draw_game_over(void);
static void trex_draw_dino(void);
static void trex_draw_obstacle(const obstacle_t *obs);
static void trex_int_to_str(char *buf, int val);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Integer to string helper
 */
/*============================================================================*/
static void trex_int_to_str(char *buf, int val)
{
    int pos = 0;
    if (val == 0)
    {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }
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


/*============================================================================*/
/*
 * @brief  Initialize the T-Rex game state
 */
/*============================================================================*/
static void trex_init(void)
{
    memset(&trex, 0, sizeof(trex));

    trex.dino_y = TREX_GROUND_Y - DINO_H;
    trex.dino_vy = 0;
    trex.dino_jumping = false;
    trex.dino_ducking = false;
    trex.speed = TREX_BASE_SPEED;
    trex.score = 0;
    trex.distance = 0;
    trex.next_spawn_dist = 30;
    trex.game_over = false;
    trex.running = true;
    trex.anim_frame = 0;

    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        trex.obs[i].type = OBS_NONE;
    }

    game_rand_seed();
}


/*============================================================================*/
/*
 * @brief  Spawn a new obstacle at the right edge
 */
/*============================================================================*/
static void trex_spawn_obstacle(void)
{
    /* Find empty slot */
    int slot = -1;
    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        if (trex.obs[i].type == OBS_NONE)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0) return;

    /* Choose obstacle type */
    int rtype = game_rand_range(0, 99);
    obstacle_t *o = &trex.obs[slot];
    o->x = GAME_SCREEN_W + 4;

    if (trex.score >= 50 && rtype < 20)
    {
        /* Bird - only after score 50 */
        if (rtype < 10)
        {
            o->type = OBS_BIRD_LOW;
            o->y = TREX_GROUND_Y - BIRD_H - 2;
        }
        else
        {
            o->type = OBS_BIRD_HIGH;
            o->y = TREX_GROUND_Y - BIRD_H - 14;
        }
        o->w = BIRD_W;
        o->h = BIRD_H;
    }
    else if (rtype < 60)
    {
        o->type = OBS_CACTUS_SMALL;
        o->w = CACTUS_SMALL_W;
        o->h = CACTUS_SMALL_H;
        o->y = TREX_GROUND_Y - o->h;
    }
    else
    {
        o->type = OBS_CACTUS_LARGE;
        o->w = CACTUS_LARGE_W;
        o->h = CACTUS_LARGE_H;
        o->y = TREX_GROUND_Y - o->h;
    }

    /* Schedule next spawn */
    trex.next_spawn_dist = (uint16_t)game_rand_range(OBSTACLE_MIN_GAP, OBSTACLE_MAX_GAP);
    trex.distance = 0;
}


/*============================================================================*/
/*
 * @brief  Check collision between dino and all obstacles
 */
/*============================================================================*/
static bool trex_check_collision(void)
{
    int16_t dx = DINO_X;
    int16_t dy = trex.dino_y;
    uint8_t dw, dh;

    if (trex.dino_ducking)
    {
        dw = DINO_DUCK_W;
        dh = DINO_DUCK_H;
        dy = TREX_GROUND_Y - DINO_DUCK_H;
    }
    else
    {
        dw = DINO_W;
        dh = DINO_H;
    }

    /* Shrink hitbox slightly for fairness */
    int16_t dx1 = dx + 2;
    int16_t dy1 = dy + 2;
    int16_t dx2 = dx + dw - 2;
    int16_t dy2 = dy + dh - 2;

    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        if (trex.obs[i].type == OBS_NONE) continue;

        int16_t ox1 = trex.obs[i].x + 1;
        int16_t oy1 = trex.obs[i].y + 1;
        int16_t ox2 = trex.obs[i].x + trex.obs[i].w - 1;
        int16_t oy2 = trex.obs[i].y + trex.obs[i].h - 1;

        /* AABB overlap check */
        if (dx1 < ox2 && dx2 > ox1 && dy1 < oy2 && dy2 > oy1)
        {
            return true;
        }
    }
    return false;
}


/*============================================================================*/
/*
 * @brief  Update game state for one frame
 */
/*============================================================================*/
static void trex_update(game_button_t btn)
{
    if (trex.game_over) return;

    /* Handle input */
    if ((btn == GAME_BTN_UP || btn == GAME_BTN_OK) && !trex.dino_jumping)
    {
        trex.dino_jumping = true;
        trex.dino_ducking = false;
        trex.dino_vy = JUMP_VELOCITY;
    }

    if (btn == GAME_BTN_DOWN)
    {
        if (trex.dino_jumping)
        {
            /* Fast fall */
            trex.dino_vy += GRAVITY;
        }
        else
        {
            trex.dino_ducking = true;
        }
    }
    else
    {
        if (!trex.dino_jumping)
        {
            trex.dino_ducking = false;
        }
    }

    /* Apply gravity */
    if (trex.dino_jumping)
    {
        trex.dino_vy += GRAVITY;
        trex.dino_y += trex.dino_vy;

        int16_t ground_pos = TREX_GROUND_Y - DINO_H;
        if (trex.dino_y >= ground_pos)
        {
            trex.dino_y = ground_pos;
            trex.dino_vy = 0;
            trex.dino_jumping = false;
        }
    }

    /* Move obstacles */
    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        if (trex.obs[i].type == OBS_NONE) continue;

        trex.obs[i].x -= (int16_t)trex.speed;

        /* Remove if off-screen */
        if (trex.obs[i].x + trex.obs[i].w < 0)
        {
            trex.obs[i].type = OBS_NONE;
        }
    }

    /* Spawn logic */
    trex.distance += trex.speed;
    if (trex.distance >= trex.next_spawn_dist)
    {
        trex_spawn_obstacle();
    }

    /* Score */
    trex.score++;

    /* Speed increase */
    trex.speed = TREX_BASE_SPEED + (trex.score / TREX_SPEED_INC);
    if (trex.speed > TREX_MAX_SPEED)
    {
        trex.speed = TREX_MAX_SPEED;
    }

    /* Animation frame for leg movement */
    trex.anim_frame = (trex.anim_frame + 1) & 7;

    /* Check collision */
    if (trex_check_collision())
    {
        trex.game_over = true;
    }
}


/*============================================================================*/
/*
 * @brief  Draw the dino character
 */
/*============================================================================*/
static void trex_draw_dino(void)
{
    if (trex.dino_ducking && !trex.dino_jumping)
    {
        /* Draw ducking sprite using bitmap data */
        int16_t dy = TREX_GROUND_Y - DINO_DUCK_H;
        for (int r = 0; r < DINO_DUCK_H; r++)
        {
            uint16_t row_bits = ((uint16_t)dino_duck_sprite[r * 2] << 8) | dino_duck_sprite[r * 2 + 1];
            for (int c = 0; c < DINO_DUCK_W; c++)
            {
                if (row_bits & (0x8000 >> c))
                {
                    u8g2_DrawPixel(&m1_u8g2, DINO_X + c, dy + r);
                }
            }
        }

        /* Animate legs while ducking */
        if (trex.anim_frame < 4)
        {
            /* Alternate leg positions */
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            u8g2_DrawPixel(&m1_u8g2, DINO_X + 2, dy + DINO_DUCK_H - 1);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        }
        else
        {
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            u8g2_DrawPixel(&m1_u8g2, DINO_X + 4, dy + DINO_DUCK_H - 1);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        }
    }
    else
    {
        /* Draw standing/jumping sprite */
        int16_t dy = trex.dino_y;
        for (int r = 0; r < DINO_H; r++)
        {
            uint16_t row_bits = ((uint16_t)dino_sprite[r * 2] << 8) | dino_sprite[r * 2 + 1];
            for (int c = 0; c < DINO_W; c++)
            {
                if (row_bits & (0x8000 >> c))
                {
                    u8g2_DrawPixel(&m1_u8g2, DINO_X + c, dy + r);
                }
            }
        }

        /* Leg animation when running (not jumping) */
        if (!trex.dino_jumping)
        {
            if (trex.anim_frame < 4)
            {
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
                u8g2_DrawPixel(&m1_u8g2, DINO_X + 3, dy + DINO_H - 1);
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            }
            else
            {
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
                u8g2_DrawPixel(&m1_u8g2, DINO_X + 5, dy + DINO_H - 1);
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            }
        }
    }
}


/*============================================================================*/
/*
 * @brief  Draw a single obstacle
 */
/*============================================================================*/
static void trex_draw_obstacle(const obstacle_t *obs)
{
    if (obs->type == OBS_NONE) return;
    if (obs->x > GAME_SCREEN_W || obs->x + obs->w < 0) return;

    switch (obs->type)
    {
        case OBS_CACTUS_SMALL:
        {
            /* Small cactus — solid filled block */
            int16_t cx = obs->x;
            int16_t cy = obs->y;
            /* Entire cactus as one solid block */
            u8g2_DrawBox(&m1_u8g2, cx, cy, obs->w, obs->h);
            break;
        }

        case OBS_CACTUS_LARGE:
        {
            /* Large cactus — solid filled block */
            int16_t cx = obs->x;
            int16_t cy = obs->y;
            /* Entire cactus as one solid block */
            u8g2_DrawBox(&m1_u8g2, cx, cy, obs->w, obs->h);
            break;
        }

        case OBS_BIRD_LOW:
        case OBS_BIRD_HIGH:
        {
            /* Bird: solid filled block */
            int16_t bx = obs->x;
            int16_t by = obs->y;
            /* Entire bird as one solid block */
            u8g2_DrawBox(&m1_u8g2, bx, by, obs->w, obs->h);
            break;
        }

        default:
            break;
    }
}


/*============================================================================*/
/*
 * @brief  Draw the complete game frame
 */
/*============================================================================*/
static void trex_draw(void)
{
    char buf[12];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Ground line */
    u8g2_DrawHLine(&m1_u8g2, 0, TREX_GROUND_LINE_Y, GAME_SCREEN_W);

    /* Ground texture dots */
    for (int x = (trex.score * 2) % 8; x < GAME_SCREEN_W; x += 8)
    {
        u8g2_DrawPixel(&m1_u8g2, x, TREX_GROUND_LINE_Y + 2);
    }
    for (int x = (trex.score * 3 + 4) % 12; x < GAME_SCREEN_W; x += 12)
    {
        u8g2_DrawPixel(&m1_u8g2, x, TREX_GROUND_LINE_Y + 4);
    }

    /* Draw dino */
    trex_draw_dino();

    /* Draw obstacles */
    for (int i = 0; i < MAX_OBSTACLES; i++)
    {
        trex_draw_obstacle(&trex.obs[i]);
    }

    /* Score display */
    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    trex_int_to_str(buf, trex.score);
    uint8_t sw = u8g2_GetStrWidth(&m1_u8g2, buf);
    u8g2_DrawStr(&m1_u8g2, GAME_SCREEN_W - sw - 2, 8, buf);

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Draw the title screen
 */
/*============================================================================*/
static void trex_draw_title(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 16, 22, "T-REX");

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    u8g2_DrawStr(&m1_u8g2, 35, 35, "RUNNER");

    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    u8g2_DrawStr(&m1_u8g2, 22, 48, "Press OK to start");
    u8g2_DrawStr(&m1_u8g2, 12, 60, "UP/OK:Jump DOWN:Duck");

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Draw the game over screen
 */
/*============================================================================*/
static void trex_draw_game_over(void)
{
    char buf[24];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 8, 20, "GAME OVER");

    memcpy(buf, "Score: ", 7);
    trex_int_to_str(buf + 7, trex.score);

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    uint8_t sw = u8g2_GetStrWidth(&m1_u8g2, buf);
    u8g2_DrawStr(&m1_u8g2, (GAME_SCREEN_W - sw) / 2, 38, buf);

    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    u8g2_DrawStr(&m1_u8g2, 10, 56, "OK:Retry  BACK:Exit");

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Main T-Rex runner entry point. Runs own event loop, returns on BACK.
 */
/*============================================================================*/
void game_trex_run(void)
{
    game_button_t btn;

    /* Title screen */
    trex_draw_title();
    while (1)
    {
        btn = game_poll_button(500);
        if (btn == GAME_BTN_BACK) return;
        if (btn == GAME_BTN_OK || btn == GAME_BTN_UP) break;
    }

restart:
    trex_init();

    /* Main game loop */
    while (trex.running)
    {
        uint32_t frame_start = HAL_GetTick();

        /* Poll for input (non-blocking during frame) */
        btn = game_poll_button(TREX_FRAME_MS);

        if (btn == GAME_BTN_BACK)
        {
            return;
        }

        /* Update game state */
        trex_update(btn);

        if (trex.game_over)
        {
            m1_buzzer_set(BUZZER_FREQ_01_KHZ, 200);
            trex_draw_game_over();

            while (1)
            {
                btn = game_poll_button(500);
                if (btn == GAME_BTN_BACK) return;
                if (btn == GAME_BTN_OK) goto restart;
            }
        }

        /* Draw */
        trex_draw();

        /* Frame rate limiting */
        uint32_t elapsed = HAL_GetTick() - frame_start;
        if (elapsed < TREX_FRAME_MS)
        {
            vTaskDelay(pdMS_TO_TICKS(TREX_FRAME_MS - elapsed));
        }
    }
}
