/* See COPYING.txt for license details. */

/*
*
* game_tetris.c
*
* Tetris game for M1
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

#define TET_FIELD_W         10
#define TET_FIELD_H         20
#define TET_CELL_PX         3      /* pixels per cell */

/* Playfield pixel offset: center the 30px wide field in the left portion */
#define TET_FIELD_OX        2      /* x offset in pixels */
#define TET_FIELD_OY        2      /* y offset in pixels (top 2px margin) */

/* Side panel starts after the playfield */
#define TET_PANEL_X         (TET_FIELD_OX + TET_FIELD_W * TET_CELL_PX + 4)

/* Number of tetromino types */
#define TET_NUM_PIECES      7

/* Piece rotation states (4x4 grid stored as 16 bits) */
#define TET_PIECE_SIZE      4

/* Tick rates */
#define TET_BASE_DELAY_MS   500
#define TET_MIN_DELAY_MS    80
#define TET_SPEED_STEP_MS   40

/* Input polling within a tick */
#define TET_INPUT_POLL_MS   50

//************************** S T R U C T U R E S *******************************

/* Each tetromino has 4 rotation states, each stored as a 4x4 bit grid */
typedef struct {
    uint16_t shape[4]; /* 4 rotations, each 16 bits for 4x4 grid */
} tetromino_t;

typedef struct {
    uint8_t  field[TET_FIELD_H][TET_FIELD_W]; /* 0=empty, 1=filled */
    uint8_t  cur_piece;    /* index into tetrominos[] */
    uint8_t  cur_rot;      /* 0-3 */
    int8_t   cur_x;        /* piece position (can be negative) */
    int8_t   cur_y;
    uint8_t  next_piece;
    uint16_t score;
    uint16_t lines;
    uint8_t  level;
    uint16_t delay_ms;
    bool     game_over;
    bool     running;
} tetris_state_t;

/***************************** V A R I A B L E S ******************************/

/*
 * Tetromino shapes encoded as 4x4 bit grids.
 * Bit layout (MSB first): row0[3..0], row1[3..0], row2[3..0], row3[3..0]
 * Row 0 = top, bit3 = left, bit0 = right within each row.
 *
 * Index:  0=I, 1=O, 2=T, 3=S, 4=Z, 5=J, 6=L
 */
static const tetromino_t tetrominos[TET_NUM_PIECES] = {
    /* I piece */
    { .shape = {
        0x0F00, /* ....  ....  ..X.  .X.. */
        0x2222, /* XXXX  ..X.  ....  .X.. */
        0x00F0, /* ....  ..X.  XXXX  .X.. */
        0x4444  /* ....  ..X.  ....  .X.. */
    }},
    /* O piece */
    { .shape = {
        0x6600, /* .XX.  .XX.  .XX.  .XX. */
        0x6600, /* .XX.  .XX.  .XX.  .XX. */
        0x6600, /* ....  ....  ....  .... */
        0x6600  /* ....  ....  ....  .... */
    }},
    /* T piece */
    { .shape = {
        0x4E00, /* .X..  .X..  ....  .X.. */
        0x4C40, /* XXX.  XX..  XXX.  .XX. */
        0x0E40, /* ....  .X..  .X..  .X.. */
        0x4640  /* ....  ....  ....  .... */
    }},
    /* S piece */
    { .shape = {
        0x6C00, /* .XX.  X...  .XX.  X... */
        0x8C40, /* XX..  XX..  XX..  XX.. */
        0x6C00, /* ....  .X..  ....  .X.. */
        0x8C40  /* ....  ....  ....  .... */
    }},
    /* Z piece */
    { .shape = {
        0xC600, /* XX..  .X..  XX..  .X.. */
        0x4C80, /* .XX.  XX..  .XX.  XX.. */
        0xC600, /* ....  X...  ....  X... */
        0x4C80  /* ....  ....  ....  .... */
    }},
    /* J piece */
    { .shape = {
        0x8E00, /* X...  .XX.  ....  .X.. */
        0x6440, /* XXX.  .X..  XXX.  .X.. */
        0x0E20, /* ....  .X..  ..X.  XX.. */
        0x44C0  /* ....  ....  ....  .... */
    }},
    /* L piece */
    { .shape = {
        0x2E00, /* ..X.  .X..  ....  XX.. */
        0x4460, /* XXX.  .X..  XXX.  .X.. */
        0x0E80, /* ....  .XX.  X...  .X.. */
        0xC440  /* ....  ....  ....  .... */
    }}
};

static tetris_state_t tet;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void tet_init(void);
static bool tet_check_collision(uint8_t piece, uint8_t rot, int8_t x, int8_t y);
static void tet_lock_piece(void);
static uint16_t tet_clear_lines(void);
static void tet_spawn_piece(void);
static bool tet_move(int8_t dx, int8_t dy);
static void tet_rotate(void);
static void tet_hard_drop(void);
static void tet_draw(void);
static void tet_draw_title(void);
static void tet_draw_game_over(void);
static bool tet_get_cell(uint16_t shape, int r, int c);
static void tet_draw_piece_at(uint8_t piece, uint8_t rot, int8_t gx, int8_t gy, uint8_t ox, uint8_t oy, uint8_t cell_sz);
static void tet_int_to_str(char *buf, int val);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Get a cell from a 4x4 shape bitfield
 * @param  shape  16-bit shape
 * @param  r      Row (0-3, top to bottom)
 * @param  c      Column (0-3, left to right)
 * @retval true if cell is filled
 */
/*============================================================================*/
static bool tet_get_cell(uint16_t shape, int r, int c)
{
    /* Bit 15 = row0,col0 ... Bit 0 = row3,col3 */
    int bit = 15 - (r * 4 + c);
    return (shape >> bit) & 1;
}


/*============================================================================*/
/*
 * @brief  Integer to string helper (no sprintf)
 */
/*============================================================================*/
static void tet_int_to_str(char *buf, int val)
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
 * @brief  Check if a piece at given position/rotation collides
 */
/*============================================================================*/
static bool tet_check_collision(uint8_t piece, uint8_t rot, int8_t x, int8_t y)
{
    uint16_t shape = tetrominos[piece].shape[rot & 3];

    for (int r = 0; r < TET_PIECE_SIZE; r++)
    {
        for (int c = 0; c < TET_PIECE_SIZE; c++)
        {
            if (!tet_get_cell(shape, r, c))
            {
                continue;
            }

            int fx = x + c;
            int fy = y + r;

            /* Check bounds */
            if (fx < 0 || fx >= TET_FIELD_W || fy >= TET_FIELD_H)
            {
                return true;
            }
            /* Allow above top of field */
            if (fy < 0)
            {
                continue;
            }
            /* Check field */
            if (tet.field[fy][fx])
            {
                return true;
            }
        }
    }
    return false;
}


/*============================================================================*/
/*
 * @brief  Lock the current piece into the field
 */
/*============================================================================*/
static void tet_lock_piece(void)
{
    uint16_t shape = tetrominos[tet.cur_piece].shape[tet.cur_rot & 3];

    for (int r = 0; r < TET_PIECE_SIZE; r++)
    {
        for (int c = 0; c < TET_PIECE_SIZE; c++)
        {
            if (!tet_get_cell(shape, r, c))
            {
                continue;
            }

            int fx = tet.cur_x + c;
            int fy = tet.cur_y + r;

            if (fx >= 0 && fx < TET_FIELD_W && fy >= 0 && fy < TET_FIELD_H)
            {
                tet.field[fy][fx] = 1;
            }
        }
    }
}


/*============================================================================*/
/*
 * @brief  Clear completed lines and return number cleared
 */
/*============================================================================*/
static uint16_t tet_clear_lines(void)
{
    uint16_t cleared = 0;

    for (int r = TET_FIELD_H - 1; r >= 0; r--)
    {
        bool full = true;
        for (int c = 0; c < TET_FIELD_W; c++)
        {
            if (!tet.field[r][c])
            {
                full = false;
                break;
            }
        }

        if (full)
        {
            cleared++;
            /* Shift everything above down */
            for (int rr = r; rr > 0; rr--)
            {
                memcpy(tet.field[rr], tet.field[rr - 1], TET_FIELD_W);
            }
            memset(tet.field[0], 0, TET_FIELD_W);
            r++; /* Re-check this row since it now has the row above */
        }
    }
    return cleared;
}


/*============================================================================*/
/*
 * @brief  Spawn a new piece at the top
 */
/*============================================================================*/
static void tet_spawn_piece(void)
{
    tet.cur_piece = tet.next_piece;
    tet.next_piece = (uint8_t)game_rand_range(0, TET_NUM_PIECES - 1);
    tet.cur_rot = 0;
    tet.cur_x = TET_FIELD_W / 2 - 2;
    tet.cur_y = -1;

    if (tet_check_collision(tet.cur_piece, tet.cur_rot, tet.cur_x, tet.cur_y))
    {
        /* Try one row higher */
        tet.cur_y = -2;
        if (tet_check_collision(tet.cur_piece, tet.cur_rot, tet.cur_x, tet.cur_y))
        {
            tet.game_over = true;
        }
    }
}


/*============================================================================*/
/*
 * @brief  Initialize tetris state
 */
/*============================================================================*/
static void tet_init(void)
{
    memset(&tet, 0, sizeof(tet));
    tet.delay_ms = TET_BASE_DELAY_MS;
    tet.level = 1;
    tet.running = true;

    game_rand_seed();
    tet.next_piece = (uint8_t)game_rand_range(0, TET_NUM_PIECES - 1);
    tet_spawn_piece();
}


/*============================================================================*/
/*
 * @brief  Try to move the current piece by dx, dy
 * @retval true if move succeeded
 */
/*============================================================================*/
static bool tet_move(int8_t dx, int8_t dy)
{
    int8_t nx = tet.cur_x + dx;
    int8_t ny = tet.cur_y + dy;

    if (!tet_check_collision(tet.cur_piece, tet.cur_rot, nx, ny))
    {
        tet.cur_x = nx;
        tet.cur_y = ny;
        return true;
    }
    return false;
}


/*============================================================================*/
/*
 * @brief  Rotate the current piece clockwise with wall kick
 */
/*============================================================================*/
static void tet_rotate(void)
{
    uint8_t new_rot = (tet.cur_rot + 1) & 3;

    /* Try normal rotation */
    if (!tet_check_collision(tet.cur_piece, new_rot, tet.cur_x, tet.cur_y))
    {
        tet.cur_rot = new_rot;
        return;
    }

    /* Wall kick: try shifting left/right */
    for (int8_t kick = 1; kick <= 2; kick++)
    {
        if (!tet_check_collision(tet.cur_piece, new_rot, tet.cur_x - kick, tet.cur_y))
        {
            tet.cur_x -= kick;
            tet.cur_rot = new_rot;
            return;
        }
        if (!tet_check_collision(tet.cur_piece, new_rot, tet.cur_x + kick, tet.cur_y))
        {
            tet.cur_x += kick;
            tet.cur_rot = new_rot;
            return;
        }
    }
}


/*============================================================================*/
/*
 * @brief  Hard drop the current piece straight down
 */
/*============================================================================*/
static void tet_hard_drop(void)
{
    while (tet_move(0, 1))
    {
        tet.score += 1;
    }
}


/*============================================================================*/
/*
 * @brief  Draw a tetromino piece at given grid and pixel offsets
 */
/*============================================================================*/
static void tet_draw_piece_at(uint8_t piece, uint8_t rot, int8_t gx, int8_t gy,
                              uint8_t ox, uint8_t oy, uint8_t cell_sz)
{
    uint16_t shape = tetrominos[piece].shape[rot & 3];

    for (int r = 0; r < TET_PIECE_SIZE; r++)
    {
        for (int c = 0; c < TET_PIECE_SIZE; c++)
        {
            if (tet_get_cell(shape, r, c))
            {
                int px = ox + (gx + c) * cell_sz;
                int py = oy + (gy + r) * cell_sz;

                if (px >= 0 && py >= 0 && px + cell_sz <= GAME_SCREEN_W && py + cell_sz <= GAME_SCREEN_H)
                {
                    u8g2_DrawBox(&m1_u8g2, px, py, cell_sz, cell_sz);
                }
            }
        }
    }
}


/*============================================================================*/
/*
 * @brief  Draw the tetris game screen
 */
/*============================================================================*/
static void tet_draw(void)
{
    char buf[12];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Draw field border */
    u8g2_DrawFrame(&m1_u8g2,
                   TET_FIELD_OX - 1, TET_FIELD_OY - 1,
                   TET_FIELD_W * TET_CELL_PX + 2,
                   TET_FIELD_H * TET_CELL_PX + 2);

    /* Draw locked cells */
    for (int r = 0; r < TET_FIELD_H; r++)
    {
        for (int c = 0; c < TET_FIELD_W; c++)
        {
            if (tet.field[r][c])
            {
                u8g2_DrawBox(&m1_u8g2,
                             TET_FIELD_OX + c * TET_CELL_PX,
                             TET_FIELD_OY + r * TET_CELL_PX,
                             TET_CELL_PX, TET_CELL_PX);
            }
        }
    }

    /* Draw current piece */
    tet_draw_piece_at(tet.cur_piece, tet.cur_rot, tet.cur_x, tet.cur_y,
                      TET_FIELD_OX, TET_FIELD_OY, TET_CELL_PX);

    /* Draw ghost piece (shadow) */
    {
        int8_t ghost_y = tet.cur_y;
        while (!tet_check_collision(tet.cur_piece, tet.cur_rot, tet.cur_x, ghost_y + 1))
        {
            ghost_y++;
        }
        if (ghost_y != tet.cur_y)
        {
            uint16_t shape = tetrominos[tet.cur_piece].shape[tet.cur_rot & 3];
            for (int r = 0; r < TET_PIECE_SIZE; r++)
            {
                for (int c = 0; c < TET_PIECE_SIZE; c++)
                {
                    if (tet_get_cell(shape, r, c))
                    {
                        int px = TET_FIELD_OX + (tet.cur_x + c) * TET_CELL_PX;
                        int py = TET_FIELD_OY + (ghost_y + r) * TET_CELL_PX;
                        if (px >= 0 && py >= 0)
                        {
                            u8g2_DrawFrame(&m1_u8g2, px, py, TET_CELL_PX, TET_CELL_PX);
                        }
                    }
                }
            }
        }
    }

    /* Side panel: next piece, score, level, lines */
    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);

    u8g2_DrawStr(&m1_u8g2, TET_PANEL_X, 8, "NEXT");

    /* Draw next piece preview */
    u8g2_DrawFrame(&m1_u8g2, TET_PANEL_X, 10, 14, 14);
    {
        uint16_t shape = tetrominos[tet.next_piece].shape[0];
        for (int r = 0; r < TET_PIECE_SIZE; r++)
        {
            for (int c = 0; c < TET_PIECE_SIZE; c++)
            {
                if (tet_get_cell(shape, r, c))
                {
                    u8g2_DrawBox(&m1_u8g2,
                                 TET_PANEL_X + 1 + c * 3,
                                 11 + r * 3,
                                 3, 3);
                }
            }
        }
    }

    /* Score */
    u8g2_DrawStr(&m1_u8g2, TET_PANEL_X, 34, "SCORE");
    tet_int_to_str(buf, tet.score);
    u8g2_DrawStr(&m1_u8g2, TET_PANEL_X, 42, buf);

    /* Level */
    u8g2_DrawStr(&m1_u8g2, TET_PANEL_X, 52, "LVL");
    tet_int_to_str(buf, tet.level);
    u8g2_DrawStr(&m1_u8g2, TET_PANEL_X + 18, 52, buf);

    /* Lines */
    u8g2_DrawStr(&m1_u8g2, TET_PANEL_X, 62, "LN");
    tet_int_to_str(buf, tet.lines);
    u8g2_DrawStr(&m1_u8g2, TET_PANEL_X + 14, 62, buf);

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Draw the title screen
 */
/*============================================================================*/
static void tet_draw_title(void)
{
    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 18, 25, "TETRIS");

    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    u8g2_DrawStr(&m1_u8g2, 22, 42, "Press OK to start");
    u8g2_DrawStr(&m1_u8g2, 4, 56, "L/R:Move OK:Rot UP:Drop");

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Draw the game over screen
 */
/*============================================================================*/
static void tet_draw_game_over(void)
{
    char buf[24];

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
    u8g2_DrawStr(&m1_u8g2, 8, 20, "GAME OVER");

    memcpy(buf, "Score: ", 7);
    tet_int_to_str(buf + 7, tet.score);

    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    uint8_t sw = u8g2_GetStrWidth(&m1_u8g2, buf);
    u8g2_DrawStr(&m1_u8g2, (GAME_SCREEN_W - sw) / 2, 38, buf);

    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    u8g2_DrawStr(&m1_u8g2, 10, 56, "OK:Retry  BACK:Exit");

    m1_u8g2_nextpage();
}


/*============================================================================*/
/*
 * @brief  Main tetris game entry point. Runs own event loop, returns on BACK.
 */
/*============================================================================*/
void game_tetris_run(void)
{
    game_button_t btn;

    /* Title screen */
    tet_draw_title();
    while (1)
    {
        btn = game_poll_button(500);
        if (btn == GAME_BTN_BACK) return;
        if (btn == GAME_BTN_OK) break;
    }

restart:
    tet_init();
    tet_draw();

    while (tet.running)
    {
        /* Process input within the gravity tick period */
        uint32_t tick_start = HAL_GetTick();
        uint32_t elapsed = 0;
        bool dropped = false;

        while (elapsed < tet.delay_ms)
        {
            uint32_t remaining = tet.delay_ms - elapsed;
            uint32_t poll_time = (remaining < TET_INPUT_POLL_MS) ? remaining : TET_INPUT_POLL_MS;

            btn = game_poll_button(poll_time);
            elapsed = HAL_GetTick() - tick_start;

            if (btn == GAME_BTN_BACK)
            {
                return;
            }

            switch (btn)
            {
                case GAME_BTN_LEFT:
                    tet_move(-1, 0);
                    tet_draw();
                    break;

                case GAME_BTN_RIGHT:
                    tet_move(1, 0);
                    tet_draw();
                    break;

                case GAME_BTN_DOWN:
                    if (tet_move(0, 1))
                    {
                        tet.score += 1;
                    }
                    tet_draw();
                    break;

                case GAME_BTN_OK:
                    tet_rotate();
                    tet_draw();
                    break;

                case GAME_BTN_UP:
                    tet_hard_drop();
                    dropped = true;
                    break;

                default:
                    break;
            }

            if (dropped) break;
        }

        /* Gravity: move piece down */
        if (!tet_move(0, 1))
        {
            /* Lock piece */
            tet_lock_piece();

            /* Clear lines */
            uint16_t cleared = tet_clear_lines();
            if (cleared > 0)
            {
                /* Scoring: 100, 300, 500, 800 for 1-4 lines */
                static const uint16_t line_scores[5] = {0, 100, 300, 500, 800};
                uint16_t idx = (cleared > 4) ? 4 : cleared;
                tet.score += line_scores[idx] * tet.level;
                tet.lines += cleared;

                /* Level up every 10 lines */
                tet.level = (uint8_t)(tet.lines / 10) + 1;
                if (tet.level > 15) tet.level = 15;

                /* Speed up */
                tet.delay_ms = TET_BASE_DELAY_MS - (tet.level - 1) * TET_SPEED_STEP_MS;
                if (tet.delay_ms < TET_MIN_DELAY_MS) tet.delay_ms = TET_MIN_DELAY_MS;

                m1_buzzer_set(BUZZER_FREQ_04_KHZ, 50);
            }

            /* Spawn next piece */
            tet_spawn_piece();

            if (tet.game_over)
            {
                m1_buzzer_set(BUZZER_FREQ_01_KHZ, 200);
                tet_draw_game_over();

                while (1)
                {
                    btn = game_poll_button(500);
                    if (btn == GAME_BTN_BACK) return;
                    if (btn == GAME_BTN_OK) goto restart;
                }
            }
        }

        tet_draw();
    }
}
