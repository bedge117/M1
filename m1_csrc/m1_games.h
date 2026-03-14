/* See COPYING.txt for license details. */

/*
*
* m1_games.h
*
* Built-in games for M1
*
* M1 Project
*
*/

#ifndef M1_GAMES_H_
#define M1_GAMES_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_system.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_tasks.h"
#include "m1_buzzer.h"

/* Display dimensions */
#define GAME_SCREEN_W    128
#define GAME_SCREEN_H    64

/* Simplified button reading for games */
typedef enum {
    GAME_BTN_NONE = 0,
    GAME_BTN_UP,
    GAME_BTN_DOWN,
    GAME_BTN_LEFT,
    GAME_BTN_RIGHT,
    GAME_BTN_OK,
    GAME_BTN_BACK
} game_button_t;

/* Poll for a button press with timeout in ms. Returns GAME_BTN_NONE on timeout. */
game_button_t game_poll_button(uint32_t timeout_ms);

/* Seed the game RNG from hardware tick */
void game_rand_seed(void);

/* Get random number in range [min, max] inclusive */
int game_rand_range(int min, int max);

/* Game entry points */
void game_snake_run(void);
void game_tetris_run(void);
void game_trex_run(void);
void game_pong_run(void);
void game_dice_run(void);

/* SD card apps browser (Phase 2) */
void game_apps_browser_run(void);

#endif /* M1_GAMES_H_ */
