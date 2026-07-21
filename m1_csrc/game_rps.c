/* See COPYING.txt for license details. */

/*
*
* game_rps.c
*
* Rock Paper Scissors — play against the M1
*
* LEFT/RIGHT to choose, OK to play. Score tracking. BACK exits.
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "m1_games.h"

/*************************** D E F I N E S ************************************/

/* Game state */
typedef enum { PICK_ROCK = 0, PICK_PAPER, PICK_SCISSORS } choice_t;
typedef enum { STATE_CHOOSE, STATE_REVEAL, STATE_RESULT } state_t;

/***************************** V A R I A B L E S ******************************/

static int player_score = 0;
static int cpu_score = 0;
static int draws = 0;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void draw_rock(int x, int y, int selected);
static void draw_paper(int x, int y, int selected);
static void draw_scissors(int x, int y, int selected);
static void draw_choice(int x, int y, choice_t c, int selected);
static int judge(choice_t player, choice_t cpu);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Draw a rock (circle with R)
 */
/*============================================================================*/
static void draw_rock(int x, int y, int selected)
{
    if (selected)
        u8g2_DrawDisc(&m1_u8g2, x + 12, y + 12, 12, U8G2_DRAW_ALL);
    else
        u8g2_DrawCircle(&m1_u8g2, x + 12, y + 12, 12, U8G2_DRAW_ALL);

    u8g2_SetDrawColor(&m1_u8g2, selected ? 0 : 1);
    u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(&m1_u8g2, x + 8, y + 16, "R");
    u8g2_SetDrawColor(&m1_u8g2, 1);
}


/*============================================================================*/
/*
 * @brief  Draw paper (rectangle with P)
 */
/*============================================================================*/
static void draw_paper(int x, int y, int selected)
{
    if (selected)
        u8g2_DrawBox(&m1_u8g2, x, y, 24, 24);
    else
        u8g2_DrawFrame(&m1_u8g2, x, y, 24, 24);

    u8g2_SetDrawColor(&m1_u8g2, selected ? 0 : 1);
    u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(&m1_u8g2, x + 8, y + 16, "P");
    u8g2_SetDrawColor(&m1_u8g2, 1);
}


/*============================================================================*/
/*
 * @brief  Draw scissors (framed box with X and S)
 */
/*============================================================================*/
static void draw_scissors(int x, int y, int selected)
{
    if (selected)
    {
        u8g2_DrawBox(&m1_u8g2, x, y, 24, 24);
        u8g2_SetDrawColor(&m1_u8g2, 0);
    }
    else
    {
        u8g2_DrawFrame(&m1_u8g2, x, y, 24, 24);
        /* Draw X lines */
        u8g2_DrawLine(&m1_u8g2, x + 4, y + 4, x + 20, y + 20);
        u8g2_DrawLine(&m1_u8g2, x + 20, y + 4, x + 4, y + 20);
    }

    u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(&m1_u8g2, x + 8, y + 16, "S");
    u8g2_SetDrawColor(&m1_u8g2, 1);
}


/*============================================================================*/
/*
 * @brief  Draw a choice symbol at the given position
 */
/*============================================================================*/
static void draw_choice(int x, int y, choice_t c, int selected)
{
    switch (c)
    {
        case PICK_ROCK:     draw_rock(x, y, selected); break;
        case PICK_PAPER:    draw_paper(x, y, selected); break;
        case PICK_SCISSORS: draw_scissors(x, y, selected); break;
    }
}


/*============================================================================*/
/*
 * @brief  Judge a round
 * @retval 0=draw, 1=player wins, 2=cpu wins
 */
/*============================================================================*/
static int judge(choice_t player, choice_t cpu)
{
    if (player == cpu) return 0;
    if ((player == PICK_ROCK && cpu == PICK_SCISSORS) ||
        (player == PICK_PAPER && cpu == PICK_ROCK) ||
        (player == PICK_SCISSORS && cpu == PICK_PAPER))
        return 1;
    return 2;
}


/*============================================================================*/
/*
 * @brief  Main entry point for Rock Paper Scissors.
 *         Runs its own event loop; returns when user presses BACK.
 */
/*============================================================================*/
void game_rps_run(void)
{
    game_rand_seed();

    state_t  state         = STATE_CHOOSE;
    choice_t selection     = PICK_ROCK;
    choice_t player_choice = PICK_ROCK;
    choice_t cpu_choice    = PICK_ROCK;
    int      outcome       = 0;
    int      reveal_timer  = 0;

    player_score = 0;
    cpu_score    = 0;
    draws        = 0;

    while (1)
    {
        game_button_t btn = game_poll_button(state == STATE_REVEAL ? 80 : 200);

        if (btn == GAME_BTN_BACK)
            return;

        /* State machine */
        switch (state)
        {
        case STATE_CHOOSE:
            if (btn == GAME_BTN_LEFT)
            {
                selection = (selection == PICK_ROCK) ? PICK_SCISSORS : (choice_t)(selection - 1);
                m1_buzzer_notification();
            }
            else if (btn == GAME_BTN_RIGHT)
            {
                selection = (selection == PICK_SCISSORS) ? PICK_ROCK : (choice_t)(selection + 1);
                m1_buzzer_notification();
            }
            else if (btn == GAME_BTN_OK)
            {
                player_choice = selection;
                cpu_choice = (choice_t)game_rand_range(0, 2);
                state = STATE_REVEAL;
                reveal_timer = 0;
                m1_buzzer_notification();
            }
            break;

        case STATE_REVEAL:
            reveal_timer++;
            if (reveal_timer >= 10)
            {
                outcome = judge(player_choice, cpu_choice);
                if (outcome == 1) player_score++;
                else if (outcome == 2) cpu_score++;
                else draws++;

                if (outcome == 1)
                    m1_buzzer_notification();
                else if (outcome == 2)
                    m1_buzzer_notification2();
                else
                    m1_buzzer_notification();

                state = STATE_RESULT;
            }
            break;

        case STATE_RESULT:
            if (btn == GAME_BTN_OK)
            {
                state = STATE_CHOOSE;
            }
            break;
        }

        /* Draw */
        m1_u8g2_firstpage();
        do {
            u8g2_SetDrawColor(&m1_u8g2, 1);
            u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);

            /* Score bar */
            char score[32];
            snprintf(score, sizeof(score), "You:%d  CPU:%d  Draw:%d",
                     player_score, cpu_score, draws);
            u8g2_DrawStr(&m1_u8g2, 2, 8, score);
            u8g2_DrawHLine(&m1_u8g2, 0, 10, 128);

            switch (state)
            {
            case STATE_CHOOSE:
            {
                /* Draw three choices side by side */
                int base_x = 8;
                int base_y = 16;
                int spacing = 40;

                for (int i = 0; i < 3; i++)
                {
                    draw_choice(base_x + i * spacing, base_y,
                                (choice_t)i, (int)selection == i);
                }

                /* Labels */
                u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
                u8g2_DrawStr(&m1_u8g2, 10, 46, "Rock");
                u8g2_DrawStr(&m1_u8g2, 47, 46, "Paper");
                u8g2_DrawStr(&m1_u8g2, 82, 46, "Sciss");

                /* Arrow indicator */
                int arrow_x = base_x + (int)selection * spacing + 10;
                u8g2_DrawStr(&m1_u8g2, arrow_x, 54, "^");

                u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
                u8g2_DrawStr(&m1_u8g2, 10, 63, "<LEFT/RIGHT> OK:Play");
                break;
            }

            case STATE_REVEAL:
            {
                /* Countdown animation — show "3..2..1.." */
                u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
                char countdown[8];
                int n = 3 - (reveal_timer / 3);
                if (n < 1) n = 1;
                snprintf(countdown, sizeof(countdown), "%d...", n);
                u8g2_DrawStr(&m1_u8g2, 50, 40, countdown);

                /* Flash random symbols */
                choice_t flash = (choice_t)(reveal_timer % 3);
                draw_choice(10, 20, flash, 0);
                draw_choice(94, 20, (choice_t)((reveal_timer + 1) % 3), 0);
                break;
            }

            case STATE_RESULT:
            {
                /* Show both choices */
                u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
                u8g2_DrawStr(&m1_u8g2, 8, 20, "You:");
                u8g2_DrawStr(&m1_u8g2, 80, 20, "CPU:");

                draw_choice(10, 22, player_choice, 0);
                draw_choice(90, 22, cpu_choice, 0);

                /* VS */
                u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
                u8g2_DrawStr(&m1_u8g2, 55, 38, "VS");

                /* Result text */
                u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
                if (outcome == 1)
                    u8g2_DrawStr(&m1_u8g2, 34, 58, "You WIN!");
                else if (outcome == 2)
                    u8g2_DrawStr(&m1_u8g2, 30, 58, "You LOSE!");
                else
                    u8g2_DrawStr(&m1_u8g2, 40, 58, "DRAW!");

                u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
                u8g2_DrawStr(&m1_u8g2, 34, 64, "OK: Again");
                break;
            }
            }

        } while (m1_u8g2_nextpage());
    }
}
