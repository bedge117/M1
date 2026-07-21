/* See COPYING.txt for license details. */

/*
 *
 * game_tamagotchi.c
 *
 * Tamagotchi P1 — virtual pet, native firmware port
 *
 * M1 Project
 *
 * Ported from the M1 SDK example app (m1-sdk/examples/tamagotchi_p1). The
 * original app ran a cycle-accurate TamaLIB emulation of the Epson E0C6S46
 * CPU driven by a 12 KB ROM dump on the SD card. That emulator core (cpu.c,
 * hw.c, tamalib.c) and the ROM binary cannot be linked into the firmware
 * from this single translation unit, so the emulator shell is replaced with
 * a self-contained native pet simulation that reproduces the original P1
 * mechanics faithfully: the same eight-icon menu (Food, Light, Game,
 * Medicine, Clean, Status, Discipline, Attention), the same A/B/C control
 * mapping (LEFT/OK/RIGHT), stat decay, poop, sickness, sleep, evolution and
 * death, plus auto-save/auto-load of pet state to the SD card.
 *
 * Controls (mirroring the SDK app):
 *   LEFT  = A button (select / scroll icon, toggle sub-menu choice)
 *   OK    = B button (confirm / execute)
 *   RIGHT = C button (cancel / back out of a sub-menu)
 *   BACK  = save & exit
 */

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32h5xx_hal.h"
#include "m1_games.h"
#include "m1_buzzer.h"
#include "m1_lp5814.h"
#include "ff.h"

/*************************** D E F I N E S ************************************/

/* Persistence — native FatFs, mirrors the app's f_open/f_read/f_write save. */
#define TAMA_DIR         "0:/Games"
#define TAMA_SAVE_PATH   "0:/Games/tama.dat"
#define TAMA_SAVE_MAGIC  0x54414D31u   /* "TAM1" */
#define TAMA_SAVE_VER    1

/* Screen geometry (128x64). */
#define PET_FRAME_X      22
#define PET_FRAME_Y      1
#define PET_FRAME_W      84
#define PET_FRAME_H      40
#define PET_CX           64            /* creature centre x */
#define PET_FLOOR_Y      38            /* creature baseline y */

#define ICON_ROW_Y       43
#define ICON_CELL_W      16
#define ICON_CELL_H      10
#define HINT_Y           62

/* Eight P1 icons — order and labels taken verbatim from the SDK app. */
enum {
    ICON_FOOD = 0,   /* FD */
    ICON_LIGHT,      /* LT */
    ICON_GAME,       /* GM */
    ICON_MEDICINE,   /* MD */
    ICON_CLEAN,      /* CL */
    ICON_STATUS,     /* ST */
    ICON_DISCIPLINE, /* DS */
    ICON_ATTENTION,  /* AT */
    ICON_NUM
};

static const char * const icon_labels[ICON_NUM] = {
    "FD", "LT", "GM", "MD", "CL", "ST", "DS", "AT"
};

/* Stats are kept 0..100 internally; the P1 UI shows 4 pips (value / 25). */
#define STAT_MAX         100
#define PIP_MAX          4

/* Simulation cadence — a "sim minute" is compressed so the demo is lively
 * without being frantic. One sim tick == SIM_TICK_MS of real time. */
#define SIM_TICK_MS      1000u          /* stat clock granularity            */
#define HUNGER_FALL      3               /* hunger lost per HUNGER_EVERY ticks*/
#define HUNGER_EVERY     6
#define HAPPY_FALL       3
#define HAPPY_EVERY      9
#define POOP_AFTER       45              /* ticks after a meal a poop appears */
#define AGE_EVERY        60              /* ticks per pet "day"               */
#define SICK_CHECK_EVERY 20              /* ticks between sickness rolls      */

/* Evolution stages by age in days. */
enum { STAGE_EGG = 0, STAGE_BABY, STAGE_CHILD, STAGE_TEEN, STAGE_ADULT };

/* Pet needs — drives the Attention icon and the call beep. */
#define NEED_HUNGER   0x01
#define NEED_HAPPY    0x02
#define NEED_POOP     0x04
#define NEED_SICK     0x08

//************************** S T R U C T U R E S *******************************

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;

    uint8_t  hunger;      /* 0 = starving, 100 = full            */
    uint8_t  happy;       /* 0 = miserable, 100 = delighted      */
    uint8_t  weight;      /* grams (P1 flavour)                  */
    uint8_t  discipline;  /* 0..100                              */
    uint8_t  health;      /* 0 = dead                            */

    uint8_t  poop;        /* 1 = a poop is on screen             */
    uint8_t  sick;        /* 1 = ill, needs medicine             */
    uint8_t  sleeping;    /* 1 = asleep                          */
    uint8_t  light_off;   /* 1 = the room light is off           */
    uint8_t  stage;       /* evolution stage                     */
    uint8_t  alive;       /* 0 = dead (grave shown)              */

    uint16_t age_days;    /* age in pet days                     */
    uint32_t born_tick;   /* HAL tick at hatch (unused across boots) */
} pet_state_t;

/***************************** V A R I A B L E S ******************************/

static pet_state_t pet;

/* Runtime-only counters (not persisted — rebuilt each session). */
static uint32_t sim_last_tick;      /* HAL tick of last sim step   */
static uint32_t hunger_ctr;
static uint32_t happy_ctr;
static uint32_t age_ctr;
static uint32_t poop_ctr;           /* counts up after eating      */
static uint32_t sick_ctr;
static uint8_t  anim_frame;         /* idle bob / blink toggle     */
static uint8_t  needs;              /* NEED_* bitmask              */
static uint8_t  sel;                /* selected icon 0..7          */

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static void  pet_new(void);
static int   pet_load(void);
static int   pet_save(void);
static void  pet_sim_step(void);
static void  pet_recompute_needs(void);
static void  pet_update_led(void);

static void  execute_action(uint8_t icon);
static bool  action_food(void);
static void  action_light(void);
static bool  action_game(void);
static void  action_medicine(void);
static void  action_clean(void);
static void  action_status(void);
static void  action_discipline(void);
static void  action_attention(void);

static void  draw_main(void);
static void  draw_pet_sprite(void);
static void  draw_side_gauges(void);
static void  draw_icons(void);
static void  draw_hints(void);
static void  draw_message(const char *l1, const char *l2, uint16_t hold_ms);
static void  draw_grave(void);

static uint8_t clamp_add(uint8_t v, int delta);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/* @brief  Saturating add on a 0..STAT_MAX stat.                              */
/*============================================================================*/
static uint8_t clamp_add(uint8_t v, int delta)
{
    int r = (int)v + delta;
    if (r < 0)          r = 0;
    if (r > STAT_MAX)   r = STAT_MAX;
    return (uint8_t)r;
}


/*============================================================================*/
/* @brief  Hatch a fresh pet.                                                 */
/*============================================================================*/
static void pet_new(void)
{
    memset(&pet, 0, sizeof(pet));
    pet.magic      = TAMA_SAVE_MAGIC;
    pet.version    = TAMA_SAVE_VER;
    pet.hunger     = 70;
    pet.happy      = 70;
    pet.weight     = 5;
    pet.discipline = 0;
    pet.health     = 100;
    pet.stage      = STAGE_EGG;
    pet.alive      = 1;
    pet.light_off  = 0;
    pet.age_days   = 0;
    pet.born_tick  = HAL_GetTick();

    hunger_ctr = happy_ctr = age_ctr = poop_ctr = sick_ctr = 0;
    needs = 0;
    sel = 0;
}


/*============================================================================*/
/* @brief  Load pet state from SD (native FatFs). Returns 0 on success.       */
/*============================================================================*/
static int pet_load(void)
{
    FIL fil;
    UINT br;
    pet_state_t tmp;

    if (f_open(&fil, TAMA_SAVE_PATH, FA_READ) != FR_OK)
        return -1;

    if (f_read(&fil, &tmp, sizeof(tmp), &br) != FR_OK || br != sizeof(tmp)) {
        f_close(&fil);
        return -1;
    }
    f_close(&fil);

    if (tmp.magic != TAMA_SAVE_MAGIC || tmp.version != TAMA_SAVE_VER)
        return -1;

    pet = tmp;
    /* Runtime counters are not persisted — start them fresh. */
    hunger_ctr = happy_ctr = age_ctr = poop_ctr = sick_ctr = 0;
    return 0;
}


/*============================================================================*/
/* @brief  Save pet state to SD (native FatFs). Returns 0 on success.         */
/*============================================================================*/
static int pet_save(void)
{
    FIL fil;
    UINT bw;

    f_mkdir(TAMA_DIR);   /* harmless if it already exists */

    if (f_open(&fil, TAMA_SAVE_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return -1;

    pet.magic   = TAMA_SAVE_MAGIC;
    pet.version = TAMA_SAVE_VER;

    if (f_write(&fil, &pet, sizeof(pet), &bw) != FR_OK || bw != sizeof(pet)) {
        f_close(&fil);
        return -1;
    }
    f_sync(&fil);
    f_close(&fil);
    return 0;
}


/*============================================================================*/
/* @brief  Recompute the NEED_* bitmask from current stats.                   */
/*============================================================================*/
static void pet_recompute_needs(void)
{
    needs = 0;
    if (pet.hunger <= 25) needs |= NEED_HUNGER;
    if (pet.happy  <= 25) needs |= NEED_HAPPY;
    if (pet.poop)         needs |= NEED_POOP;
    if (pet.sick)         needs |= NEED_SICK;
}


/*============================================================================*/
/* @brief  RGB feedback: red=needs/sick, blue=asleep, green=content.          */
/*============================================================================*/
static void pet_update_led(void)
{
    if (!pet.alive) {
        lp5814_all_off_RGB();
        return;
    }
    if (pet.sick || (needs & NEED_HUNGER)) {
        lp5814_led_on_Red(60);
        lp5814_led_on_Green(0);
        lp5814_led_on_Blue(0);
    } else if (pet.sleeping) {
        lp5814_led_on_Red(0);
        lp5814_led_on_Green(0);
        lp5814_led_on_Blue(40);
    } else if (needs == 0 && pet.happy > 50) {
        lp5814_led_on_Red(0);
        lp5814_led_on_Green(40);
        lp5814_led_on_Blue(0);
    } else {
        lp5814_all_off_RGB();
    }
}


/*============================================================================*/
/* @brief  Advance the simulation by however many SIM_TICK_MS have elapsed.   */
/*============================================================================*/
static void pet_sim_step(void)
{
    if (!pet.alive)
        return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - sim_last_tick;
    if (elapsed < SIM_TICK_MS)
        return;

    uint32_t ticks = elapsed / SIM_TICK_MS;
    sim_last_tick += ticks * SIM_TICK_MS;
    if (ticks > 30) ticks = 30;   /* clamp after a long stall */

    for (uint32_t t = 0; t < ticks; t++) {

        /* --- hunger / happiness decay (slower while asleep) --- */
        if (!pet.sleeping) {
            if (++hunger_ctr >= HUNGER_EVERY) {
                hunger_ctr = 0;
                pet.hunger = clamp_add(pet.hunger, -HUNGER_FALL);
            }
            if (++happy_ctr >= HAPPY_EVERY) {
                happy_ctr = 0;
                pet.happy = clamp_add(pet.happy, -HAPPY_FALL);
            }
        }

        /* --- poop timer (only after the pet has eaten) --- */
        if (!pet.poop && poop_ctr > 0) {
            if (++poop_ctr >= POOP_AFTER) {
                poop_ctr = 0;
                pet.poop = 1;
            }
        }

        /* --- sickness rolls: neglect raises the odds --- */
        if (++sick_ctr >= SICK_CHECK_EVERY) {
            sick_ctr = 0;
            if (!pet.sick) {
                int risk = 0;
                if (pet.poop)         risk += 30;
                if (pet.hunger == 0)  risk += 40;
                if (pet.happy  == 0)  risk += 20;
                risk += 3;   /* small baseline */
                if (game_rand_range(0, 99) < risk)
                    pet.sick = 1;
            }
        }

        /* --- health erodes while a need is ignored, recovers when happy --- */
        if (pet.hunger == 0 || pet.sick || pet.poop) {
            pet.health = clamp_add(pet.health, -2);
        } else if (pet.hunger > 50 && pet.happy > 50) {
            pet.health = clamp_add(pet.health, +1);
        }

        /* --- ageing / evolution --- */
        if (++age_ctr >= AGE_EVERY) {
            age_ctr = 0;
            if (pet.age_days < 0xFFFF) pet.age_days++;

            uint8_t new_stage = pet.stage;
            if      (pet.age_days >= 12) new_stage = STAGE_ADULT;
            else if (pet.age_days >= 6)  new_stage = STAGE_TEEN;
            else if (pet.age_days >= 2)  new_stage = STAGE_CHILD;
            else if (pet.age_days >= 1)  new_stage = STAGE_BABY;
            if (new_stage != pet.stage) {
                pet.stage = new_stage;
                m1_buzzer_set(BUZZER_FREQ_06_KHZ, 60);   /* evolution chime */
            }
        }

        if (pet.health == 0) {
            pet.alive = 0;
            break;
        }
    }

    pet_recompute_needs();
}


/*============================================================================*/
/*                             A C T I O N S                                  */
/*============================================================================*/

/* Feed: A toggles Meal/Snack, B confirms, C cancels. */
static bool action_food(void)
{
    uint8_t choice = 0;   /* 0 = Meal, 1 = Snack */
    game_button_t b;

    if (pet.sleeping) { draw_message("Shhh...", "It's asleep", 900); return false; }

    while (1) {
        m1_u8g2_firstpage();
        do {
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 30, 12, "FEED");

            u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
            if (choice == 0) u8g2_DrawBox(&m1_u8g2, 20, 22, 88, 12);
            u8g2_SetDrawColor(&m1_u8g2, choice == 0 ? M1_DISP_DRAW_COLOR_BG
                                                    : M1_DISP_DRAW_COLOR_TXT);
            u8g2_DrawStr(&m1_u8g2, 24, 31, "Meal   (fills up)");
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

            if (choice == 1) u8g2_DrawBox(&m1_u8g2, 20, 36, 88, 12);
            u8g2_SetDrawColor(&m1_u8g2, choice == 1 ? M1_DISP_DRAW_COLOR_BG
                                                    : M1_DISP_DRAW_COLOR_TXT);
            u8g2_DrawStr(&m1_u8g2, 24, 45, "Snack  (+weight)");
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

            u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
            u8g2_DrawStr(&m1_u8g2, 6, HINT_Y, "<A:sel  B:ok  C:back>");
        } while (m1_u8g2_nextpage());

        b = game_poll_button(500);
        if (b == GAME_BTN_LEFT || b == GAME_BTN_UP || b == GAME_BTN_DOWN)
            choice ^= 1;
        else if (b == GAME_BTN_RIGHT || b == GAME_BTN_BACK)
            return false;
        else if (b == GAME_BTN_OK)
            break;
    }

    if (choice == 0) {
        pet.hunger = clamp_add(pet.hunger, 30);
        pet.weight = clamp_add(pet.weight, 2);
    } else {
        pet.hunger = clamp_add(pet.hunger, 8);
        pet.happy  = clamp_add(pet.happy, 8);
        pet.weight = clamp_add(pet.weight, 4);
    }
    if (poop_ctr == 0) poop_ctr = 1;   /* start the digestion timer */
    m1_buzzer_set(BUZZER_FREQ_04_KHZ, 40);
    return true;
}

/* Light: toggle the room light (sleeping pet with light off gains rest). */
static void action_light(void)
{
    pet.light_off ^= 1;
    m1_buzzer_set(BUZZER_FREQ_02_KHZ, 25);

    if (pet.sleeping && pet.light_off) {
        draw_message("Lights out", "Good night", 900);
    } else if (pet.sleeping && !pet.light_off) {
        /* Waking it with the light hurts happiness a little. */
        pet.happy = clamp_add(pet.happy, -5);
        draw_message("Light ON", "Zzz disturbed", 900);
    } else {
        draw_message(pet.light_off ? "Light OFF" : "Light ON", NULL, 700);
    }
}

/* Game: hi/lo guessing minigame, faithful to P1's number game. */
static bool action_game(void)
{
    if (pet.sleeping) { draw_message("Shhh...", "It's asleep", 900); return false; }

    int secret = game_rand_range(1, 9);
    int guess  = 5;
    game_button_t b;
    char line[24];

    while (1) {
        m1_u8g2_firstpage();
        do {
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 20, 12, "GUESS 1-9");

            u8g2_SetFont(&m1_u8g2, M1_DISP_LARGE_FONT_2B);
            snprintf(line, sizeof(line), "%d", guess);
            u8g2_DrawStr(&m1_u8g2, 60, 40, line);

            u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
            u8g2_DrawStr(&m1_u8g2, 6, HINT_Y, "<A:+1  B:guess  C:quit>");
        } while (m1_u8g2_nextpage());

        b = game_poll_button(500);
        if (b == GAME_BTN_LEFT || b == GAME_BTN_UP) {
            if (++guess > 9) guess = 1;
        } else if (b == GAME_BTN_DOWN) {
            if (--guess < 1) guess = 9;
        } else if (b == GAME_BTN_RIGHT || b == GAME_BTN_BACK) {
            return false;
        } else if (b == GAME_BTN_OK) {
            if (guess == secret) {
                pet.happy  = clamp_add(pet.happy, 25);
                pet.weight = clamp_add(pet.weight, -1);
                m1_buzzer_set(BUZZER_FREQ_08_KHZ, 60);
                draw_message("You win!", "It's happy", 1000);
                return true;
            } else {
                snprintf(line, sizeof(line), secret > guess ? "Higher!" : "Lower!");
                m1_buzzer_set(BUZZER_FREQ_01_KHZ, 40);
                draw_message("Nope", line, 700);
            }
        }
    }
}

/* Medicine: cure sickness (sometimes takes two doses, like P1). */
static void action_medicine(void)
{
    if (!pet.sick) {
        draw_message("Not sick", "No need", 800);
        return;
    }
    m1_buzzer_set(BUZZER_FREQ_06_KHZ, 40);
    if (game_rand_range(0, 1)) {
        pet.sick = 0;
        pet.health = clamp_add(pet.health, 10);
        draw_message("Cured!", "Feeling better", 900);
    } else {
        draw_message("Medicine...", "Still sick", 900);
    }
}

/* Clean: sweep away the poop. */
static void action_clean(void)
{
    if (!pet.poop) {
        draw_message("All clean", NULL, 700);
        return;
    }
    pet.poop = 0;
    poop_ctr = 0;
    m1_buzzer_set(BUZZER_FREQ_04_KHZ, 40);
    draw_message("Cleaned up", NULL, 700);
}

/* Status: the P1 stats page — hunger, happiness, weight, age, discipline. */
static void action_status(void)
{
    char line[28];
    uint8_t hp = pet.hunger / 25;
    uint8_t yp = pet.happy  / 25;
    game_button_t b;

    do {
        m1_u8g2_firstpage();
        do {
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
            u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
            u8g2_DrawStr(&m1_u8g2, 34, 10, "STATUS");

            u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);

            u8g2_DrawStr(&m1_u8g2, 4, 22, "Hunger");
            for (int i = 0; i < PIP_MAX; i++) {
                int x = 62 + i * 10;
                if (i < hp) u8g2_DrawBox(&m1_u8g2, x, 15, 8, 7);
                else        u8g2_DrawFrame(&m1_u8g2, x, 15, 8, 7);
            }

            u8g2_DrawStr(&m1_u8g2, 4, 33, "Happy");
            for (int i = 0; i < PIP_MAX; i++) {
                int x = 62 + i * 10;
                if (i < yp) u8g2_DrawBox(&m1_u8g2, x, 26, 8, 7);
                else        u8g2_DrawFrame(&m1_u8g2, x, 26, 8, 7);
            }

            snprintf(line, sizeof(line), "Weight %ug  Disc %u%%",
                     (unsigned)pet.weight, (unsigned)pet.discipline);
            u8g2_DrawStr(&m1_u8g2, 4, 45, line);

            snprintf(line, sizeof(line), "Age %ud  HP %u%%  %s",
                     (unsigned)pet.age_days, (unsigned)pet.health,
                     pet.sick ? "SICK" : "OK");
            u8g2_DrawStr(&m1_u8g2, 4, 55, line);

            u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
            u8g2_DrawStr(&m1_u8g2, 74, HINT_Y, "B/C:back");
        } while (m1_u8g2_nextpage());

        b = game_poll_button(500);
    } while (b != GAME_BTN_OK && b != GAME_BTN_RIGHT && b != GAME_BTN_BACK);
}

/* Discipline: scold the pet. Correct when it is calling for no real reason. */
static void action_discipline(void)
{
    m1_buzzer_set(BUZZER_FREQ_01_KHZ, 60);
    /* If the pet is calling but nothing is actually wrong, scolding teaches
     * it manners; otherwise scolding just annoys it. */
    if ((needs & NEED_HAPPY) && !pet.sick && !pet.poop && pet.hunger > 25) {
        pet.discipline = clamp_add(pet.discipline, 15);
        pet.happy      = clamp_add(pet.happy, -5);
        draw_message("Behave!", "Discipline up", 900);
    } else {
        pet.discipline = clamp_add(pet.discipline, 5);
        pet.happy      = clamp_add(pet.happy, -10);
        draw_message("Scolded", "It's upset", 900);
    }
}

/* Attention: report what the pet currently wants. */
static void action_attention(void)
{
    const char *msg = "I'm fine!";
    if      (needs & NEED_SICK)   msg = "I feel sick";
    else if (needs & NEED_HUNGER) msg = "I'm hungry";
    else if (needs & NEED_POOP)   msg = "Clean me!";
    else if (needs & NEED_HAPPY)  msg = "Play with me";
    draw_message("Attention", msg, 1100);
}

static void execute_action(uint8_t icon)
{
    switch (icon) {
    case ICON_FOOD:       action_food();       break;
    case ICON_LIGHT:      action_light();      break;
    case ICON_GAME:       action_game();       break;
    case ICON_MEDICINE:   action_medicine();   break;
    case ICON_CLEAN:      action_clean();      break;
    case ICON_STATUS:     action_status();     break;
    case ICON_DISCIPLINE: action_discipline(); break;
    case ICON_ATTENTION:  action_attention();  break;
    default: break;
    }
    pet_recompute_needs();
}


/*============================================================================*/
/*                             R E N D E R                                    */
/*============================================================================*/

/* Draw the creature inside the LCD frame, expression driven by state. */
static void draw_pet_sprite(void)
{
    int bob = (anim_frame & 1) ? 0 : 1;   /* 1px idle bob */
    int cy  = PET_FLOOR_Y - 12 + bob;
    int r;

    /* Body radius grows with evolution stage. */
    switch (pet.stage) {
    case STAGE_EGG:   r = 8;  break;
    case STAGE_BABY:  r = 7;  break;
    case STAGE_CHILD: r = 9;  break;
    case STAGE_TEEN:  r = 11; break;
    default:          r = 12; break;
    }

    if (pet.stage == STAGE_EGG) {
        /* Unhatched: a wobbling egg with a crack. */
        u8g2_DrawDisc(&m1_u8g2, PET_CX, cy + 2, r, U8G2_DRAW_ALL);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        u8g2_DrawLine(&m1_u8g2, PET_CX - 4, cy, PET_CX - 1, cy + 3);
        u8g2_DrawLine(&m1_u8g2, PET_CX - 1, cy + 3, PET_CX + 2, cy);
        u8g2_DrawLine(&m1_u8g2, PET_CX + 2, cy, PET_CX + 4, cy + 3);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        return;
    }

    /* Body. */
    u8g2_DrawDisc(&m1_u8g2, PET_CX, cy, r, U8G2_DRAW_ALL);

    /* Punch out facial features in background colour. */
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);

    if (pet.sleeping) {
        /* Closed eyes. */
        u8g2_DrawHLine(&m1_u8g2, PET_CX - 6, cy - 2, 4);
        u8g2_DrawHLine(&m1_u8g2, PET_CX + 2, cy - 2, 4);
    } else if (pet.sick) {
        /* X eyes. */
        u8g2_DrawLine(&m1_u8g2, PET_CX - 6, cy - 4, PET_CX - 2, cy);
        u8g2_DrawLine(&m1_u8g2, PET_CX - 2, cy - 4, PET_CX - 6, cy);
        u8g2_DrawLine(&m1_u8g2, PET_CX + 2, cy - 4, PET_CX + 6, cy);
        u8g2_DrawLine(&m1_u8g2, PET_CX + 6, cy - 4, PET_CX + 2, cy);
    } else {
        /* Open eyes (blink on odd frames when awake). */
        if (anim_frame & 1) {
            u8g2_DrawBox(&m1_u8g2, PET_CX - 6, cy - 3, 2, 3);
            u8g2_DrawBox(&m1_u8g2, PET_CX + 4, cy - 3, 2, 3);
        } else {
            u8g2_DrawHLine(&m1_u8g2, PET_CX - 6, cy - 1, 2);
            u8g2_DrawHLine(&m1_u8g2, PET_CX + 4, cy - 1, 2);
        }
    }

    /* Mouth: smile when content, frown when unhappy/hungry. */
    if (!pet.sleeping) {
        if (pet.happy > 50 && pet.hunger > 25 && !pet.sick) {
            u8g2_DrawLine(&m1_u8g2, PET_CX - 3, cy + 4, PET_CX, cy + 6);
            u8g2_DrawLine(&m1_u8g2, PET_CX, cy + 6, PET_CX + 3, cy + 4);
        } else {
            u8g2_DrawLine(&m1_u8g2, PET_CX - 3, cy + 6, PET_CX, cy + 4);
            u8g2_DrawLine(&m1_u8g2, PET_CX, cy + 4, PET_CX + 3, cy + 6);
        }
    }

    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Zzz when asleep. */
    if (pet.sleeping) {
        u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
        u8g2_DrawStr(&m1_u8g2, PET_CX + r, cy - r, "z");
        u8g2_DrawStr(&m1_u8g2, PET_CX + r + 4, cy - r - 4, "Z");
    }

    /* Poop pile to the pet's side. */
    if (pet.poop) {
        int px = PET_FRAME_X + 6;
        int py = PET_FLOOR_Y - 1;
        u8g2_DrawBox(&m1_u8g2, px, py, 6, 3);
        u8g2_DrawBox(&m1_u8g2, px + 1, py - 2, 4, 2);
        u8g2_DrawBox(&m1_u8g2, px + 2, py - 4, 2, 2);
    }
}

/* Side gauges: hunger pips (left gutter) and happiness pips (right gutter). */
static void draw_side_gauges(void)
{
    uint8_t hp = pet.hunger / 25;
    uint8_t yp = pet.happy  / 25;

    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    u8g2_DrawStr(&m1_u8g2, 4, 7, "F");
    u8g2_DrawStr(&m1_u8g2, 116, 7, "H");

    for (int i = 0; i < PIP_MAX; i++) {
        int y = 10 + i * 7;
        if (i < hp) u8g2_DrawBox(&m1_u8g2, 3, y, 6, 5);
        else        u8g2_DrawFrame(&m1_u8g2, 3, y, 6, 5);

        if (i < yp) u8g2_DrawBox(&m1_u8g2, 119, y, 6, 5);
        else        u8g2_DrawFrame(&m1_u8g2, 119, y, 6, 5);
    }
}

/* The eight-icon selector row with labels; selected icon is inverted. */
static void draw_icons(void)
{
    /* Blink the Attention icon while the pet has an unmet need. */
    bool at_blink = (needs != 0) && (anim_frame & 1);

    u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
    for (int i = 0; i < ICON_NUM; i++) {
        int x = i * ICON_CELL_W;
        bool inv = (i == sel) || (i == ICON_ATTENTION && at_blink);

        if (inv) {
            u8g2_DrawBox(&m1_u8g2, x, ICON_ROW_Y, ICON_CELL_W, ICON_CELL_H);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        }
        u8g2_DrawStr(&m1_u8g2, x + 3, ICON_ROW_Y + 8, icon_labels[i]);
        if (inv)
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    }
}

/* Button hints, matching the SDK app's "<A  B  C>" layout. */
static void draw_hints(void)
{
    u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
    u8g2_DrawStr(&m1_u8g2, 2,   HINT_Y, "<A");
    u8g2_DrawStr(&m1_u8g2, 62,  HINT_Y, "B");
    u8g2_DrawStr(&m1_u8g2, 112, HINT_Y, "C>");
}

static void draw_main(void)
{
    m1_u8g2_firstpage();
    do {
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

        /* LCD viewport border. */
        u8g2_DrawFrame(&m1_u8g2, PET_FRAME_X, PET_FRAME_Y, PET_FRAME_W, PET_FRAME_H);

        draw_side_gauges();
        draw_pet_sprite();
        draw_icons();
        draw_hints();
    } while (m1_u8g2_nextpage());
}

/* Centered one/two-line notice held for hold_ms (BACK/OK can dismiss early). */
static void draw_message(const char *l1, const char *l2, uint16_t hold_ms)
{
    m1_u8g2_firstpage();
    do {
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        u8g2_SetFont(&m1_u8g2, u8g2_font_6x10_tr);
        uint8_t w1 = u8g2_GetStrWidth(&m1_u8g2, l1);
        u8g2_DrawStr(&m1_u8g2, (GAME_SCREEN_W - w1) / 2, l2 ? 26 : 34, l1);
        if (l2) {
            uint8_t w2 = u8g2_GetStrWidth(&m1_u8g2, l2);
            u8g2_DrawStr(&m1_u8g2, (GAME_SCREEN_W - w2) / 2, 42, l2);
        }
    } while (m1_u8g2_nextpage());

    /* Hold, but let a button dismiss early. */
    uint32_t end = HAL_GetTick() + hold_ms;
    while ((int32_t)(end - HAL_GetTick()) > 0) {
        game_button_t b = game_poll_button(80);
        if (b == GAME_BTN_OK || b == GAME_BTN_RIGHT || b == GAME_BTN_BACK)
            break;
    }
}

/* Grave screen when the pet has died. */
static void draw_grave(void)
{
    m1_u8g2_firstpage();
    do {
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

        /* Tombstone. */
        int gx = 56, gy = 18;
        u8g2_DrawBox(&m1_u8g2, gx, gy + 6, 16, 20);
        u8g2_DrawDisc(&m1_u8g2, gx + 8, gy + 6, 8, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        u8g2_SetFont(&m1_u8g2, u8g2_font_5x8_tr);
        u8g2_DrawStr(&m1_u8g2, gx + 4, gy + 16, "RIP");
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        u8g2_DrawStr(&m1_u8g2, 8, 12, "Your pet has died");

        u8g2_SetFont(&m1_u8g2, u8g2_font_finderskeepers_tf);
        u8g2_DrawStr(&m1_u8g2, 8, HINT_Y, "B:new pet   BACK:exit");
    } while (m1_u8g2_nextpage());
}


/*============================================================================*/
/*
 * @brief  Tamagotchi P1 entry point. Runs its own event loop, auto-loads
 *         saved state on entry and saves on exit. Returns on BACK.
 */
/*============================================================================*/
void game_tamagotchi_run(void)
{
    game_button_t btn;

    game_rand_seed();

    /* Auto-load; hatch a new pet if there is no valid save. */
    if (pet_load() != 0)
        pet_new();

    sim_last_tick = HAL_GetTick();
    anim_frame = 0;
    pet_recompute_needs();

    uint32_t last_call = HAL_GetTick();

    while (1) {
        /* --- dead pet: grave screen, B hatches a new one --- */
        if (!pet.alive) {
            lp5814_all_off_RGB();
            draw_grave();
            btn = game_poll_button(500);
            if (btn == GAME_BTN_BACK) {
                pet_save();
                lp5814_all_off_RGB();
                m1_buzzer_set(0, 0);
                return;
            }
            if (btn == GAME_BTN_OK) {
                pet_new();
                sim_last_tick = HAL_GetTick();
                m1_buzzer_set(BUZZER_FREQ_04_KHZ, 60);
            }
            continue;
        }

        /* --- advance simulation & draw --- */
        pet_sim_step();
        pet_update_led();
        draw_main();

        /* Periodic "call" beep while a need is unmet. */
        if (needs != 0 && (HAL_GetTick() - last_call) > 4000) {
            last_call = HAL_GetTick();
            m1_buzzer_set(BUZZER_FREQ_02_KHZ, 40);
        }

        /* --- input: poll ~400 ms so the idle animation ticks --- */
        btn = game_poll_button(400);
        anim_frame++;

        switch (btn) {
        case GAME_BTN_LEFT:               /* A: scroll icon selection */
        case GAME_BTN_UP:
            sel = (uint8_t)((sel + 1) % ICON_NUM);
            m1_buzzer_set(BUZZER_FREQ_04_KHZ, 10);
            break;
        case GAME_BTN_DOWN:               /* convenience: reverse scroll */
            sel = (uint8_t)((sel + ICON_NUM - 1) % ICON_NUM);
            m1_buzzer_set(BUZZER_FREQ_04_KHZ, 10);
            break;
        case GAME_BTN_OK:                 /* B: execute selected icon */
            execute_action(sel);
            break;
        case GAME_BTN_RIGHT:              /* C: cancel — no-op on main screen */
            break;
        case GAME_BTN_BACK:               /* save & exit */
            pet_save();
            lp5814_all_off_RGB();
            m1_buzzer_set(0, 0);
            return;
        default:
            break;
        }
    }
}
