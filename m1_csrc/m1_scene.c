/* See COPYING.txt for license details. */

/**
 * @file   m1_scene.c
 * @brief  Generic Scene Manager — stack-based scene dispatch engine.
 *
 * Shared implementation for all M1 modules that use scene-based architecture
 * (everything except Sub-GHz, which has its own radio-specific event loop).
 * Provides push/pop/replace, button-event translation, a generic blocking
 * event loop, and a standard Flipper-style menu draw helper.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "m1_tasks.h"
#include "m1_system.h"
#include "m1_scene.h"
#include "m1_display.h"
#include "m1_lcd.h"
#include "m1_lib.h"

/*============================================================================*/
/* Scene Manager API                                                          */
/*============================================================================*/

void m1_scene_init(M1SceneApp *app,
                   const M1SceneHandlers *const *registry,
                   uint8_t scene_count)
{
    memset(app, 0, sizeof(*app));
    app->registry    = registry;
    app->scene_count = scene_count;
    app->running     = true;
}

static const M1SceneHandlers *get_handlers(const M1SceneApp *app, uint8_t id)
{
    if (id < app->scene_count && app->registry[id])
        return app->registry[id];
    return NULL;
}

uint8_t m1_scene_current(const M1SceneApp *app)
{
    if (app->scene_depth == 0)
        return 0;  /* fallback to menu scene */
    return app->scene_stack[app->scene_depth - 1];
}

void m1_scene_push(M1SceneApp *app, uint8_t scene)
{
    /* Exit current scene if any */
    if (app->scene_depth > 0)
    {
        const M1SceneHandlers *cur = get_handlers(app, m1_scene_current(app));
        if (cur && cur->on_exit)
            cur->on_exit(app);
    }

    /* Push new scene */
    if (app->scene_depth < M1_SCENE_STACK_MAX)
        app->scene_stack[app->scene_depth++] = scene;

    /* Enter new scene */
    const M1SceneHandlers *next = get_handlers(app, scene);
    if (next && next->on_enter)
        next->on_enter(app);

    app->need_redraw = true;
}

void m1_scene_pop(M1SceneApp *app)
{
    if (app->scene_depth == 0)
    {
        app->running = false;
        return;
    }

    /* Exit current scene */
    const M1SceneHandlers *cur = get_handlers(app, m1_scene_current(app));
    if (cur && cur->on_exit)
        cur->on_exit(app);

    app->scene_depth--;

    if (app->scene_depth == 0)
    {
        app->running = false;
        return;
    }

    /* Re-enter the scene that's now on top */
    const M1SceneHandlers *prev = get_handlers(app, m1_scene_current(app));
    if (prev && prev->on_enter)
        prev->on_enter(app);

    app->need_redraw = true;
}

void m1_scene_replace(M1SceneApp *app, uint8_t scene)
{
    /* Exit current scene */
    if (app->scene_depth > 0)
    {
        const M1SceneHandlers *cur = get_handlers(app, m1_scene_current(app));
        if (cur && cur->on_exit)
            cur->on_exit(app);
        app->scene_stack[app->scene_depth - 1] = scene;
    }
    else
    {
        app->scene_stack[app->scene_depth++] = scene;
    }

    /* Enter replacement scene */
    const M1SceneHandlers *next = get_handlers(app, scene);
    if (next && next->on_enter)
        next->on_enter(app);

    app->need_redraw = true;
}

bool m1_scene_send_event(M1SceneApp *app, M1SceneEvent event)
{
    const M1SceneHandlers *cur = get_handlers(app, m1_scene_current(app));
    if (cur && cur->on_event)
        return cur->on_event(app, event);
    return false;
}

void m1_scene_draw(M1SceneApp *app)
{
    const M1SceneHandlers *cur = get_handlers(app, m1_scene_current(app));
    if (cur && cur->draw)
        cur->draw(app);
}

/*============================================================================*/
/* Button-event translation                                                   */
/*============================================================================*/

static M1SceneEvent translate_button(void)
{
    S_M1_Buttons_Status btn;
    BaseType_t ret = xQueueReceive(button_events_q_hdl, &btn, 0);
    if (ret != pdTRUE)
        return M1SceneEventNone;

    if (btn.event[BUTTON_OK_KP_ID]    == BUTTON_EVENT_CLICK) return M1SceneEventOk;
    if (btn.event[BUTTON_BACK_KP_ID]  == BUTTON_EVENT_CLICK) return M1SceneEventBack;
    if (btn.event[BUTTON_LEFT_KP_ID]  == BUTTON_EVENT_CLICK) return M1SceneEventLeft;
    if (btn.event[BUTTON_RIGHT_KP_ID] == BUTTON_EVENT_CLICK) return M1SceneEventRight;
    if (btn.event[BUTTON_UP_KP_ID]    == BUTTON_EVENT_CLICK) return M1SceneEventUp;
    if (btn.event[BUTTON_DOWN_KP_ID]  == BUTTON_EVENT_CLICK) return M1SceneEventDown;

    return M1SceneEventNone;
}

/*============================================================================*/
/* Generic event loop                                                         */
/*============================================================================*/

void m1_scene_run(const M1SceneHandlers *const *registry,
                  uint8_t scene_count,
                  void (*hw_init)(void),
                  void (*hw_deinit)(void))
{
    M1SceneApp app;
    S_M1_Main_Q_t q_item;
    BaseType_t ret;

    /* Initialize scene manager */
    m1_scene_init(&app, registry, scene_count);

    /* Hardware init (optional) */
    if (hw_init)
        hw_init();

    /* Push initial scene (scene 0 = module menu) */
    m1_scene_push(&app, 0);
    m1_scene_draw(&app);
    app.need_redraw = false;

    /* Main event loop — button events only */
    while (app.running)
    {
        ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);

        if (ret == pdTRUE)
        {
            M1SceneEvent evt = M1SceneEventNone;

            if (q_item.q_evt_type == Q_EVENT_KEYPAD)
                evt = translate_button();

            if (evt != M1SceneEventNone)
                m1_scene_send_event(&app, evt);
        }

        if (app.need_redraw)
        {
            m1_scene_draw(&app);
            app.need_redraw = false;
        }
    }

    /* Hardware deinit (optional) */
    if (hw_deinit)
        hw_deinit();

    /* Drain any stale button events so the main menu handler does not
     * process a leftover press from the module that just exited. */
    xQueueReset(button_events_q_hdl);
    xQueueReset(main_q_hdl);
}

/*============================================================================*/
/* Menu layout helpers                                                        */
/*============================================================================*/

uint8_t m1_menu_item_h(void)
{
    switch (m1_menu_style)
    {
    case 1:  return M1_MENU_ITEM_H_MEDIUM;
    case 2:  return M1_MENU_ITEM_H_LARGE;
    default: return M1_MENU_ITEM_H_SMALL;
    }
}

uint8_t m1_menu_max_visible(void)
{
    return M1_MENU_AREA_H / m1_menu_item_h();
}

const uint8_t *m1_menu_font(void)
{
    switch (m1_menu_style)
    {
    case 1:  return M1_DISP_FUNC_MENU_FONT_N;
    case 2:  return M1_DISP_FUNC_MENU_FONT_N2;
    default: return M1_DISP_SUB_MENU_FONT_N;
    }
}

/*============================================================================*/
/* Menu drawing helper                                                        */
/*============================================================================*/

void m1_scene_draw_menu(const char *title,
                        const char *const *labels,
                        uint8_t count,
                        uint8_t sel,
                        uint8_t scroll,
                        uint8_t visible)
{
    const uint8_t item_h   = m1_menu_item_h();
    const uint8_t text_ofs = item_h - 1;

    /* ---- Portrait (R1, 64x128): full-width list spread down the tall screen.
     * Same visible-count/scroll the caller computed, only the geometry differs. */
    if ( m1_screen_orientation == M1_ORIENT_PORTRAIT )
    {
        int pw = u8g2_GetDisplayWidth(&m1_u8g2);    /* 64 */
        int ph = u8g2_GetDisplayHeight(&m1_u8g2);   /* 128 */
        const int area_top = 14;
        int pih = visible ? (ph - area_top) / visible : (ph - area_top);
        if (pih < 12) pih = 12;

        m1_u8g2_firstpage();
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
        m1_draw_text(&m1_u8g2, 0, 9, pw, title, TEXT_ALIGN_CENTER);
        u8g2_DrawHLine(&m1_u8g2, 0, 11, pw);

        u8g2_SetFont(&m1_u8g2, m1_menu_font());
        for (uint8_t i = 0; i < visible && (scroll + i) < count; i++)
        {
            uint8_t idx = scroll + i;
            int y = area_top + i * pih;
            if (idx == sel)
            {
                u8g2_DrawRBox(&m1_u8g2, 0, y, pw - 4, pih - 2, 2);
                u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
            }
            u8g2_DrawStr(&m1_u8g2, 3, y + pih - 5, labels[idx]);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
        }
        if (count > 0)
        {
            int sb_area_h = visible * pih;
            int sb_h = sb_area_h / count; if (sb_h < 6) sb_h = 6;
            int sb_travel = (sb_area_h > sb_h) ? sb_area_h - sb_h : 0;
            int sb_y = area_top + (count > 1 ? sb_travel * sel / (count - 1) : 0);
            u8g2_DrawVLine(&m1_u8g2, pw - 2, area_top, sb_area_h);
            u8g2_DrawRBox(&m1_u8g2, pw - 3, sb_y, 3, sb_h, 1);
        }
        m1_u8g2_nextpage();
        return;
    }

    m1_u8g2_firstpage();
    u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

    /* Title */
    u8g2_SetFont(&m1_u8g2, M1_DISP_FUNC_MENU_FONT_N);
    m1_draw_text(&m1_u8g2, 2, 9, 120, title, TEXT_ALIGN_CENTER);

    /* Separator line */
    u8g2_DrawHLine(&m1_u8g2, 0, 10, M1_LCD_DISPLAY_WIDTH);

    /* Menu items */
    u8g2_SetFont(&m1_u8g2, m1_menu_font());
    for (uint8_t i = 0; i < visible && (scroll + i) < count; i++)
    {
        uint8_t idx = scroll + i;
        uint8_t y = M1_MENU_AREA_TOP + i * item_h;

        if (idx == sel)
        {
            /* Highlight selected item — rounded corners, leave room for scrollbar */
            u8g2_DrawRBox(&m1_u8g2, 1, y, M1_MENU_TEXT_W, item_h, 2);
            u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
        }

        u8g2_DrawStr(&m1_u8g2, 4, y + text_ofs, labels[idx]);
        u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    }

    /* Scrollbar — proportional position indicator on the right edge */
    if (count > 0)
    {
        uint8_t sb_area_h   = visible * item_h;
        uint8_t sb_handle_h = sb_area_h / count;
        if (sb_handle_h < 6)
            sb_handle_h = 6;
        uint8_t sb_travel_h = (sb_area_h > sb_handle_h) ? (sb_area_h - sb_handle_h) : 0;
        uint8_t sb_handle_y = M1_MENU_AREA_TOP;
        if (count > 1)
            sb_handle_y += (uint8_t)((uint16_t)sb_travel_h * sel / (count - 1));

        /* Track — single centerline pixel */
        u8g2_DrawVLine(&m1_u8g2, M1_MENU_SCROLLBAR_X + M1_MENU_SCROLLBAR_W / 2,
                       M1_MENU_AREA_TOP, sb_area_h);
        /* Handle — filled rounded rectangle */
        u8g2_DrawRBox(&m1_u8g2, M1_MENU_SCROLLBAR_X, sb_handle_y,
                      M1_MENU_SCROLLBAR_W, sb_handle_h, 1);
    }

    m1_u8g2_nextpage();
}

/*============================================================================*/
/* Generic menu event handler                                                 */
/*============================================================================*/

bool m1_scene_menu_event(M1SceneApp *app,
                         M1SceneEvent event,
                         uint8_t *sel,
                         uint8_t *scroll,
                         uint8_t count,
                         uint8_t visible,
                         const uint8_t *targets)
{
    switch (event)
    {
        case M1SceneEventBack:
            m1_scene_pop(app);
            return true;

        case M1SceneEventUp:
            *sel = (*sel > 0) ? *sel - 1 : count - 1;
            /* Adjust scroll window */
            if (*sel < *scroll)
                *scroll = *sel;
            else if (*sel >= *scroll + visible)
                *scroll = *sel - visible + 1;
            app->need_redraw = true;
            return true;

        case M1SceneEventDown:
            *sel = (*sel + 1) % count;
            /* Adjust scroll window */
            if (*sel >= *scroll + visible)
                *scroll = *sel - visible + 1;
            /* Handle wrap-around */
            if (*sel == 0)
                *scroll = 0;
            app->need_redraw = true;
            return true;

        case M1SceneEventOk:
            if (targets && targets[*sel] < app->scene_count)
                m1_scene_push(app, targets[*sel]);
            return true;

        default:
            break;
    }
    return false;
}
