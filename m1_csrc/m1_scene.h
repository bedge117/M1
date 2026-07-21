/* See COPYING.txt for license details. */

/**
 * @file   m1_scene.h
 * @brief  Generic Scene Manager — reusable scene framework for M1 modules.
 *
 * Provides a stack-based scene manager, button-event translation, a generic
 * event loop, and a standard menu draw helper.  All modules except Sub-GHz
 * (which has radio-specific event handling) share this framework.
 *
 * Usage:
 *   1.  Define a scene ID enum starting at 0 (menu scene) for your module.
 *   2.  Define M1SceneHandlers for each scene (blocking delegates + menus).
 *   3.  Build a scene registry array indexed by scene ID.
 *   4.  Call m1_scene_run() with the registry, scene count, and optional
 *       hardware init/deinit callbacks.
 */

#ifndef M1_SCENE_H_
#define M1_SCENE_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Scene manager limits                                                       */
/*============================================================================*/

#define M1_SCENE_STACK_MAX  8   /**< Maximum scene stack depth */

/*============================================================================*/
/* Generic event types (button events only)                                   */
/*============================================================================*/

typedef enum {
    M1SceneEventNone = 0,
    M1SceneEventBack,
    M1SceneEventOk,
    M1SceneEventUp,
    M1SceneEventDown,
    M1SceneEventLeft,
    M1SceneEventRight,
} M1SceneEvent;

/*============================================================================*/
/* Application context — shared across all scenes in a module                 */
/*============================================================================*/

typedef struct M1SceneApp M1SceneApp;

struct M1SceneApp {
    /* Scene stack */
    uint8_t scene_stack[M1_SCENE_STACK_MAX];
    uint8_t scene_depth;

    /* Flags */
    bool    need_redraw;
    bool    running;

    /* Registry (set during init, used by push/pop/replace) */
    const struct M1SceneHandlers *const *registry;
    uint8_t scene_count;
};

/*============================================================================*/
/* Scene handler callbacks                                                    */
/*============================================================================*/

typedef void (*M1SceneOnEnter)(M1SceneApp *app);
typedef bool (*M1SceneOnEvent)(M1SceneApp *app, M1SceneEvent event);
typedef void (*M1SceneOnExit)(M1SceneApp *app);
typedef void (*M1SceneDraw)(M1SceneApp *app);

typedef struct M1SceneHandlers {
    M1SceneOnEnter on_enter;
    M1SceneOnEvent on_event;
    M1SceneOnExit  on_exit;
    M1SceneDraw    draw;
} M1SceneHandlers;

/*============================================================================*/
/* Scene Manager API                                                          */
/*============================================================================*/

/** Initialize the scene manager.  Must be called before push/pop. */
void m1_scene_init(M1SceneApp *app,
                   const M1SceneHandlers *const *registry,
                   uint8_t scene_count);

/** Push a new scene onto the stack (exits current, enters new). */
void m1_scene_push(M1SceneApp *app, uint8_t scene);

/** Pop the current scene (exits current, re-enters previous).
 *  If stack is empty after pop, sets app->running = false. */
void m1_scene_pop(M1SceneApp *app);

/** Replace the top scene (exit current, enter replacement). */
void m1_scene_replace(M1SceneApp *app, uint8_t scene);

/** Dispatch an event to the current scene's on_event handler. */
bool m1_scene_send_event(M1SceneApp *app, M1SceneEvent event);

/** Call the current scene's draw handler. */
void m1_scene_draw(M1SceneApp *app);

/** Get the currently active scene ID. */
uint8_t m1_scene_current(const M1SceneApp *app);

/*============================================================================*/
/* Generic event loop                                                         */
/*============================================================================*/

/**
 * @brief  Run the generic scene loop (blocking).
 *
 * Initializes the app context, optionally calls hw_init, pushes scene 0
 * (the module's menu scene), and processes button events until
 * app->running becomes false.  On exit, optionally calls hw_deinit,
 * resets the FreeRTOS queue, and sends Q_EVENT_MENU_EXIT.
 *
 * @param registry    Scene handler registry array.
 * @param scene_count Number of entries in registry.
 * @param hw_init     Hardware init function (can be NULL).
 * @param hw_deinit   Hardware deinit function (can be NULL).
 */
void m1_scene_run(const M1SceneHandlers *const *registry,
                  uint8_t scene_count,
                  void (*hw_init)(void),
                  void (*hw_deinit)(void));

/*============================================================================*/
/* Menu drawing helper                                                        */
/*============================================================================*/

/** Menu layout constants — highlight starts at M1_MENU_HIGHLIGHT_X (1px
 *  left inset) and leaves a 1px gap before the scrollbar on the right. */
#define M1_MENU_AREA_TOP      12  /**< Y below title + separator line       */
#define M1_MENU_AREA_H        52  /**< Available height for menu items (64-12) */
#define M1_MENU_ITEM_H_SMALL   8  /**< Pixel height per row — Small mode    */
#define M1_MENU_ITEM_H_MEDIUM 10  /**< Pixel height per row — Medium mode   */
#define M1_MENU_ITEM_H_LARGE  13  /**< Pixel height per row — Large mode    */
#define M1_MENU_HIGHLIGHT_X    1  /**< Highlight left x-inset (1px from edge) */
#define M1_MENU_SCROLLBAR_X  124  /**< Scrollbar left edge (3px wide)       */
#define M1_MENU_SCROLLBAR_W    3  /**< Scrollbar track width                */
/** Highlight/text area width: from M1_MENU_HIGHLIGHT_X to 1px before scrollbar */
#define M1_MENU_TEXT_W  (M1_MENU_SCROLLBAR_X - M1_MENU_HIGHLIGHT_X - 1)

/**
 * @brief  Get current menu item height based on m1_menu_style setting.
 * @return 8 (Small), 10 (Medium), or 13 (Large).
 */
uint8_t m1_menu_item_h(void);

/**
 * @brief  Get maximum visible menu items for the standard 52px menu area.
 * @return 6 (Small), 5 (Medium), or 4 (Large).
 */
uint8_t m1_menu_max_visible(void);

/**
 * @brief  Get the u8g2 font for menu items based on m1_menu_style.
 * @return NokiaSmallPlain (Small), profont12 (Medium), or nine_by_five_nbp (Large).
 */
const uint8_t *m1_menu_font(void);

/** Convenience: min of item count and max visible for current style */
#define M1_MENU_VIS(count) \
    ((uint8_t)((count) < m1_menu_max_visible() ? (count) : m1_menu_max_visible()))

/**
 * @brief  Draw a scrollable menu with title and scrollbar.
 *
 * Renders the same Flipper-style menu used by Sub-GHz: title bar,
 * separator line, highlighted selection, proportional scrollbar.
 *
 * @param title    Title string (drawn centred at top).
 * @param labels   Array of item label strings.
 * @param count    Total number of items.
 * @param sel      Currently selected item index.
 * @param scroll   Top visible item index.
 * @param visible  Number of items visible at once.
 */
void m1_scene_draw_menu(const char *title,
                        const char *const *labels,
                        uint8_t count,
                        uint8_t sel,
                        uint8_t scroll,
                        uint8_t visible);

/**
 * @brief  Generic menu event handler.
 *
 * Handles Up/Down/OK/Back for a scrollable menu.  On OK, pushes the
 * target scene from the targets array.  On Back, pops the current scene.
 *
 * @param app      Application context.
 * @param event    The event to handle.
 * @param sel      Pointer to selected index (updated on Up/Down).
 * @param scroll   Pointer to scroll offset (updated on Up/Down).
 * @param count    Total number of menu items.
 * @param visible  Number of visible items.
 * @param targets  Array of scene IDs to push on OK.
 * @retval true    Event was consumed.
 * @retval false   Event not consumed.
 */
bool m1_scene_menu_event(M1SceneApp *app,
                         M1SceneEvent event,
                         uint8_t *sel,
                         uint8_t *scroll,
                         uint8_t count,
                         uint8_t visible,
                         const uint8_t *targets);

#endif /* M1_SCENE_H_ */
