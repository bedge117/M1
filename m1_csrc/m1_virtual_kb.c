/* See COPYING.txt for license details. */

/*
*
* m1_virtual_kb.c
*
* Library for the virtual keyboard
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <string.h>
#include "stm32h5xx_hal.h"
#include "main.h"
#include "u8g2.h"
#include "mui.h"
#include "m1_virtual_kb.h"

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG				"VKB"

#define M1_VIRTUAL_KB_COLUMN_SIZE		14
#define M1_VIRTUAL_KB_ROW_SIZE			3

#define M1_VIRTUAL_KEY_BS				0x08 // Backspace
#define M1_VIRTUAL_KEY_ENTER			0x0A
#define M1_VIRTUAL_KEY_NONE				0x00 // NULL

#define M1_VIRTUAL_KB_FILENAME_MAX		20
//#define M1_VIRTUAL_KBS_DATA_MAX			14 // 10 + 4 spaces
uint8_t M1_VIRTUAL_KBS_DATA_MAX;

#define M1_VIRTUAL_KB_FONT_N			u8g2_font_resoledmedium_tr //u8g2_font_6x10_tf // u8g2_font_5x8_tf, u8g2_font_7x13_tf

#define M1_VIRTUAL_KB_FONT_SIZE_5x8		0
#define M1_VIRTUAL_KB_FONT_SIZE_6x10	1
#define M1_VIRTUAL_KB_FONT_SIZE_7x13	2

#define M1_VIRTUAL_KB_FONT_SIZE			M1_VIRTUAL_KB_FONT_SIZE_6x10

#if M1_VIRTUAL_KB_FONT_SIZE==M1_VIRTUAL_KB_FONT_SIZE_5x8

#define M1_VKB_GUI_FONT_WIDTH				5	// pixel
#define M1_VKB_GUI_FONT_HEIGHT				8	// pixel
#define M1_VKB_FONT_HEIGHT_SPACING			3	// pixel
#define M1_VKB_FONT_WIDTH_SPACING			4	// pixel

#elif M1_VIRTUAL_KB_FONT_SIZE==M1_VIRTUAL_KB_FONT_SIZE_6x10

#define M1_VKB_GUI_FONT_WIDTH				6	// pixel
#define M1_VKB_GUI_FONT_HEIGHT				10	// pixel
#define M1_VKB_FONT_HEIGHT_SPACING			2	// pixel
#define M1_VKB_FONT_WIDTH_SPACING			3	// pixel

#endif // #if M1_VIRTUAL_KB_FONT_SIZE==M1_VIRTUAL_KB_FONT_SIZE_5x8

// Defines for VKB
#define M1_VIRTUAL_KBS_COLUMN_SIZE			9
#define M1_VIRTUAL_KBS_ROW_SIZE				2

#define M1_VKB_FIRST_ROW_TOP_POS_Y			28
#define M1_VKB_LEFT_POS_X					M1_VKB_FONT_WIDTH_SPACING

#define M1_VKB_DESCRIPTION_POS_Y			12
#define M1_VKB_DESCRIPTION_POS_X			2

#define M1_VKB_FILENAME_POS_Y				25
#define M1_VKB_FILENAME_POS_X				2
#define M1_VKB_FILENAME_FRAME_POS_Y	14
#define M1_VKB_FILENAME_FRAME_POS_X	0
#define M1_VKB_FILENAME_FRAME_WIDTH			M1_LCD_DISPLAY_WIDTH
#define M1_VKB_FILENAME_FRAME_HEIGHT		14

#define M1_VKB_BACKSPACE_X					(M1_VKB_LEFT_POS_X + 9*(M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING))
#define M1_VKB_BACKSPACE_Y					(M1_VKB_FIRST_ROW_TOP_POS_Y + 1*M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING)
#define M1_VKB_BACKSPACE_ICON_W				15
#define M1_VKB_BACKSPACE_ICON_H				10

#define M1_VKB_ENTER_X						(M1_VKB_LEFT_POS_X + 9*(M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING))
#define M1_VKB_ENTER_Y						(M1_VKB_FIRST_ROW_TOP_POS_Y + 2*M1_VKB_GUI_FONT_HEIGHT + 2*M1_VKB_FONT_HEIGHT_SPACING)
#define M1_VKB_ENTER_ICON_W					15
#define M1_VKB_ENTER_ICON_H					10

#define M1_VKB_DEFAULT_KEY_MAP_COL_ID		9
#define M1_VKB_DEFAULT_KEY_MAP_ROW_ID		2

#define M1_VKB_NUM_OF_FUNCTION_KEYS			2

// Defines for VKBS
#define M1_VKBS_FONT_HEIGHT_SPACING			6	// pixel
#define M1_VKBS_FONT_WIDTH_SPACING			7	// pixel

#define M1_VKBS_FIRST_ROW_TOP_POS_Y			34
#define M1_VKBS_LEFT_POS_X					3

#define M1_VKBS_DESCRIPTION_POS_Y			12
#define M1_VKBS_DESCRIPTION_POS_X			2

#define M1_VKBS_DATA_POS_Y					27//25
#define M1_VKBS_DATA_POS_X					3
#define M1_VKBS_DATA_FRAME_POS_Y			16//14
#define M1_VKBS_DATA_FRAME_POS_X			0
#define M1_VKBS_DATA_FRAME_WIDTH			M1_LCD_DISPLAY_WIDTH
#define M1_VKBS_DATA_FRAME_HEIGHT			14

#define M1_VKBS_BACKSPACE_X					(M1_LCD_DISPLAY_WIDTH - M1_VKBS_BACKSPACE_ICON_W - 3 )
#define M1_VKBS_BACKSPACE_Y					(M1_VKBS_FIRST_ROW_TOP_POS_Y)
#define M1_VKBS_BACKSPACE_ICON_W			15
#define M1_VKBS_BACKSPACE_ICON_H			10

#define M1_VKBS_ENTER_X						(M1_LCD_DISPLAY_WIDTH - M1_VKBS_ENTER_ICON_W - 3 )
#define M1_VKBS_ENTER_Y						(M1_VKBS_FIRST_ROW_TOP_POS_Y + M1_VKB_GUI_FONT_HEIGHT + M1_VKBS_FONT_HEIGHT_SPACING)
#define M1_VKBS_ENTER_ICON_W				15
#define M1_VKBS_ENTER_ICON_H				10

#define M1_VKBS_DEFAULT_KEY_MAP_COL_ID		0
#define M1_VKBS_DEFAULT_KEY_MAP_ROW_ID		0

#define M1_VKBS_NUM_OF_FUNCTION_KEYS		2

//************************** C O N S T A N T **********************************/

// 'backspace key', 15x10px
const uint8_t m1_virtual_kb_icon_backspace[] = {
/*
	0x00, 0x00, 0x10, 0x00, 0x18, 0x00, 0xfc, 0x3f, 0xfe, 0x3f, 0xfe, 0x3f, 0xfc, 0x3f, 0x18, 0x00,
	0x10, 0x00, 0x00, 0x00
*/
	0x00, 0x00, 0xf8, 0x7f, 0x1c, 0x60, 0xce, 0x66, 0x87, 0x63, 0x87, 0x63, 0xce, 0x66, 0x1c, 0x60,
	0xf8, 0x7f, 0x00, 0x00
};

// 'backspace key', 15x10px
const uint8_t m1_virtual_kb_icon_backspace_inv[] = {
/*
	0xff, 0x7f, 0xef, 0x7f, 0xe7, 0x7f, 0x03, 0x40, 0x01, 0x40, 0x01, 0x40, 0x03, 0x40, 0xe7, 0x7f,
	0xef, 0x7f, 0xff, 0x7f
*/
	0xff, 0x7f, 0x07, 0x00, 0xe3, 0x1f, 0x31, 0x19, 0x78, 0x1c, 0x78, 0x1c, 0x31, 0x19, 0xe3, 0x1f,
	0x07, 0x00, 0xff, 0x7f
};

// 'enter key', 15x10px
const uint8_t m1_virtual_kb_icon_enter[] = {
	0x10, 0x1e, 0x18, 0x1e, 0x1c, 0x1e, 0xfe, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xfe, 0x1f, 0x1c, 0x00,
	0x18, 0x00, 0x10, 0x00
};

// 'enter key', 15x10px
const uint8_t m1_virtual_kb_icon_enter_inv[] = {
	0xef, 0x61, 0xe7, 0x61, 0xe3, 0x61, 0x01, 0x60, 0x00, 0x60, 0x00, 0x60, 0x01, 0x60, 0xe3, 0x7f,
	0xe7, 0x7f, 0xef, 0x7f
};

#define M1_VKB_PAGE_COUNT    3
#define M1_VKB_PAGE_LOWER    0
#define M1_VKB_PAGE_UPPER    1
#define M1_VKB_PAGE_SYMBOL   2

/* Page indicator strings shown in top-right corner */
static const char *vkb_page_labels[M1_VKB_PAGE_COUNT] = { "abc", "ABC", "#$%" };

const uint8_t m1_vkb_pages[M1_VKB_PAGE_COUNT][M1_VIRTUAL_KB_ROW_SIZE][M1_VIRTUAL_KB_COLUMN_SIZE] =
{
	/* Page 0: lowercase */
	{
		{'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', 					'0', 					'1', '2', '3'},
		{'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', M1_VIRTUAL_KEY_BS, 	M1_VIRTUAL_KEY_BS,		'4', '5', '6'},
		{'z', 'x', 'c', 'v', 'b', 'n', 'm', '_', '.', M1_VIRTUAL_KEY_ENTER, M1_VIRTUAL_KEY_ENTER, 	'7', '8', '9'}
	},
	/* Page 1: UPPERCASE */
	{
		{'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', 					'0', 					'1', '2', '3'},
		{'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', M1_VIRTUAL_KEY_BS, 	M1_VIRTUAL_KEY_BS,		'4', '5', '6'},
		{'Z', 'X', 'C', 'V', 'B', 'N', 'M', '_', '.', M1_VIRTUAL_KEY_ENTER, M1_VIRTUAL_KEY_ENTER, 	'7', '8', '9'}
	},
	/* Page 2: symbols */
	{
		{'!', '@', '#', '$', '%', '^', '&', '*', '(', ')', 					'0', 					'1', '2', '3'},
		{'-', '+', '=', '[', ']', '{', '}', ':', ';', M1_VIRTUAL_KEY_BS, 	M1_VIRTUAL_KEY_BS,		'4', '5', '6'},
		{'~', '\'', '"', '<', '>', ',', '?', '/', '|', M1_VIRTUAL_KEY_ENTER, M1_VIRTUAL_KEY_ENTER, 	'7', '8', '9'}
	}
};

/* Alias for backward compat — points to page 0 (lowercase) */
const uint8_t (*m1_vkb_map)[M1_VIRTUAL_KB_COLUMN_SIZE] = m1_vkb_pages[0];

const uint8_t m1_vkbs_map[M1_VIRTUAL_KBS_ROW_SIZE][M1_VIRTUAL_KBS_COLUMN_SIZE] =
{
		{'1', '2', '3', '4', '5', 'A', 'B', 'C', M1_VIRTUAL_KEY_BS},
		{'6', '7', '8', '9', '0', 'D', 'E', 'F', M1_VIRTUAL_KEY_ENTER}
};


const uint8_t m1_vkb_func_key_codes[M1_VKB_NUM_OF_FUNCTION_KEYS] = {M1_VIRTUAL_KEY_BS, M1_VIRTUAL_KEY_ENTER};

//************************** S T R U C T U R E S *******************************

typedef struct
{
	const uint8_t *icon_reg; // icon in normal mode
	const uint8_t *icon_inv; // icon in inverted mode
	uint8_t icon_w, icon_h; // icon width and height
	uint8_t icon_x[2], icon_y[2]; // icon position x and y
	uint8_t col_factor; // column factor of this icon compared to that of a normal character
	uint8_t key_row_id, key_col_id;
} S_M1_VKB_X_Key;

typedef enum
{
	M1_VKB_FUNC_BS_KEY_ID = 0,
	M1_VKB_FUNC_ENTER_KEY_ID,
	M1_VKB_FUNC_UNDEFINED_KEY_ID
} S_M1_VKB_Func_Key_ID;

/***************************** V A R I A B L E S ******************************/
S_M1_VKB_X_Key m1_x_keys[M1_VKB_NUM_OF_FUNCTION_KEYS] =
{
		{	.icon_reg = m1_virtual_kb_icon_backspace,
			.icon_inv = m1_virtual_kb_icon_backspace_inv,
			.icon_w = M1_VKB_BACKSPACE_ICON_W,
			.icon_h = M1_VKB_BACKSPACE_ICON_H,
			.icon_x = {M1_VKB_BACKSPACE_X, M1_VKBS_BACKSPACE_X},
			.icon_y = {M1_VKB_BACKSPACE_Y, M1_VKBS_BACKSPACE_Y},
			.col_factor = 2,
			.key_col_id = 9,
			.key_row_id = 1
		},
		{	.icon_reg = m1_virtual_kb_icon_enter,
			.icon_inv = m1_virtual_kb_icon_enter_inv,
			.icon_w = M1_VKB_ENTER_ICON_W,
			.icon_h = M1_VKB_ENTER_ICON_H,
			.icon_x = {M1_VKB_ENTER_X, M1_VKBS_ENTER_X},
			.icon_y = {M1_VKB_ENTER_Y, M1_VKBS_ENTER_Y},
			.col_factor = 2,
			.key_col_id = 9,
			.key_row_id = 2
		}
};

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

uint8_t m1_vkb_get_filename(char *description, char *default_name, char *new_name);
uint8_t m1_vkbs_get_data(char *description, char *data_buffer);
S_M1_VKB_Func_Key_ID m1_vkb_check_function_key(uint8_t kb_key);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/* Page indicator position: top-right corner */
#define M1_VKB_PAGE_IND_X   108
#define M1_VKB_PAGE_IND_Y   M1_VKB_DESCRIPTION_POS_Y
#define M1_VKB_PAGE_IND_W   20
#define M1_VKB_PAGE_IND_H   12

/*============================================================================*/
/*
 * Redraw the keyboard area after a page switch.
 * Clears the key area, redraws all keys from the given page,
 * redraws function-key icons and the page indicator, then
 * highlights the key at (row_id, col_id).
 */
/*============================================================================*/
static void vkb_redraw_page(uint8_t page, uint8_t row_id, uint8_t col_id,
                            uint8_t *out_x, uint8_t *out_y,
                            S_M1_VKB_Func_Key_ID *out_xkey)
{
	uint8_t r, c, xp, yp;
	char k[2] = {0, 0};

	/* Clear keyboard area */
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	u8g2_DrawBox(&m1_u8g2, 0, M1_VKB_FIRST_ROW_TOP_POS_Y,
	             M1_LCD_DISPLAY_WIDTH, M1_LCD_DISPLAY_HEIGHT - M1_VKB_FIRST_ROW_TOP_POS_Y);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);

	/* Redraw all keys */
	u8g2_SetFont(&m1_u8g2, M1_VIRTUAL_KB_FONT_N);
	yp = M1_VKB_FIRST_ROW_TOP_POS_Y + M1_VKB_GUI_FONT_HEIGHT;
	for (r = 0; r < M1_VIRTUAL_KB_ROW_SIZE; r++)
	{
		xp = M1_VKB_LEFT_POS_X;
		for (c = 0; c < M1_VIRTUAL_KB_COLUMN_SIZE; c++)
		{
			k[0] = m1_vkb_pages[page][r][c];
			u8g2_DrawStr(&m1_u8g2, xp, yp, k);
			xp += M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING;
		}
		yp += M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING;
	}

	/* Redraw function key icons */
	u8g2_DrawXBMP(&m1_u8g2, M1_VKB_BACKSPACE_X, M1_VKB_BACKSPACE_Y,
	              M1_VKB_BACKSPACE_ICON_W, M1_VKB_BACKSPACE_ICON_H, m1_virtual_kb_icon_backspace);
	u8g2_DrawXBMP(&m1_u8g2, M1_VKB_ENTER_X, M1_VKB_ENTER_Y,
	              M1_VKB_ENTER_ICON_W, M1_VKB_ENTER_ICON_H, m1_virtual_kb_icon_enter);

	/* Update page indicator */
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
	u8g2_DrawBox(&m1_u8g2, M1_VKB_PAGE_IND_X, M1_VKB_PAGE_IND_Y - M1_VKB_GUI_FONT_HEIGHT,
	             M1_VKB_PAGE_IND_W, M1_VKB_PAGE_IND_H);
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
	u8g2_DrawStr(&m1_u8g2, M1_VKB_PAGE_IND_X, M1_VKB_PAGE_IND_Y, vkb_page_labels[page]);

	/* Recalculate cursor position */
	*out_y = M1_VKB_FIRST_ROW_TOP_POS_Y + M1_VKB_GUI_FONT_HEIGHT
	         + row_id * (M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING);
	*out_x = M1_VKB_LEFT_POS_X + col_id * (M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING);

	/* Highlight current key */
	*out_xkey = m1_vkb_check_function_key(m1_vkb_pages[page][row_id][col_id]);
	if (*out_xkey != M1_VKB_FUNC_UNDEFINED_KEY_ID)
	{
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
		u8g2_DrawBox(&m1_u8g2, m1_x_keys[*out_xkey].icon_x[0], m1_x_keys[*out_xkey].icon_y[0],
		             m1_x_keys[*out_xkey].icon_w, m1_x_keys[*out_xkey].icon_h);
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[*out_xkey].icon_x[0], m1_x_keys[*out_xkey].icon_y[0],
		              m1_x_keys[*out_xkey].icon_w, m1_x_keys[*out_xkey].icon_h,
		              m1_x_keys[*out_xkey].icon_inv);
	}
	else
	{
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_DrawBox(&m1_u8g2, *out_x - 1, *out_y - M1_VKB_GUI_FONT_HEIGHT,
		             M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
		k[0] = m1_vkb_pages[page][row_id][col_id];
		u8g2_DrawStr(&m1_u8g2, *out_x, *out_y, k);
	}

	m1_u8g2_nextpage();
}

/*============================================================================*/
/*
 * This function displays the virtual keyboard for user to enter the filename
 * It returns the filename length and the filename string is given in the pointer new_name
 *
 */
/*============================================================================*/
uint8_t m1_vkb_get_filename(char *description, char *default_name, char *new_name)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t row_id, col_id, x, y;
	uint8_t exit_ok;
	uint8_t len;
	uint8_t vkb_page = M1_VKB_PAGE_LOWER;
	S_M1_VKB_Func_Key_ID x_key_id;
	char key[2], filename[M1_VIRTUAL_KB_FILENAME_MAX + 1];

	memset(filename, 0, sizeof(filename));
	key[1] = 0x00;
    /* Graphic work starts here */
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    m1_u8g2_firstpage(); // This call required for page drawing in mode 1
    do
    {
		u8g2_SetFont(&m1_u8g2, M1_VIRTUAL_KB_FONT_N);
		// Display the description if not null
		if ( strlen(description) )
		{
			u8g2_DrawStr(&m1_u8g2, M1_VKB_DESCRIPTION_POS_X, M1_VKB_DESCRIPTION_POS_Y, description);
		} // if ( len )

		// Draw frame for the filename
		u8g2_DrawFrame(&m1_u8g2, M1_VKB_FILENAME_FRAME_POS_X, M1_VKB_FILENAME_FRAME_POS_Y, M1_VKB_FILENAME_FRAME_WIDTH, M1_VKB_FILENAME_FRAME_HEIGHT);

		// Display the default filename if not null
		len = strlen(default_name);
		if ( len )
		{
			strncpy(filename, default_name, M1_VIRTUAL_KB_FILENAME_MAX);
			if ( len >= M1_VIRTUAL_KB_FILENAME_MAX )
			{
				filename[M1_VIRTUAL_KB_FILENAME_MAX] = 0x00;
				len = M1_VIRTUAL_KB_FILENAME_MAX;
			}
			u8g2_DrawStr(&m1_u8g2, M1_VKB_FILENAME_POS_X, M1_VKB_FILENAME_POS_Y, filename);
		} // if ( len )

		y = M1_VKB_FIRST_ROW_TOP_POS_Y + M1_VKB_GUI_FONT_HEIGHT;
		for (row_id=0; row_id<M1_VIRTUAL_KB_ROW_SIZE; row_id++)
		{
			x = M1_VKB_LEFT_POS_X;
			for (col_id=0; col_id<M1_VIRTUAL_KB_COLUMN_SIZE; col_id++)
			{
				key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
				u8g2_DrawStr(&m1_u8g2, x, y, key);
				x += M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING;
			} // for (col_id=0; col_id<M1_VIRTUAL_KB_COLUMN_SIZE; col_id++)
			y += M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING;
		} // for (row_id=0; row_id<M1_VIRTUAL_KB_ROW_SIZE; row_id++)

		u8g2_DrawXBMP(&m1_u8g2, M1_VKB_BACKSPACE_X, M1_VKB_BACKSPACE_Y, M1_VKB_BACKSPACE_ICON_W, M1_VKB_BACKSPACE_ICON_H, m1_virtual_kb_icon_backspace);
		u8g2_DrawXBMP(&m1_u8g2, M1_VKB_ENTER_X, M1_VKB_ENTER_Y, M1_VKB_ENTER_ICON_W, M1_VKB_ENTER_ICON_H, m1_virtual_kb_icon_enter_inv);

		/* Page indicator (top-right) */
		u8g2_DrawStr(&m1_u8g2, M1_VKB_PAGE_IND_X, M1_VKB_PAGE_IND_Y, vkb_page_labels[vkb_page]);

    } while (m1_u8g2_nextpage());

	col_id = M1_VKB_DEFAULT_KEY_MAP_COL_ID;
	row_id = M1_VKB_DEFAULT_KEY_MAP_ROW_ID;
	exit_ok = 0;

	while (1 ) // Main loop of this task
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				// Notification is only sent to this task when there's any button activity,
				// so it doesn't need to wait when reading the event from the queue
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK ) // User press OK?
				{
					x_key_id = m1_vkb_check_function_key(m1_vkb_pages[vkb_page][row_id][col_id]);

					if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Is a function key hit?
					{
						if ( x_key_id==M1_VKB_FUNC_ENTER_KEY_ID )
						{
							if (len)
							{
								exit_ok = 1;
								if ( new_name != NULL )
								{
									strcpy(new_name, filename);
								}
								else
								{
									len = 0;
								}
							} // if (len)
						} // if ( x_key_id==M1_VKB_FUNC_ENTER_KEY_ID )
						else if ( x_key_id==M1_VKB_FUNC_BS_KEY_ID )
						{
							if (len)
							{
								len--;
								filename[len] = 0x00; // Add NULL to the end of the string
								x = M1_VKB_FILENAME_POS_X;
								x += M1_VKB_GUI_FONT_WIDTH*len;
								y = M1_VKB_FILENAME_POS_Y;
								// Clear this character
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, x, y - M1_VKB_GUI_FONT_HEIGHT + 2, M1_VKB_GUI_FONT_WIDTH, M1_VKB_GUI_FONT_HEIGHT);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								m1_u8g2_nextpage(); // Update graphic to the display RAM
							} // if (len)
						}
					} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
					else // Normal key is hit
					{
						if ( len < M1_VIRTUAL_KB_FILENAME_MAX )
						{
							key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
							filename[len] = key[0];
							len++;
							filename[len] = 0x00; // Add NULL to the end of the string
							x = M1_VKB_FILENAME_POS_X;
							x += M1_VKB_GUI_FONT_WIDTH*(len-1);
							y = M1_VKB_FILENAME_POS_Y;
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
							u8g2_DrawStr(&m1_u8g2, x, y, key); // Display this character
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( len < M1_VIRTUAL_KB_FILENAME_MAX )
					} // else
				}
				else if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // User wants to exit?
				{
					exit_ok = 1;
					len = 0;
				}
				else
				{
					x_key_id = m1_vkb_check_function_key(m1_vkb_pages[vkb_page][row_id][col_id]);

					// Find the (x,y) of the current (col_id, row_id)
					x = M1_VKB_LEFT_POS_X;
					x += (M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING)*col_id;
					y = M1_VKB_FIRST_ROW_TOP_POS_Y + M1_VKB_GUI_FONT_HEIGHT;
					y += (M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING)*row_id;

					if ( this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK ) // User moves to right?
					{
						if ( col_id < (M1_VIRTUAL_KB_COLUMN_SIZE - 1) )
						{
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Current key is a function key?
							{
								// Restore the icon to normal mode
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_reg
												);
								x += m1_x_keys[x_key_id].col_factor*(M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING); // Update x for the next key
								col_id += m1_x_keys[x_key_id].col_factor; // Move to next key
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Restore the current key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set the color to Black
								key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Restore to normal mode
								x += M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING; // Update x for the next key
								col_id++; // Move to next key
							} // else
							x_key_id = m1_vkb_check_function_key(m1_vkb_pages[vkb_page][row_id][col_id]);
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Next key is a function key?
							{
								// Invert the next function key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_inv
												);
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Invert text for next key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( col_id < (M1_VIRTUAL_KB_COLUMN_SIZE - 1) )
					} // if ( this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK )

					else if ( this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK ) // User moves to left?
					{
						if ( col_id > 0 )
						{
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Current key is a function key?
							{
								// Restore the icon to normal mode
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_reg
												);
								x -= M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING; // Update x for the next key
								col_id--; // Move to next key
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Restore the current key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set the color to Black
								key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Restore to normal mode
								x -= M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING; // Update x for the next key
								col_id--; // Move to next key
							} // else
							x_key_id = m1_vkb_check_function_key(m1_vkb_pages[vkb_page][row_id][col_id]);
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Next key is a function key?
							{
								// Invert the next function key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_inv
												);
								col_id++; // Restore
								col_id -= m1_x_keys[x_key_id].col_factor; // Update again for function key
								x += M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING; // Restore
								x -= m1_x_keys[x_key_id].col_factor*(M1_VKB_GUI_FONT_WIDTH + M1_VKB_FONT_WIDTH_SPACING); // Update again for function key

							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Invert text for next key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( col_id > 0 )
					}

					else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK ) // User moves down?
					{
						if ( row_id < (M1_VIRTUAL_KB_ROW_SIZE - 1) )
						{
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Current key is a function key?
							{
								// Restore the icon to normal mode
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_reg
												);
								y += M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING; // Update y for the next key
								row_id++;
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Restore the current key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set the color to Black
								key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Restore to normal mode
								y += M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING; // Update y for the next key
								row_id++;
							} // else
							x_key_id = m1_vkb_check_function_key(m1_vkb_pages[vkb_page][row_id][col_id]);
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Next key is a function key?
							{
								// Invert the next function key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_inv
												);
								col_id = m1_x_keys[x_key_id].key_col_id;
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Invert text for next key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( row_id < (M1_VIRTUAL_KB_ROW_SIZE - 1) )
						else
						{
							/* At bottom row — cycle to next keyboard page */
							vkb_page = (vkb_page + 1) % M1_VKB_PAGE_COUNT;
							vkb_redraw_page(vkb_page, row_id, col_id, &x, &y, &x_key_id);
						}
					} // else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )

					else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK ) // User moves up?
					{
						if ( row_id > 0 )
						{
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Current key is a function key?
							{
								// Restore the icon to normal mode
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_reg
												);
								y -= M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING; // Update y for the next key
								row_id--;
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Restore the current key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set the color to Black
								key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Restore to normal mode
								y -= M1_VKB_GUI_FONT_HEIGHT + M1_VKB_FONT_HEIGHT_SPACING; // Update y for the next key
								row_id--;
							} // else
							x_key_id = m1_vkb_check_function_key(m1_vkb_pages[vkb_page][row_id][col_id]);
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Next key is a function key?
							{
								// Invert the next function key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[0], m1_x_keys[x_key_id].icon_y[0], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_inv
												);
								col_id = m1_x_keys[x_key_id].key_col_id;
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Invert text for next key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								key[0] = m1_vkb_pages[vkb_page][row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( row_id > 0 )
						else
						{
							/* At top row — cycle to previous keyboard page */
							vkb_page = (vkb_page + M1_VKB_PAGE_COUNT - 1) % M1_VKB_PAGE_COUNT;
							vkb_redraw_page(vkb_page, row_id, col_id, &x, &y, &x_key_id);
						}
					} // else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
				} // else

				if ( exit_ok )
				{
					; // Do extra tasks here if needed
					xQueueReset(main_q_hdl); // Reset main q before return
					break; // Exit and return to the calling task (subfunc_handler_task)
				} // if ( m1_buttons_status[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				else
				{
					; // Do other things for this task, if needed
				}
			} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			else
			{
				; // Do other things for this task
			}
		} // if (ret==pdTRUE)
	} // while (1 ) // Main loop of this task

	return len;
}// uint8_t m1_vkb_get_filename(char *description, char *default_name, char *new_name)



/*============================================================================*/
/**
 * @brief: This function checks if a key at (col, row) is a function key or not
 * @param:
 * @retval Function key index if found
 */
/*============================================================================*/
S_M1_VKB_Func_Key_ID m1_vkb_check_function_key(uint8_t kb_key)
{
	uint8_t i;

	for ( i=0; i<M1_VKB_NUM_OF_FUNCTION_KEYS; i++)
	{
		if ( m1_vkb_func_key_codes[i]==kb_key )
			break;
	} // for ( for ( i=0; i<M1_VKB_NUM_OF_FUNCTION_KEYS; i++)

	return i;
} // S_M1_VKB_Func_Key_ID m1_vkb_check_function_key(uint8_t kb_key)




/*============================================================================*/
/**
 * @brief: This function displays the virtual hex keyboard for user to edit data
 * @param: None
 * @retval 0 if user cancels, otherwise data length and data buffer
 */
/*============================================================================*/
uint8_t m1_vkbs_get_data(char *description, char *data_buffer)
{
	S_M1_Buttons_Status this_button_status;
	S_M1_Main_Q_t q_item;
	BaseType_t ret;
	uint8_t row_id, col_id, data_id, buffer_id;
	uint8_t exit_ok, len, x, y;
	S_M1_VKB_Func_Key_ID x_key_id;
	char key[2];

	if ( data_buffer==NULL )
		return 0;

	//strcpy(data_buffer, "00 00 00 00 00"); // Default data
	key[1] = '\0'; // NULL at the end of a string
	exit_ok = 0;
	M1_VIRTUAL_KBS_DATA_MAX = strlen(data_buffer);
    /* Graphic work starts here */
	u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
    m1_u8g2_firstpage(); // This call required for page drawing in mode 1
    do
    {
		u8g2_SetFont(&m1_u8g2, M1_VIRTUAL_KB_FONT_N);
		// Display the description if not null
		len = strlen(description);
		if ( len )
		{
			u8g2_DrawStr(&m1_u8g2, M1_VKBS_DESCRIPTION_POS_X, M1_VKBS_DESCRIPTION_POS_Y, description);
		} // if ( len )

		// Draw frame for the data
		u8g2_DrawFrame(&m1_u8g2, M1_VKBS_DATA_FRAME_POS_X, M1_VKBS_DATA_FRAME_POS_Y, M1_VKBS_DATA_FRAME_WIDTH, M1_VKBS_DATA_FRAME_HEIGHT);

		// Display the default data
		u8g2_DrawStr(&m1_u8g2, M1_VKBS_DATA_POS_X, M1_VKBS_DATA_POS_Y, data_buffer);

		y = M1_VKBS_FIRST_ROW_TOP_POS_Y + M1_VKB_GUI_FONT_HEIGHT;
		for (row_id=0; row_id<M1_VIRTUAL_KBS_ROW_SIZE; row_id++)
		{
			x = M1_VKBS_LEFT_POS_X;
			for (col_id=0; col_id<M1_VIRTUAL_KBS_COLUMN_SIZE; col_id++)
			{
				key[0] = m1_vkbs_map[row_id][col_id];
				u8g2_DrawStr(&m1_u8g2, x, y, key);
				x += M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING;
			} // for (col_id=0; col_id<M1_VIRTUAL_KBS_COLUMN_SIZE; col_id++)
			y += M1_VKB_GUI_FONT_HEIGHT + M1_VKBS_FONT_HEIGHT_SPACING;
		} // for (row_id=0; row_id<M1_VIRTUAL_KB_ROW_SIZE; row_id++)

		u8g2_DrawXBMP(&m1_u8g2, M1_VKBS_BACKSPACE_X, M1_VKBS_BACKSPACE_Y, M1_VKBS_BACKSPACE_ICON_W, M1_VKBS_BACKSPACE_ICON_H, m1_virtual_kb_icon_backspace);
		u8g2_DrawXBMP(&m1_u8g2, M1_VKBS_ENTER_X, M1_VKBS_ENTER_Y, M1_VKBS_ENTER_ICON_W, M1_VKBS_ENTER_ICON_H, m1_virtual_kb_icon_enter);

		col_id = 0;
		row_id = 0;
		data_id = 0;
		buffer_id = 0;

		// Find the (x,y) of the current (col_id, row_id)
		x = M1_VKBS_LEFT_POS_X;
		x += (M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING)*col_id;
		y = M1_VKBS_FIRST_ROW_TOP_POS_Y + M1_VKB_GUI_FONT_HEIGHT;
		y += (M1_VKB_GUI_FONT_HEIGHT + M1_VKBS_FONT_HEIGHT_SPACING)*row_id;
		// Invert text for active key
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
		key[0] = m1_vkbs_map[row_id][col_id];
		u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode

		// Find the (x,y) of the current [data_id]
		x = M1_VKBS_DATA_POS_X;
		y = M1_VKBS_DATA_POS_Y;
		// Invert text for active [data_id]
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
		u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT); // Invert background
		u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
		key[0] = data_buffer[buffer_id];
		u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
    } while (m1_u8g2_nextpage());

	while (1 ) // Main loop of this task
	{
		ret = xQueueReceive(main_q_hdl, &q_item, portMAX_DELAY);
		if (ret==pdTRUE)
		{
			if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			{
				// Notification is only sent to this task when there's any button activity,
				// so it doesn't need to wait when reading the event from the queue
				ret = xQueueReceive(button_events_q_hdl, &this_button_status, 0);
				if ( this_button_status.event[BUTTON_OK_KP_ID]==BUTTON_EVENT_CLICK ) // User press OK?
				{
					x_key_id = m1_vkb_check_function_key(m1_vkbs_map[row_id][col_id]);
					if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Is a function key hit?
					{
						if ( x_key_id==M1_VKB_FUNC_ENTER_KEY_ID )
						{
							exit_ok = 1;
						} // if ( x_key_id==M1_VKB_FUNC_ENTER_KEY_ID )
						else if ( x_key_id==M1_VKB_FUNC_BS_KEY_ID )
						{
							data_buffer[buffer_id] = '0'; // Clear this digit
							x = M1_VKBS_DATA_POS_X;
							x += M1_VKB_GUI_FONT_WIDTH*buffer_id;
							y = M1_VKBS_DATA_POS_Y;
							key[0] = data_buffer[buffer_id];
							if (buffer_id)
							{
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 1, M1_VKB_GUI_FONT_HEIGHT); // Clear
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // Restore the text
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Draw text in normal mode
								data_id--;
								buffer_id--;
								if ( data_buffer[buffer_id]==' ' )
									buffer_id--;
								x = M1_VKBS_DATA_POS_X;
								x += M1_VKB_GUI_FONT_WIDTH*buffer_id; // Update new pos x
								key[0] = data_buffer[buffer_id];
								// Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 1, M1_VKB_GUI_FONT_HEIGHT); // Inverted
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // if (buffer_id)
							else // First data id
							{
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 1, M1_VKB_GUI_FONT_HEIGHT); // Inverted
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // else if ( x_key_id==M1_VKB_FUNC_BS_KEY_ID )
					} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
					else // Normal key is hit
					{
						key[0] = m1_vkbs_map[row_id][col_id]; // Get key from keyboard
						data_buffer[buffer_id] = key[0]; // Update new key
						x = M1_VKBS_DATA_POS_X;
						x += M1_VKB_GUI_FONT_WIDTH*buffer_id;
						y = M1_VKBS_DATA_POS_Y;
						if ( buffer_id < (M1_VIRTUAL_KBS_DATA_MAX-1) )
						{
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
							u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 1, M1_VKB_GUI_FONT_HEIGHT); // Clear
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
							u8g2_DrawStr(&m1_u8g2, x, y, key); // Display this character
							buffer_id++;
							if ( data_buffer[buffer_id]==' ' )
								buffer_id++;
							x = M1_VKBS_DATA_POS_X;
							x += M1_VKB_GUI_FONT_WIDTH*buffer_id; // Update new pos x
							key[0] = data_buffer[buffer_id]; // Update next key from buffer
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
							u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 1, M1_VKB_GUI_FONT_HEIGHT); // Inverted
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);	// Inverted text
							u8g2_DrawStr(&m1_u8g2, x, y, key); // Display next character
						} // if ( buffer_id < (M1_VIRTUAL_KBS_DATA_MAX-1) )
						else // Last buffer id
						{
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
							u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 1, M1_VKB_GUI_FONT_HEIGHT); // Inverted
							u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);	// Inverted text
							u8g2_DrawStr(&m1_u8g2, x, y, key); // Display this character
						} // else
						m1_u8g2_nextpage(); // Update graphic to the display RAM
					} // else
				}
				else if ( this_button_status.event[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK ) // User wants to exit?
				{
					exit_ok = 1;
					len = 0;
					xQueueReset(main_q_hdl); // Reset main q before return
					break;
				}
				else
				{
					x_key_id = m1_vkb_check_function_key(m1_vkbs_map[row_id][col_id]);
					// Find the (x,y) of the current (col_id, row_id)
					x = M1_VKBS_LEFT_POS_X;
					x += (M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING)*col_id;
					y = M1_VKBS_FIRST_ROW_TOP_POS_Y + M1_VKB_GUI_FONT_HEIGHT;
					y += (M1_VKB_GUI_FONT_HEIGHT + M1_VKBS_FONT_HEIGHT_SPACING)*row_id;

					if ( this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK ) // User moves to right?
					{
						if ( col_id < (M1_VIRTUAL_KBS_COLUMN_SIZE - 1) )
						{
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Current key is a function key?
							{
								// Restore the icon to normal mode
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_reg
												);
								x += m1_x_keys[x_key_id].col_factor*(M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING); // Update x for the next key
								//col_id += m1_x_keys[x_key_id].col_factor; // Move to next key
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Restore the current key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set the color to Black
								key[0] = m1_vkbs_map[row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Restore to normal mode
								x += M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING; // Update x for the next key
								col_id++; // Move to next key
							} // else
							x_key_id = m1_vkb_check_function_key(m1_vkbs_map[row_id][col_id]);
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Next key is a function key?
							{
								// Invert the next function key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_inv
												);
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Invert text for next key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								key[0] = m1_vkbs_map[row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( col_id < (M1_VIRTUAL_KBS_COLUMN_SIZE - 1) )
					} // if ( this_button_status.event[BUTTON_RIGHT_KP_ID]==BUTTON_EVENT_CLICK )

					else if ( this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK ) // User moves to left?
					{
						if ( col_id > 0 )
						{
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Current key is a function key?
							{
								// Restore the icon to normal mode
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_reg
												);
								x -= M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING; // Update x for the next key
								col_id--; // Move to next key
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Restore the current key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set the color to Black
								key[0] = m1_vkbs_map[row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Restore to normal mode
								x -= M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING; // Update x for the next key
								col_id--; // Move to next key
							} // else
							x_key_id = m1_vkb_check_function_key(m1_vkbs_map[row_id][col_id]);
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Next key is a function key?
							{
								// Invert the next function key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_inv
												);
								col_id++; // Restore
								//col_id -= m1_x_keys[x_key_id].col_factor; // Update again for function key
								x += M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING; // Restore
								//x -= m1_x_keys[x_key_id].col_factor*(M1_VKB_GUI_FONT_WIDTH + M1_VKBS_FONT_WIDTH_SPACING); // Update again for function key

							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Invert text for next key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								key[0] = m1_vkbs_map[row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( col_id > 0 )
					} // else if ( this_button_status.event[BUTTON_LEFT_KP_ID]==BUTTON_EVENT_CLICK )

					else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK ) // User moves down?
					{
						if ( row_id < (M1_VIRTUAL_KBS_ROW_SIZE - 1) )
						{
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Current key is a function key?
							{
								// Restore the icon to normal mode
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_reg
												);
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Restore the current key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set the color to Black
								key[0] = m1_vkbs_map[row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Restore to normal mode
							} // else

							y += M1_VKB_GUI_FONT_HEIGHT + M1_VKBS_FONT_HEIGHT_SPACING; // Update y for the next key
							row_id++;
							x_key_id = m1_vkb_check_function_key(m1_vkbs_map[row_id][col_id]);
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Next key is a function key?
							{
								// Invert the next function key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_inv
												);
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Invert text for next key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								key[0] = m1_vkbs_map[row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( row_id < (M1_VIRTUAL_KBS_ROW_SIZE - 1) )
					} // else if ( this_button_status.event[BUTTON_DOWN_KP_ID]==BUTTON_EVENT_CLICK )

					else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK ) // User moves up?
					{
						if ( row_id > 0 )
						{
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Current key is a function key?
							{
								// Restore the icon to normal mode
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_reg
												);
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Restore the current key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG); // set the color to White
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT); // set the color to Black
								key[0] = m1_vkbs_map[row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key); // Restore to normal mode
							} // else

							y -= M1_VKB_GUI_FONT_HEIGHT + M1_VKBS_FONT_HEIGHT_SPACING; // Update y for the next key
							row_id--;
							x_key_id = m1_vkb_check_function_key(m1_vkbs_map[row_id][col_id]);
							if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID ) // Next key is a function key?
							{
								// Invert the next function key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								u8g2_DrawBox(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w, m1_x_keys[x_key_id].icon_h);
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawXBMP(&m1_u8g2, m1_x_keys[x_key_id].icon_x[1], m1_x_keys[x_key_id].icon_y[1], m1_x_keys[x_key_id].icon_w,
												m1_x_keys[x_key_id].icon_h, m1_x_keys[x_key_id].icon_inv
												);
							} // if ( x_key_id != M1_VKB_FUNC_UNDEFINED_KEY_ID )
							else
							{
								// Invert text for next key
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_TXT);
								u8g2_DrawBox(&m1_u8g2, x - 1, y - M1_VKB_GUI_FONT_HEIGHT + 1, M1_VKB_GUI_FONT_WIDTH + 2, M1_VKB_GUI_FONT_HEIGHT + 2); // Invert background
								u8g2_SetDrawColor(&m1_u8g2, M1_DISP_DRAW_COLOR_BG);
								key[0] = m1_vkbs_map[row_id][col_id];
								u8g2_DrawStr(&m1_u8g2, x, y, key);// Draw text in inverted mode
							} // else
							m1_u8g2_nextpage(); // Update graphic to the display RAM
						} // if ( row_id > 0 )
					} // else if ( this_button_status.event[BUTTON_UP_KP_ID]==BUTTON_EVENT_CLICK )
				} // else

				if ( exit_ok )
				{
					; // Do extra tasks here if needed
					xQueueReset(main_q_hdl); // Reset main q before return
					len = M1_VIRTUAL_KBS_DATA_MAX;
					break; // Exit and return to the calling task (subfunc_handler_task)
				} // if ( m1_buttons_status[BUTTON_BACK_KP_ID]==BUTTON_EVENT_CLICK )
				else
				{
					; // Do other things for this task, if needed
				}
			} // if ( q_item.q_evt_type==Q_EVENT_KEYPAD )
			else
			{
				; // Do other things for this task
			}
		} // if (ret==pdTRUE)
	} // while (1 ) // Main loop of this task

	return len;
}// uint8_t m1_vkbs_get_data(char *description, char *data_buffer)

