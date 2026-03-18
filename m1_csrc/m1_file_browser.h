/* See COPYING.txt for license details. */

/*
*
* m1_file_browser.h
*
* Library for sd card file browsing
*
 * M1 Project
*
*/

#ifndef M1_FILE_BROWSER_H_
#define M1_FILE_BROWSER_H_

#include "u8g2.h"
#include "mui.h"
#include "ff.h"
#include "ff_gen_drv.h"

typedef enum
{
	F_EXT_DATA = 0,
	F_EXT_OTHER
} S_M1_file_browser_ext;

typedef enum
{
	FB_OK = 0,
	FB_ERR_SDCARD,
	FB_ERR_GUI
} S_M1_file_browser_code;

typedef struct
{
	char *dir_name;
	char *file_name;
	bool file_is_selected;
	S_M1_file_browser_code status;
} S_M1_file_info;

typedef struct
{
	S_M1_file_info info;
	uint16_t x, y, gui_width, gui_height;
	uint8_t font_w, font_h;
	uint8_t font_h_spacing;
	uint16_t *listing_index_buffer, *row_index_buffer;
	uint16_t listing_index, row_index;
	uint8_t dir_level;
} S_M1_file_browser_hdl;

S_M1_file_browser_hdl *m1_fb_init(u8g2_t *lcd_hdl);
void m1_fb_set_dir(const char *dir);
void m1_fb_deinit(void);
void m1_fb_popup(void);
S_M1_file_info *m1_fb_display(S_M1_Buttons_Status *button_status);
FRESULT m1_fb_listing(const char *dir_name);
uint8_t m1_fb_dyn_strcat(char *buffer, uint8_t num, const char *format, ...);
uint8_t m1_fb_open_new_file(FIL *file, const char *filename);
uint8_t m1_fb_open_file(FIL *file, const char *filename);
uint8_t m1_fb_open_dir(DIR *dir, const char *directory);
uint8_t m1_fb_make_dir(const char *directory);
uint8_t m1_fb_check_existence(const char *filedir);
uint8_t m1_fb_close_file(FIL *file);
uint8_t m1_fb_open_log_file(const char *filename);
uint8_t m1_fb_close_log_file(void);
uint8_t m1_fb_delete_file(const char *filename);
uint16_t m1_fb_write_to_file(FIL *pfile, const char *buffer, uint16_t size);
uint16_t m1_fb_read_from_file(FIL *pfile, char *buffer, uint16_t size);

#endif /* M1_FILE_BROWSER_H_ */
