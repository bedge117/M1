/* See COPYING.txt for license details. */

/*
*
*  m1_nfc.h
*
*  M1 NFC functions
*
* M1 Project
*
*/

#ifndef M1_NFC_H_
#define M1_NFC_H_

void nfc_read(void);
void nfc_detect_reader(void);
void nfc_saved(void);
void nfc_extra_actions(void);
void nfc_add_manually(void);
void nfc_tools(void);

void menu_nfc_init(void);
void menu_nfc_deinit(void);

#endif /* M1_NFC_H_ */
