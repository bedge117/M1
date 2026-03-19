/* See COPYING.txt for license details. */

/*
*
* m1_bt.h
*
* Header for M1 bluetooth
*
* M1 Project
*
*/

#ifndef M1_BT_H_
#define M1_BT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "m1_compile_cfg.h"
#include "ctrl_api.h"


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,
    IDX_CHAR_CFG_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,
    IDX_CHAR_CFG_C,
    IDX_CHAR_CFG_C_2,

    HRS_IDX_NB,
};

void menu_bluetooth_init(void);
void bluetooth_config(void);
void bluetooth_scan(void);
void bluetooth_advertise(void);

#ifdef M1_APP_BT_MANAGE_ENABLE

typedef struct {
    bool connected;
    char addr[BSSID_STR_SIZE];
    char name[SSID_LENGTH];
} bt_connection_state_t;

void bluetooth_saved_devices(void);
void bluetooth_info(void);
bt_connection_state_t *bt_get_connection_state(void);

#endif /* M1_APP_BT_MANAGE_ENABLE */

#ifdef M1_APP_BADBT_ENABLE
void bluetooth_set_badbt_name(void);
#endif

#endif /* M1_BT_H_ */
