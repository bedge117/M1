/* See COPYING.txt for license details. */

/*
*
* m1_at_response_parser.h
*
* M1 parser for EPS32 module
*
* M1 Project
*
*/

#ifndef M1_AT_RESPONSE_PARSER_H_
#define M1_AT_RESPONSE_PARSER_H_

#include "m1_compile_cfg.h"

char *m1_resp_string_strip(char *resp, const char *substr);
uint8_t m1_parse_spi_at_resp(char *resp, const char *resp_key, ctrl_cmd_t *app_resp);

#ifdef M1_APP_BT_MANAGE_ENABLE
uint8_t m1_parse_ble_scan_resp(char *resp, const char *resp_key, ctrl_cmd_t *app_resp);
#endif

#endif /* M1_AT_RESPONSE_PARSER_H_ */
