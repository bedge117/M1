/* See COPYING.txt for license details. */

/*
*
* m1_watchdog.h
*
* Watchdog functions
*
* M1 Project
*
*/

#ifndef M1_WATCHDOG_H_
#define M1_WATCHDOG_H_

#include "FreeRTOS.h"   /* TickType_t used in the report API below */
#include "task.h"

typedef enum
{
	M1_REPORT_ID_BUTTONS_HANDLER_TASK = 0,
	/* Supervised background tasks. Only tasks that (a) loop on a bounded cadence and
	 * (b) have NO multi-second blocking operation may be added — otherwise a
	 * legitimate long op would be misread as a hang and reset the device (see the
	 * resume_grace note in m1_wdt_resume_task). */
	M1_REPORT_ID_USB2SER_HANDLER_TASK,   /* USB RX -> RPC; the lost-wakeup task, now bounded 100ms */
	M1_REPORT_ID_QMON_RELAY_TASK,        /* ESP background poll relay (50/250ms, bounded esp_calls) */
	/* rpc_task IS supervised now, but it OWNS the ESP/SD flash (legitimately
	 * blocks for seconds), so it suspends its own report around each flash op
	 * (m1_wdt_suspend/resume_task) and heartbeats every loop otherwise. Every
	 * other deferred op is bounded (<=5s SD software timeout), inside the window. */
	M1_REPORT_ID_RPC_TASK,
	M1_REPORT_ID_END_OF_LIST
} S_M1_WDT_Report_ID;

typedef struct
{
	S_M1_WDT_Report_ID report_id;
	bool inactive;
	bool resume_grace;   /* one free system-check pass right after resume (see m1_wdt_system_check) */
	uint32_t report_period;
	uint32_t min_rpt_percent, max_rpt_percent;
	uint32_t run_time;
} S_M1_WDT_Report;

void m1_wdt_init(void);
void m1_wdt_report_init(void);
void m1_wdt_send_report(S_M1_WDT_Report_ID rpt_id, uint32_t time);
void m1_wdt_send_report_ex(S_M1_WDT_Report_ID rpt_id, TickType_t start_time);
void m1_wdt_send_delayed_report(S_M1_WDT_Report_ID rpt_id, uint32_t delay_ms, uint8_t repeat);
void m1_wdt_reset(void);
void m1_wdt_suspend_task(S_M1_WDT_Report_ID rpt_id);
void m1_wdt_resume_task(S_M1_WDT_Report_ID rpt_id);

#endif /* M1_WATCHDOG_H_ */
