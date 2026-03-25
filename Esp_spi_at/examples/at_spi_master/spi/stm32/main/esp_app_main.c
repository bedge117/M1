/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stream_buffer.h"

//#include "esp_system.h"
#include "esp_err.h"
#include "m1_log_debug.h"
#include "spi_master.h"
#include "m1_tasks.h"
#include "m1_compile_cfg.h"
#include "m1_esp32_hal.h"
#include "m1_io_defs.h"
#include "esp_at_list.h"
#include "esp_queue.h"
#include "m1_at_response_parser.h"

#define STREAM_BUFFER_SIZE    	SPI_TRANS_MAX_LEN

#define TAG						"SPI_AT_Master"
#define ESP_SPI_ID				0x00

#define CR_LF					"\r\n"

#define MILLISEC_TO_SEC			1000
#define TICKS_PER_SEC (1000 / portTICK_PERIOD_MS);
#define SEC_TO_MILLISEC(x) (1000*(x))

/* If request is already being served and
 * another request is pending, time period for
 * which new request will wait in seconds
 * */
#define WAIT_TIME_B2B_CTRL_REQ               5
#define DEFAULT_CTRL_RESP_TIMEOUT            30
#define DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT    (60*3)
#define DEFAULT_CTRL_RESP_CONNECT_AP_TIMEOUT (15*3)

QueueHandle_t esp_spi_msg_queue; // message queue used for communicating read/write start
QueueHandle_t esp_resp_read_sem = NULL;
QueueHandle_t esp_ctrl_req_sem = NULL;
esp_queue_t* ctrl_msg_Q = NULL;

static spi_device_handle_t spi_dev_handle = NULL;
static StreamBufferHandle_t spi_master_tx_ring_buf = NULL;
static SemaphoreHandle_t pxMutex;
static uint8_t initiative_send_flag = 0; // it means master has data to send to slave
static uint32_t plan_send_len = 0; // master plan to send data len

static uint8_t current_send_seq = 0;
static uint8_t current_recv_seq = 0;

static bool esp32_main_init_done = false;

/* uid to link between requests and responses
 * uids are incrementing values from 1 onwards. */
static int32_t uid = 0;
/* uid of request / response */
static int32_t current_uid = 0;

static void spi_mutex_lock(void)
{
    while (xSemaphoreTake(pxMutex, portMAX_DELAY) != pdPASS);
}

static void spi_mutex_unlock(void)
{
    xSemaphoreGive(pxMutex);
}

static void at_spi_master_send_data(uint8_t* data, uint32_t len)
{
	HAL_StatusTypeDef ret;

	spi_transaction_t trans = {
        .cmd = CMD_HD_WRDMA_REG,    // master -> slave command, donnot change
        .length = len * 8,
        .tx_buffer = (void*)data
    };
	ret = spi_device_polling_transmit(spi_dev_handle, &trans);
	current_uid = uid; // Update current request command id after it has been transmitted
}

static void at_spi_master_recv_data(uint8_t* data, uint32_t len)
{
	spi_transaction_t trans = {
        .cmd = CMD_HD_RDDMA_REG,    // master -> slave command, donnot change
        .rxlength = len * 8,
        .rx_buffer = (void*)data
    };
	spi_device_polling_transmit(spi_dev_handle, &trans);
}

// send a single to slave to tell slave that master has read DMA done
static void at_spi_rddma_done(void)
{
    spi_transaction_t end_t = {
        .cmd = CMD_HD_INT0_REG,
    };
    spi_device_polling_transmit(spi_dev_handle, &end_t);
}

// send a single to slave to tell slave that master has write DMA done
static void at_spi_wrdma_done(void)
{
    spi_transaction_t end_t = {
        .cmd = CMD_HD_WR_END_REG,
    };
    spi_device_polling_transmit(spi_dev_handle, &end_t);
}

// when spi slave ready to send/recv data from the spi master, the spi slave will a trigger GPIO interrupt,
// then spi master should query whether the slave will perform read or write operation.
static spi_recv_opt_t query_slave_data_trans_info()
{
    spi_recv_opt_t recv_opt = {};
    spi_transaction_t trans = {
        .cmd = CMD_HD_RDBUF_REG,
        .addr = RDBUF_START_ADDR,
        .rxlength = 4 * 8,
        .rx_buffer = &recv_opt,
    };
    spi_device_polling_transmit(spi_dev_handle, (spi_transaction_t*)&trans);

    return recv_opt;
}

// before spi master write to slave, the master should write WRBUF_REG register to notify slave,
// and then wait for handshake line trigger gpio interrupt to start the data transmission.
static void spi_master_request_to_write(uint8_t send_seq, uint16_t send_len)
{
    spi_send_opt_t send_opt;
    send_opt.magic = 0xFE;
    send_opt.send_seq = send_seq;
    send_opt.send_len = send_len;

    spi_transaction_t trans = {
        .cmd = CMD_HD_WRBUF_REG,
        .addr = WRBUF_START_ADDR,
        .length = 4 * 8,
        .tx_buffer = &send_opt,
    };
    spi_device_polling_transmit(spi_dev_handle, (spi_transaction_t*)&trans);
    // increment
    current_send_seq  = send_seq;
}

// spi master write data to slave
static int8_t spi_write_data(uint8_t* buf, int32_t len)
{
    if (len > SPI_TRANS_MAX_LEN) {
        M1_LOG_E(TAG, "Send length error, len:%ld\r\n", len);
        return -1;
    }
    at_spi_master_send_data(buf, len);
    at_spi_wrdma_done();
    return 0;
}

// write data to spi tx_ring_buf, this is just for test
static int32_t write_data_to_spi_task_tx_ring_buf(const void* data, size_t size)
{
    int32_t length = size;

    if (data == NULL  || length > STREAM_BUFFER_SIZE) {
        M1_LOG_E(TAG, "Write data error, len:%ld\r\n", length);
        return -1;
    }

    length = xStreamBufferSend(spi_master_tx_ring_buf, data, size, portMAX_DELAY);
    return length;
}


// notify slave to recv data
static void notify_slave_to_recv(void)
{
    if (initiative_send_flag == 0)
    {
        spi_mutex_lock();
        uint32_t tmp_send_len = xStreamBufferBytesAvailable(spi_master_tx_ring_buf);
        if (tmp_send_len > 0)
        {
            plan_send_len = tmp_send_len > SPI_TRANS_MAX_LEN ? SPI_TRANS_MAX_LEN : tmp_send_len;
            spi_master_request_to_write(current_send_seq + 1, plan_send_len); // to tell slave that the master want to write data
            initiative_send_flag = 1;
        }
        spi_mutex_unlock();
    }
}


static void spi_trans_control_task(void* arg)
{
    esp_err_t ret;
    spi_master_msg_t trans_msg = {0};
    uint32_t send_len = 0;
    esp_queue_elem_t *elem = NULL;
    char *app_resp = NULL;

    uint8_t *trans_data = (uint8_t*)malloc(SPI_TRANS_MAX_LEN * sizeof(uint8_t));
    if (trans_data == NULL)
    {
        M1_LOG_E(TAG, "malloc fail\r\n");
        return;
    }

    while (1)
    {
        xQueueReceive(esp_spi_msg_queue, (void*)&trans_msg, (TickType_t)portMAX_DELAY);
        spi_mutex_lock();
        spi_recv_opt_t recv_opt = query_slave_data_trans_info();

        if (recv_opt.direct == SPI_WRITE)
        {
            if (plan_send_len == 0) {
                M1_LOG_E(TAG, "master want send data but length is 0\r\n");
                spi_mutex_unlock();
                continue;
            }

            if (recv_opt.seq_num != current_send_seq) {
                M1_LOG_E(TAG, "SPI send seq error, %x, %x\r\n", recv_opt.seq_num, current_send_seq);
                if (recv_opt.seq_num == 1) {
                    M1_LOG_E(TAG, "Maybe SLAVE restart, ignore\r\n");
                }
                current_send_seq = recv_opt.seq_num;
            }

            //initiative_send_flag = 0;
            send_len = xStreamBufferReceive(spi_master_tx_ring_buf, (void*) trans_data, plan_send_len, 0);

            if (send_len != plan_send_len) {
                M1_LOG_E(TAG, "Read len expect %lu, but actual read %lu\r\n", plan_send_len, send_len);
                initiative_send_flag = 0;
                spi_mutex_unlock();
                continue;
            }

            ret = spi_write_data(trans_data, plan_send_len);
            if (ret < 0) {
                M1_LOG_E(TAG, "Load data error\r\n");
                initiative_send_flag = 0;
                spi_mutex_unlock();
                continue;
            }

            // maybe streambuffer filled some data when SPI transmit, just consider it after send done, because send flag has already in SLAVE queue
            uint32_t tmp_send_len = xStreamBufferBytesAvailable(spi_master_tx_ring_buf);
            if (tmp_send_len > 0) {
                plan_send_len = tmp_send_len > SPI_TRANS_MAX_LEN ? SPI_TRANS_MAX_LEN : tmp_send_len;
                spi_master_request_to_write(current_send_seq + 1, plan_send_len);
            } else {
                initiative_send_flag = 0;
            }

        } // if (recv_opt.direct == SPI_WRITE)
        else if (recv_opt.direct == SPI_READ)
        {
            if (recv_opt.seq_num != ((current_recv_seq + 1) & 0xFF)) {
                M1_LOG_E(TAG, "SPI recv seq error, %x, %x\r\n", recv_opt.seq_num, (current_recv_seq + 1));
                if (recv_opt.seq_num == 1) {
                    M1_LOG_E(TAG, "Maybe SLAVE restart, ignore\r\n");
                }
                current_recv_seq = recv_opt.seq_num;
            }

            if (recv_opt.transmit_len > STREAM_BUFFER_SIZE || recv_opt.transmit_len == 0) {
                M1_LOG_E(TAG, "SPI read len error, %x\r\n", recv_opt.transmit_len);
                at_spi_rddma_done();
                spi_mutex_unlock();
                continue;
            }

            current_recv_seq = recv_opt.seq_num;
            memset(trans_data, 0x0, recv_opt.transmit_len);
            at_spi_master_recv_data(trans_data, recv_opt.transmit_len);
            at_spi_rddma_done();
            trans_data[recv_opt.transmit_len] = '\0';
#ifdef M1_APP_ESP_RESPONSE_PRINT_ENABLE
            printf("%s", trans_data);
            fflush(stdout);    //Force to print even if have not '\n'
#endif // #ifdef M1_APP_ESP_RESPONSE_PRINT_ENABLE
    		/* Allocate app struct for response */
    		app_resp = (uint8_t *)malloc(recv_opt.transmit_len + 1);
    		if (!app_resp)
    		{
    			M1_LOG_E(TAG, "Failed to allocate app_resp %d\r\n", recv_opt.transmit_len + 1);
    			spi_mutex_unlock();
    			continue;
    		}
    		strcpy(app_resp, trans_data);

    		xSemaphoreGive(esp_ctrl_req_sem);

    		elem = (esp_queue_elem_t*)malloc(sizeof(esp_queue_elem_t));
			if (!elem)
			{
				M1_LOG_E(TAG, "%s %u: Malloc failed\n",__func__,__LINE__);
				free(app_resp);
				spi_mutex_unlock();
				continue;
			}
			elem->buf = app_resp;
			elem->buf_len = recv_opt.transmit_len;
			elem->uid = current_uid;
			if ( esp_queue_put(ctrl_msg_Q, (void*)elem) )
			{
				M1_LOG_E(TAG, "%s %u: ctrl Q put fail\r\n",__func__,__LINE__);
				free(app_resp);
				free(elem);
				spi_mutex_unlock();
				continue;
			} // if ( esp_queue_put(ctrl_msg_Q, (void*)elem) )

			xSemaphoreGive(esp_resp_read_sem);
        } // else if (recv_opt.direct == SPI_READ)
        else
        {
            M1_LOG_D(TAG, "Unknown direct: %d", recv_opt.direct);
            spi_mutex_unlock();
            continue;
        }

        spi_mutex_unlock();
    } // while (1)

    free(trans_data);
    vTaskDelete(NULL);
}


uint8_t spi_AT_app_send_command(ctrl_cmd_t *app_req)
{
	int ret = SUCCESS;

	if (!app_req)
	{
		return CTRL_ERR_INCORRECT_ARG;
	}

	/* 1. Check if any ongoing request present
	 * Send failure in that case */
	ret = xSemaphoreTake(esp_ctrl_req_sem, SEC_TO_MILLISEC(WAIT_TIME_B2B_CTRL_REQ));
	if (ret!=pdPASS)
	{
		return CTRL_ERR_REQ_IN_PROG;
	}
	app_req->msg_type = CTRL_REQ;
	// handle rollover in uid value (range: 1 to INT32_MAX)
	if (uid < INT32_MAX)
		uid++;
	else
		uid = 1;
	app_req->uid = uid;

    write_data_to_spi_task_tx_ring_buf(app_req->at_cmd, app_req->cmd_len);
    notify_slave_to_recv();

    return SUCCESS;
} // uint8_t spi_AT_app_send_command(ctrl_cmd_t *app_req)


static uint8_t *spi_AT_app_get_response(int *read_len, uint32_t *uid, int timeout_sec)
{
	void *data = NULL;
	uint8_t *buf = NULL;
	esp_queue_elem_t *elem = NULL;
	int ret = 0;

	/* 1. Any problems in response, return NULL */
	if (!read_len)
	{
		M1_LOG_E(TAG, "Invalid input parameter\r\n");
		return NULL;
	}

	/* 2. If timeout not specified, use default */
	if (!timeout_sec)
		timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT;

	/* 3. Wait for response */
	ret = xSemaphoreTake(esp_resp_read_sem, SEC_TO_MILLISEC(timeout_sec));
	if (ret!=pdPASS)
	{
		M1_LOG_E(TAG, "ESP response timed out after %u sec\r\n", timeout_sec);
		xSemaphoreGive(esp_ctrl_req_sem);
		return NULL;
	}

	/* 4. Fetch response from `esp_queue` */
	data = esp_queue_get(ctrl_msg_Q);
	if (data)
	{
		elem = (esp_queue_elem_t *)data;
		if (!elem)
			return NULL;

		*read_len = elem->buf_len;
		*uid = elem->uid;
		buf = elem->buf;
		free(elem);
		if ( esp_queue_check(ctrl_msg_Q) ) // There's still data in the queue?
			xSemaphoreGive(esp_resp_read_sem); // Give the app the chance to read again
		return buf;
	}
	else
	{
		M1_LOG_E(TAG, "Ctrl Q empty or uninitialized\r\n");
		return NULL;
	}

	return NULL;
} // static uint8_t *spi_AT_app_get_response(int *read_len, uint32_t *uid, int timeout_sec)


uint8_t spi_AT_send_recv(const char *at_cmd, char *out_buf, int out_buf_size, int timeout_sec)
{
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	uint8_t ret;
	int rx_len = 0;
	uint32_t rx_uid = 0;
	uint8_t *rx_buf = NULL;
	int total_len = 0;

	if (!at_cmd || !out_buf || out_buf_size < 2)
		return CTRL_ERR_INCORRECT_ARG;

	out_buf[0] = '\0';

	req.at_cmd = (char *)at_cmd;
	req.cmd_len = strlen(at_cmd);
	if (!timeout_sec)
		timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT;
	req.cmd_timeout_sec = timeout_sec;

	ret = spi_AT_app_send_command(&req);
	if (ret != SUCCESS)
	{
		snprintf(out_buf, out_buf_size, "SEND_ERR=%d", ret);
		return ret;
	}

	/* Collect responses until OK/ERROR/timeout (up to buffer) */
	while (total_len < out_buf_size - 1)
	{
		rx_buf = spi_AT_app_get_response(&rx_len, &rx_uid, timeout_sec);
		if (!rx_buf)
		{
			if (total_len == 0)
				snprintf(out_buf, out_buf_size, "TIMEOUT(%ds)", timeout_sec);
			break;
		}

		int copy_len = rx_len;
		if (total_len + copy_len >= out_buf_size - 1)
			copy_len = out_buf_size - 1 - total_len;

		memcpy(out_buf + total_len, rx_buf, copy_len);
		total_len += copy_len;
		out_buf[total_len] = '\0';
		free(rx_buf);

		/* Stop if we got a final response */
		if (strstr(out_buf, "\r\nOK\r\n") || strstr(out_buf, "\r\nERROR\r\n")
				|| strstr(out_buf, "OK\r\n") || strstr(out_buf, "ERROR\r\n"))
			break;
	}

	return SUCCESS;
} // uint8_t spi_AT_send_recv(...)


static void init_master_hd(spi_device_handle_t* spi)
{
	spi_device_handle_t spi_dev;

	/* queue init */
	ctrl_msg_Q = create_esp_queue();
	if (!ctrl_msg_Q) {
		M1_LOG_E(TAG, "Failed to create app ctrl msg Q\r\n");
		return;
	}
    // Create the message queue.
    esp_spi_msg_queue = xQueueCreate(5, sizeof(spi_master_msg_t));
    // Create the tx_buf.
    spi_master_tx_ring_buf = xStreamBufferCreate(STREAM_BUFFER_SIZE, 1);
    // Create the semaphore.
    pxMutex = xSemaphoreCreateMutex();

    /* semaphore init */
	esp_ctrl_req_sem = xSemaphoreCreateBinary();
	assert(esp_ctrl_req_sem);
	esp_resp_read_sem = xSemaphoreCreateBinary();
	assert(esp_resp_read_sem );
	/*
	Note that binary semaphores created using
	 * the vSemaphoreCreateBinary() macro are created in a state such that the
	 * first call to 'take' the semaphore would pass, whereas binary semaphores
	 * created using xSemaphoreCreateBinary() are created in a state such that the
	 * the semaphore must first be 'given' before it can be 'taken'
	 *
	*/
	/* Get read semaphore for first time */
	//xSemaphoreTake(esp_resp_read_sem, portMAX_DELAY);
	/* Give req semaphore for first time */
	xSemaphoreGive(esp_ctrl_req_sem);

    spi_dev = pvPortMalloc(sizeof(struct spi_device_t));
    assert(spi_dev!=NULL);
    memset(spi_dev, 0, sizeof(struct spi_device_t));
    spi_dev->id = ESP_SPI_ID;
    spi_dev->cfg.flags = 0;
    spi_dev->host = pvPortMalloc(sizeof(spi_host_t));
    assert(spi_dev->host!=NULL);
    memset(spi_dev->host, 0, sizeof(spi_host_t));
    *spi = spi_dev;

    spi_mutex_lock();

    /* Poll slave status with retries — ESP32 may still be booting */
    spi_recv_opt_t recv_opt = {};
    int retry;
    for (retry = 0; retry < 20; retry++) {
        recv_opt = query_slave_data_trans_info();
        M1_LOG_I(TAG, "init query[%d]: direct=%u seq=%u len=%u\r\n",
                 retry, recv_opt.direct, recv_opt.seq_num, recv_opt.transmit_len);
        if (recv_opt.direct == SPI_READ || recv_opt.direct == SPI_WRITE) {
            break;  /* Got a valid response from slave */
        }
        spi_mutex_unlock();
        HAL_Delay(500);  /* Wait 500ms between retries */
        spi_mutex_lock();
    }
    if (retry >= 20) {
        M1_LOG_E(TAG, "ESP32 slave not responding after %d retries\r\n", retry);
    }

    if (recv_opt.direct == SPI_READ) {
        if (recv_opt.seq_num != ((current_recv_seq + 1) & 0xFF)) {
            M1_LOG_E(TAG, "SPI recv seq error, %x, %x\r\n", recv_opt.seq_num, (current_recv_seq + 1));
            if (recv_opt.seq_num == 1) {
                M1_LOG_E(TAG, "Maybe SLAVE restart, ignore\r\n");
            }
        }
        current_recv_seq = recv_opt.seq_num;
        /* Must actually read the pending data via RDDMA before sending INT0.
         * Without this, the slave's DMA TX transaction never completes and
         * its SPI task hangs forever waiting on spi_slave_hd_get_trans_res(). */
        if (recv_opt.transmit_len > 0 && recv_opt.transmit_len <= SPI_TRANS_MAX_LEN) {
            uint8_t *drain_buf = pvPortMalloc(recv_opt.transmit_len);
            if (drain_buf) {
                at_spi_master_recv_data(drain_buf, recv_opt.transmit_len);
                M1_LOG_I(TAG, "Drained %u bytes of boot data from slave\r\n",
                         recv_opt.transmit_len);
                vPortFree(drain_buf);
            }
        }
        at_spi_rddma_done();
    }

    spi_mutex_unlock();
} // static void init_master_hd(spi_device_handle_t* spi)


bool get_esp32_main_init_status(void)
{
	return esp32_main_init_done;
} // bool get_esp32_main_init_status(void)


/**
  * @brief Delay without context switch
  * @param  x in ms approximately
  * @retval None
  */
void hard_delay(uint32_t x)
{
    volatile uint32_t idx;

    for (idx=0; idx<6000*x; idx++) // 100
    {
    	;
    }
}


/**
  * @brief  Reset slave to initialize
  * @param  None
  * @retval None
  */
static void reset_slave(void)
{
	esp32_disable();
	hard_delay(1); // 50
	esp32_enable();
	/* Brief delay for ESP32-C6 SPI slave to initialize after reset.
	 * Stock firmware used hard_delay(200) here (~200ms busy-loop).
	 * The handshake pin check in esp32_main_init() catches missed events. */
	HAL_Delay(200);
}



void esp32_main_init(void)
{
	BaseType_t ret;
	size_t free_heap;

	if ( esp32_main_init_done )
		return;

	reset_slave();

	init_master_hd(&spi_dev_handle);
    ret = xTaskCreate(spi_trans_control_task, "spi_trans_control_task", M1_TASK_STACK_SIZE_2048, NULL, TASK_PRIORITY_ESP32_TASKS, NULL);
	assert(ret==pdPASS);
	free_heap = xPortGetFreeHeapSize(); // xPortGetMinimumEverFreeHeapSize()
	assert(free_heap >= M1_LOW_FREE_HEAP_WARNING_SIZE);

	/* If handshake pin is already HIGH, we missed the rising edge interrupt.
	 * Manually enqueue a message so spi_trans_control_task processes it. */
	if (HAL_GPIO_ReadPin(ESP32_HANDSHAKE_GPIO_Port, ESP32_HANDSHAKE_Pin) == GPIO_PIN_SET) {
		M1_LOG_I(TAG, "Handshake already HIGH — injecting missed event\r\n");
		spi_master_msg_t spi_msg = { .slave_notify_flag = true };
		xQueueSend(esp_spi_msg_queue, (void*)&spi_msg, 0);
	}

	esp32_main_init_done = true;
} // void app_main(void)


static void esp_free_mem( char **buf_ptr)
{
	if ( *buf_ptr != NULL )
	{
		free (*buf_ptr);
		*buf_ptr = NULL;
	}
} // static void esp_free_mem( char **buf_ptr)


uint8_t wifi_ap_scan_list(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *ok_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);
	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_WIFI_MODE, ESP32C6_WIFI_MODE_STA));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass ) // at least one second has passed?
			{
				tick_t0 += MILLISEC_TO_SEC; // Update tick_t0
				if ( app_req->cmd_timeout_sec > tick_pass )
				{
					app_req->cmd_timeout_sec -= tick_pass;
				}
				else
					break; // Timeout
			} // if ( tick_pass )
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf )
				continue;
			if ( rx_uid != current_uid ) // Not the expected response?
				continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) ) // Not the expected response?
				continue;
			ret = SUCCESS;
			break;
		} // while ( true )
		if ( ret==SUCCESS )
		{
			esp_free_mem(&app_req->at_cmd);
			esp_free_mem(&app_req->cmd_resp);
			app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_LIST_AP, ""));
			app_req->cmd_len = strlen(app_req->at_cmd);
			app_req->cmd_resp = NULL;
			ret = spi_AT_app_send_command(app_req);
			while ( ret==SUCCESS )
			{
				esp_free_mem(&resp_buf);
				rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
				resp_buf = rx_buf;
				if ( rx_buf && rx_buf_len)
				{
					if ( rx_uid != current_uid ) // Not the expected response?
						continue;
					m1_parse_spi_at_resp(rx_buf, ESP32C6_AT_RES_LIST_AP_KEY, app_req);
					ok_buf = strstr(rx_buf, "OK");
					if ( ok_buf!=NULL ) // If "OK" found in the response, it's the last response to receive from the slave
						break; // Complete and exit
					tick_pass = HAL_GetTick() - tick_t0;
					tick_pass /= MILLISEC_TO_SEC;
					if ( tick_pass ) // at least one second has passed?
					{
						tick_t0 += MILLISEC_TO_SEC; // Update tick_t0
						if ( app_req->cmd_timeout_sec > tick_pass )
						{
							app_req->cmd_timeout_sec -= tick_pass;
						}
						else
							break; // Timeout
					} // if ( tick_pass )
				} // if ( rx_buf && rx_buf_len)
				else
					ret = ERROR;
			} // while ( ret==SUCCESS )
		} // if ( ret==SUCCESS )
	} // if ( ret==SUCCESS )
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	} // if ( ret==SUCCESS )
	else
	{
		M1_LOG_E(TAG, "Response not received\r\n");
	}

	return ret;
} // uint8_t wifi_ap_scan_list(ctrl_cmd_t *app_req)



uint8_t ble_scan_list(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *ok_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);
	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_MODE, ESP32C6_BLE_MODE_CLI));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass ) // at least one second has passed?
			{
				tick_t0 += MILLISEC_TO_SEC; // Update tick_t0
				if ( app_req->cmd_timeout_sec > tick_pass )
				{
					app_req->cmd_timeout_sec -= tick_pass;
				}
				else
					break; // Timeout
			} // if ( tick_pass )
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf )
				continue;
			if ( rx_uid != current_uid ) // Not the expected response?
				continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) ) // Not the expected response?
				continue;
			ret = SUCCESS;
			break;
		} // while ( true )
		if ( ret==SUCCESS )
		{
			esp_free_mem(&app_req->at_cmd);
			esp_free_mem(&app_req->cmd_resp);
			app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_SCAN, "1")); // Scan for 3 seconds, hard coded
			app_req->cmd_len = strlen(app_req->at_cmd);
			app_req->cmd_resp = NULL;
			ret = spi_AT_app_send_command(app_req);
			while ( ret==SUCCESS )
			{
				esp_free_mem(&resp_buf);
				rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
				resp_buf = rx_buf;
				if ( rx_buf && rx_buf_len)
				{
					if ( rx_uid != current_uid ) // Not the expected response?
						continue;
					m1_parse_spi_at_resp(rx_buf, ESP32C6_AT_RES_BLE_SCAN_KEY, app_req);
					ok_buf = strstr(rx_buf, "+BLESCANDONE");
					if ( ok_buf!=NULL ) // If "+BLESCANDONE" found in the response, it's the last response to receive from the slave
					{
						break; // Complete and exit
					}
					tick_pass = HAL_GetTick() - tick_t0;
					tick_pass /= MILLISEC_TO_SEC;
					if ( tick_pass ) // at least one second has passed?
					{
						tick_t0 += MILLISEC_TO_SEC; // Update tick_t0
						if ( app_req->cmd_timeout_sec > tick_pass )
						{
							app_req->cmd_timeout_sec -= tick_pass;
						}
						else
							break; // Timeout
					} // if ( tick_pass )
				} // if ( rx_buf && rx_buf_len)
				else
				{
					ret = ERROR;
					break;
				} // else
			} // while ( ret==SUCCESS )
		} // if ( ret==SUCCESS )
	} // if ( ret==SUCCESS )
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	} // if ( ret==SUCCESS )
	else
	{
		M1_LOG_E(TAG, "Response not received\r\n");
	}

	return ret;
} // uint8_t ble_scan_list(ctrl_cmd_t *app_req)



#ifdef M1_APP_BT_MANAGE_ENABLE

uint8_t ble_scan_list_ex(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *ok_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);

	/* Initialize ble_scan union member */
	app_req->u.ble_scan.count = 0;
	app_req->u.ble_scan.out_list = NULL;

	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_MODE, ESP32C6_BLE_MODE_CLI));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass )
			{
				tick_t0 += MILLISEC_TO_SEC;
				if ( app_req->cmd_timeout_sec > tick_pass )
					app_req->cmd_timeout_sec -= tick_pass;
				else
					break;
			}
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf )
				continue;
			if ( rx_uid != current_uid )
				continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) )
				continue;
			ret = SUCCESS;
			break;
		}
		if ( ret==SUCCESS )
		{
			esp_free_mem(&app_req->at_cmd);
			esp_free_mem(&app_req->cmd_resp);
			app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_SCAN, "3"));
			app_req->cmd_len = strlen(app_req->at_cmd);
			app_req->cmd_resp = NULL;
			app_req->msg_id = CTRL_RESP_GET_BLE_SCAN_LIST;
			ret = spi_AT_app_send_command(app_req);
			while ( ret==SUCCESS )
			{
				esp_free_mem(&resp_buf);
				rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
				resp_buf = rx_buf;
				if ( rx_buf && rx_buf_len)
				{
					if ( rx_uid != current_uid )
						continue;
					m1_parse_ble_scan_resp(rx_buf, ESP32C6_AT_RES_BLE_SCAN_KEY, app_req);
					ok_buf = strstr(rx_buf, "+BLESCANDONE");
					if ( ok_buf!=NULL )
					{
						break;
					}
					tick_pass = HAL_GetTick() - tick_t0;
					tick_pass /= MILLISEC_TO_SEC;
					if ( tick_pass )
					{
						tick_t0 += MILLISEC_TO_SEC;
						if ( app_req->cmd_timeout_sec > tick_pass )
							app_req->cmd_timeout_sec -= tick_pass;
						else
							break;
					}
				}
				else
				{
					ret = ERROR;
					break;
				}
			}
		}
	}
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	}
	else
	{
		M1_LOG_E(TAG, "BLE scan response not received\r\n");
	}

	return ret;
} // uint8_t ble_scan_list_ex(ctrl_cmd_t *app_req)



uint8_t esp_get_version(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *resp_buf = NULL;
	char *index;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);
	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_GET_VERSION, ""));
	app_req->cmd_resp = NULL;
	app_req->cmd_len = strlen(app_req->at_cmd);

	/* Clear version output */
	memset(app_req->u.wifi_ap_config.status, 0, STATUS_LENGTH);

	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass )
			{
				tick_t0 += MILLISEC_TO_SEC;
				if ( app_req->cmd_timeout_sec > tick_pass )
					app_req->cmd_timeout_sec -= tick_pass;
				else
					break;
			}
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			if ( !rx_buf || !rx_buf_len )
				continue;
			if ( rx_uid != current_uid )
				continue;

			/* Parse "AT version:x.x.x.x..." */
			index = strstr(rx_buf, ESP32C6_AT_RES_VERSION_KEY);
			if ( index )
			{
				size_t i;
				index += strlen(ESP32C6_AT_RES_VERSION_KEY);
				for (i = 0; i < STATUS_LENGTH - 1 && index[i] != '\0' &&
					index[i] != '\r' && index[i] != '\n' && index[i] != '('; i++)
				{
					app_req->u.wifi_ap_config.status[i] = index[i];
				}
				app_req->u.wifi_ap_config.status[i] = '\0';
			}

			/* Check for final "OK" */
			if ( strstr(rx_buf, ESP32C6_AT_RES_OK) )
			{
				ret = SUCCESS;
				break;
			}
		}
	}
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	}
	else
	{
		M1_LOG_E(TAG, "Version query failed\r\n");
	}

	return ret;
} // uint8_t esp_get_version(ctrl_cmd_t *app_req)



uint8_t ble_connect(ctrl_cmd_t *app_req, const char *addr, uint8_t addr_type)
{
	char *rx_buf = NULL;
	char *resp_buf = NULL;
	char at_cmd_buf[64];
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);

	/* Step 1: Init BLE in client mode */
	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_MODE, ESP32C6_BLE_MODE_CLI));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass )
			{
				tick_t0 += MILLISEC_TO_SEC;
				if ( app_req->cmd_timeout_sec > tick_pass )
					app_req->cmd_timeout_sec -= tick_pass;
				else
					break;
			}
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf ) continue;
			if ( rx_uid != current_uid ) continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) ) continue;
			ret = SUCCESS;
			break;
		}

		/* Step 2: Connect to device AT+BLECONN=0,"addr",addr_type */
		if ( ret==SUCCESS )
		{
			esp_free_mem(&app_req->at_cmd);
			esp_free_mem(&app_req->cmd_resp);

			snprintf(at_cmd_buf, sizeof(at_cmd_buf), "%s0,\"%s\",%u%s",
					ESP32C6_AT_REQ_BLE_CONNECT, addr, addr_type, ESP32C6_AT_REQ_CRLF);
			app_req->at_cmd = strdup(at_cmd_buf);
			app_req->cmd_len = strlen(app_req->at_cmd);
			app_req->cmd_resp = NULL;

			if ( app_req->cmd_timeout_sec < 15 )
				app_req->cmd_timeout_sec = 15;

			ret = spi_AT_app_send_command(app_req);
			app_req->resp_event_status = FAILURE;

			while ( ret==SUCCESS )
			{
				esp_free_mem(&resp_buf);
				rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
				resp_buf = rx_buf;
				if ( rx_buf && rx_buf_len )
				{
					if ( rx_uid != current_uid ) continue;

					/* Check for +BLECONN: success */
					if ( strstr(rx_buf, ESP32C6_AT_RES_BLE_CONNECT_KEY) )
						app_req->resp_event_status = SUCCESS;

					/* Check for final OK or ERROR */
					if ( strstr(rx_buf, ESP32C6_AT_RES_OK) )
					{
						if ( app_req->resp_event_status == SUCCESS )
							break;
					}
					if ( strstr(rx_buf, "ERROR") )
					{
						ret = ERROR;
						break;
					}

					tick_pass = HAL_GetTick() - tick_t0;
					tick_pass /= MILLISEC_TO_SEC;
					if ( tick_pass )
					{
						tick_t0 += MILLISEC_TO_SEC;
						if ( app_req->cmd_timeout_sec > tick_pass )
							app_req->cmd_timeout_sec -= tick_pass;
						else
							break;
					}
				}
				else
				{
					ret = ERROR;
				}
			}
		}
	}

	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS && app_req->resp_event_status==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
	}
	else
	{
		ret = ERROR;
		M1_LOG_E(TAG, "BLE connect failed\r\n");
	}

	return ret;
} // uint8_t ble_connect(ctrl_cmd_t *app_req, const char *addr, uint8_t addr_type)



uint8_t ble_disconnect(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);
	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_DISCONNECT, "0"));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass )
			{
				tick_t0 += MILLISEC_TO_SEC;
				if ( app_req->cmd_timeout_sec > tick_pass )
					app_req->cmd_timeout_sec -= tick_pass;
				else
					break;
			}
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf ) continue;
			if ( rx_uid != current_uid ) continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) ) continue;
			ret = SUCCESS;
			break;
		}
	}
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	}
	else
	{
		M1_LOG_E(TAG, "BLE disconnect failed\r\n");
	}

	return ret;
} // uint8_t ble_disconnect(ctrl_cmd_t *app_req)

#endif /* M1_APP_BT_MANAGE_ENABLE */


// Helper: send an AT command and wait for "OK" response (5-sec timeout)
// Reuses app_req's SPI transport. Best-effort — caller decides how to handle ERROR.
static uint8_t esp_at_send_wait_ok(ctrl_cmd_t *app_req, const char *at_cmd_str)
{
	char *rx_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0;
	uint8_t timeout_sec = 5;

	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	app_req->at_cmd = strdup(at_cmd_str);
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);

	ret = spi_AT_app_send_command(app_req);
	if ( ret!=SUCCESS )
	{
		esp_free_mem(&app_req->at_cmd);
		esp_free_mem(&app_req->cmd_resp);
		return ERROR;
	}

	ret = ERROR;
	tick_t0 = HAL_GetTick();
	while (true)
	{
		uint32_t tick_pass = (HAL_GetTick() - tick_t0) / MILLISEC_TO_SEC;
		if ( tick_pass >= timeout_sec )
			break;
		esp_free_mem(&resp_buf);
		vTaskDelay(100);
		rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, timeout_sec);
		resp_buf = rx_buf;
		rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
		if ( !rx_buf ) continue;
		if ( rx_uid != current_uid ) continue;
		if ( strcmp(rx_buf, app_req->cmd_resp) ) continue; // Not "OK"
		ret = SUCCESS;
		break;
	}

	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	return ret;
}


uint8_t ble_advertise(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *ok_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);

	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_MODE, ESP32C6_BLE_MODE_SER));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass ) // at least one second has passed?
			{
				tick_t0 += MILLISEC_TO_SEC; // Update tick_t0
				if ( app_req->cmd_timeout_sec > tick_pass )
				{
					app_req->cmd_timeout_sec -= tick_pass;
				}
				else
					break; // Timeout
			} // if ( tick_pass )
			esp_free_mem(&resp_buf);
			vTaskDelay(100); // Give the system some time to avoid possible crash for unknown reason
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf )
				continue;
			if ( rx_uid != current_uid ) // Not the expected response?
				continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) ) // Not the expected response?
				continue;
			ret = SUCCESS;
			break;
		} // while ( true )
		if ( ret==SUCCESS )
		{
			// Set up GATT service and security before advertising (best-effort)
			esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_GATTS_CRE, ""));
			esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_GATTS_START, ""));
			esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_SEC_PARAM, ""));

			esp_free_mem(&app_req->at_cmd);
			esp_free_mem(&app_req->cmd_resp);
			app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_ADVERTISE, ESP32C6_AT_REQ_ADV_DATA));
			app_req->cmd_len = strlen(app_req->at_cmd);
			app_req->cmd_resp = NULL;
			ret = spi_AT_app_send_command(app_req);
			while ( true )
			{
				esp_free_mem(&resp_buf);
				vTaskDelay(100); // Give the system some time to avoid possible crash for unknown reason
				rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
				resp_buf = rx_buf;
				if ( rx_buf && rx_buf_len)
				{
					if ( rx_uid != current_uid ) // Not the expected response?
						continue;
					ok_buf = strstr(rx_buf, "OK");
					if ( ok_buf!=NULL ) // If "OK" found in the response, it's the last response to receive from the slave
					{
						break; // Complete and exit
					}
					tick_pass = HAL_GetTick() - tick_t0;
					tick_pass /= MILLISEC_TO_SEC;
					if ( tick_pass ) // at least one second has passed?
					{
						tick_t0 += MILLISEC_TO_SEC; // Update tick_t0
						if ( app_req->cmd_timeout_sec > tick_pass )
						{
							app_req->cmd_timeout_sec -= tick_pass;
						}
						else
							break; // Timeout
					} // if ( tick_pass )
				} // if ( rx_buf && rx_buf_len)
				else
				{
					ret = ERROR;
					break;
				}
			} // while ( true )
		} // if ( ret==SUCCESS )
	} // if ( ret==SUCCESS )
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	} // if ( ret==SUCCESS )
	else
	{
		M1_LOG_E(TAG, "Response not received\r\n");
	}

	return ret;
} // uint8_t ble_advertise(ctrl_cmd_t *app_req)




#ifdef M1_APP_BADBT_ENABLE

uint8_t ble_hid_init(ctrl_cmd_t *app_req, const char *device_name)
{
	uint8_t ret;
	uint8_t fail_step = 0;

	esp_queue_reset(ctrl_msg_Q);

	// Step 1: Init BLE in server mode
	esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_MODE, ESP32C6_BLE_MODE_SER));
	vTaskDelay(200);

	// Step 2: Set GAP device name (shown after BLE connection)
	{
		char name_cmd[64];
		const char *gap_name = (device_name && device_name[0] != '\0') ? device_name : "M1-BadBT";
		snprintf(name_cmd, sizeof(name_cmd), "%s\"%s\"\r\n", ESP32C6_AT_REQ_BLE_NAME, gap_name);
		esp_at_send_wait_ok(app_req, name_cmd);
	}

	// Step 3: Register HID GATT service/appearance on ESP32
	ret = esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_HID_INIT, "1"));
	if (ret != SUCCESS) { fail_step = 3; goto cleanup; }

	// Step 4: Set security parameters for HID pairing
	esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_SEC_PARAM, ""));

	// Step 5: Set advertising parameters & raw data with keyboard appearance
	{
		char adv_cmd[196];
		const char *name = (device_name && device_name[0] != '\0') ? device_name : "M1-BadBT";
		uint8_t name_len = strlen(name);
		if (name_len > 20) name_len = 20; /* cap to fit 31-byte adv limit */

		/* AT+BLEADVPARAM: min=32(20ms),max=64(40ms),type=0(connectable),addr=0,ch=7,filter=0 */
		esp_at_send_wait_ok(app_req, "AT+BLEADVPARAM=32,64,0,0,7,0\r\n");
		vTaskDelay(50);

		/* Build raw advertising data hex string:
		 *   02 01 06               - Flags: LE General Discoverable + BR/EDR Not Supported
		 *   03 03 12 18            - Complete 16-bit UUID: 0x1812 (HID)
		 *   03 19 C1 03            - Appearance: 0x03C1 (Keyboard)
		 *   XX 09 <name bytes>     - Complete Local Name
		 */
		char hex[128];
		int pos = 0;
		/* Flags */
		pos += snprintf(hex + pos, sizeof(hex) - pos, "020106");
		/* UUID 0x1812 (little-endian) */
		pos += snprintf(hex + pos, sizeof(hex) - pos, "03031218");
		/* Appearance 0x03C1 = Keyboard (little-endian: C1 03) */
		pos += snprintf(hex + pos, sizeof(hex) - pos, "0319C103");
		/* Complete Local Name */
		pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X09", name_len + 1);
		for (uint8_t i = 0; i < name_len; i++)
			pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X", (uint8_t)name[i]);

		snprintf(adv_cmd, sizeof(adv_cmd),
		         "AT+BLEADVDATA=\"%s\"\r\n", hex);
		ret = esp_at_send_wait_ok(app_req, adv_cmd);
	}
	if (ret != SUCCESS) { fail_step = 5; goto cleanup; }

	// Step 6: Start advertising
	esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_ADV_START, ""));

	app_req->msg_type = CTRL_RESP;
	app_req->resp_event_status = SUCCESS;
	return SUCCESS;

cleanup:
	// Deinit BLE so stock BT/WiFi works after failure
	esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_ADV_STOP, ""));
	esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_MODE, ESP32C6_BLE_MODE_NULL));
	return fail_step;
}


uint8_t ble_hid_deinit(ctrl_cmd_t *app_req)
{
	esp_queue_reset(ctrl_msg_Q);

	// Stop advertising
	esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_ADV_STOP, ""));

	// Reset HID registration state so next init re-registers GATT services
	esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_HID_INIT, "0"));

	// Deinit BLE entirely
	esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_MODE, ESP32C6_BLE_MODE_NULL));

	return SUCCESS;
}


uint8_t ble_hid_send_kb(ctrl_cmd_t *app_req, uint8_t modifier, uint8_t key1)
{
	char cmd[48];
	uint8_t ret;

	// Format: AT+BLEHIDKB=<mod>,<k1>,<k2>,<k3>,<k4>,<k5>,<k6>\r\n
	snprintf(cmd, sizeof(cmd), "%s%d,%d,0,0,0,0,0\r\n",
			ESP32C6_AT_REQ_BLE_HID_KB, modifier, key1);

	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	app_req->at_cmd = strdup(cmd);
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);

	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		// Brief wait for OK — don't block long for keystroke throughput
		char *rx_buf = NULL;
		char *resp_buf = NULL;
		int rx_buf_len = 0;
		uint32_t rx_uid;
		uint32_t tick_t0 = HAL_GetTick();

		ret = ERROR;
		while (true)
		{
			uint32_t tick_pass = (HAL_GetTick() - tick_t0) / MILLISEC_TO_SEC;
			if ( tick_pass >= 2 ) // 2-sec timeout
				break;
			esp_free_mem(&resp_buf);
			vTaskDelay(10); // Short delay for keystroke speed
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, 2);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf ) continue;
			if ( rx_uid != current_uid ) continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) ) continue;
			ret = SUCCESS;
			break;
		}
		esp_free_mem(&resp_buf);
	}

	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	return ret;
}


// Wait for BLE HID connection + security handshake to complete.
// Handles: +BLECONN: → +BLESECREQ: → AT+BLEENC → +BLEAUTHCMPL:
// Also handles +BLESECNTFYNUM: (numeric comparison) → AT+BLECONFREPLY
// Returns SUCCESS when connection + pairing are both done.
uint8_t ble_hid_wait_connect(ctrl_cmd_t *app_req, uint8_t timeout_sec)
{
	char *rx_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret = ERROR;
	uint8_t got_conn = 0;
	uint8_t got_auth = 0;
	uint32_t tick_t0 = HAL_GetTick();

	while (true)
	{
		uint32_t tick_pass = (HAL_GetTick() - tick_t0) / MILLISEC_TO_SEC;
		if ( tick_pass >= timeout_sec )
			break;

		esp_free_mem(&resp_buf);
		vTaskDelay(100);
		rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, timeout_sec);
		resp_buf = rx_buf;
		if ( !rx_buf || !rx_buf_len )
			continue;

		M1_LOG_I(TAG, "BLE evt: %s\r\n", rx_buf);

		// Check for connection event
		if ( strstr(rx_buf, "+BLECONN:") || strstr(rx_buf, "+BLEHIDCONN:") )
		{
			got_conn = 1;
			M1_LOG_I(TAG, "BLE HID connected\r\n");
			// Reset timeout — give security handshake time to complete
			tick_t0 = HAL_GetTick();
			if ( timeout_sec < 15 )
				timeout_sec = 15;
			continue;
		}

		// Security request from remote device — initiate encryption
		if ( strstr(rx_buf, "+BLESECREQ:") )
		{
			M1_LOG_I(TAG, "Security request — starting encryption\r\n");
			// AT+BLEENC=0,3  (conn_index=0, sec_act=3)
			esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_ENC, "0,3"));
			continue;
		}

		// Numeric comparison — auto-confirm (Just Works with NoInputNoOutput)
		if ( strstr(rx_buf, "+BLESECNTFYNUM:") )
		{
			M1_LOG_I(TAG, "Numeric comparison — auto-confirming\r\n");
			// AT+BLECONFREPLY=0,1  (conn_index=0, confirm=1)
			esp_at_send_wait_ok(app_req, CONCAT_CMD_PARAM(ESP32C6_AT_REQ_BLE_CONF_REPLY, "0,1"));
			continue;
		}

		// Authentication complete — pairing done
		if ( strstr(rx_buf, "+BLEAUTHCMPL:") )
		{
			got_auth = 1;
			M1_LOG_I(TAG, "BLE auth complete\r\n");
			break; // Connection + auth done
		}

		// Disconnection during handshake
		if ( strstr(rx_buf, "+BLEDISCONN:") )
		{
			M1_LOG_W(TAG, "Disconnected during pairing\r\n");
			got_conn = 0;
			break;
		}
	}

	esp_free_mem(&resp_buf);

	if ( got_conn && got_auth )
	{
		ret = SUCCESS;
	}
	else if ( got_conn && !got_auth )
	{
		// Connected but auth didn't complete — some devices don't trigger BLEAUTHCMPL
		// Give it a shot anyway, the connection may still be usable
		M1_LOG_W(TAG, "Connected but no auth event — proceeding anyway\r\n");
		ret = SUCCESS;
	}

	return ret;
}

#endif /* M1_APP_BADBT_ENABLE */


uint8_t esp_dev_reset(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *ok_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret, got_at_reset = 0;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);
	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_RESET, ""));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_READY);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass ) // at least one second has passed?
			{
				tick_t0 += MILLISEC_TO_SEC; // Update tick_t0
				if ( app_req->cmd_timeout_sec > tick_pass )
				{
					app_req->cmd_timeout_sec -= tick_pass;
				}
				else
					break; // Timeout
			} // if ( tick_pass )
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf )
				continue;
			if ( rx_uid != current_uid ) // Not the expected response?
				continue;
			if ( !got_at_reset )
			{
				if ( strcmp(rx_buf, ESP32C6_AT_RESET) ) // Not the expected response?
					continue;
				got_at_reset = 1;
				continue;
			} // if ( !got_at_reset )
			else
			{
				if ( strcmp(rx_buf, app_req->cmd_resp) ) // Not the expected response?
					continue;
			}
			ret = SUCCESS;
			break;
		} // while ( true )
	} // if ( ret==SUCCESS )
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	} // if ( ret==SUCCESS )
	else
	{
		M1_LOG_E(TAG, "Response not received\r\n");
	}

	return ret;
} // uint8_t esp_dev_reset(ctrl_cmd_t *app_req)


#ifdef M1_APP_WIFI_CONNECT_ENABLE

uint8_t wifi_connect_ap(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *ok_buf = NULL;
	char *resp_buf = NULL;
	char at_cmd_buf[128];
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;
	uint8_t got_ip = 0;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);

	/* Step 1: Set station mode */
	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_WIFI_MODE, ESP32C6_WIFI_MODE_STA));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass )
			{
				tick_t0 += MILLISEC_TO_SEC;
				if ( app_req->cmd_timeout_sec > tick_pass )
					app_req->cmd_timeout_sec -= tick_pass;
				else
					break;
			}
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf )
				continue;
			if ( rx_uid != current_uid )
				continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) )
				continue;
			ret = SUCCESS;
			break;
		}

		/* Step 2: Send connect command AT+CWJAP="ssid","password" */
		if ( ret==SUCCESS )
		{
			esp_free_mem(&app_req->at_cmd);
			esp_free_mem(&app_req->cmd_resp);

			snprintf(at_cmd_buf, sizeof(at_cmd_buf), "%s\"%s\",\"%s\"%s",
					ESP32C6_AT_REQ_CONNECT_AP,
					(char *)app_req->u.wifi_ap_config.ssid,
					(char *)app_req->u.wifi_ap_config.pwd,
					ESP32C6_AT_REQ_CRLF);
			app_req->at_cmd = strdup(at_cmd_buf);
			app_req->cmd_len = strlen(app_req->at_cmd);
			app_req->cmd_resp = NULL;

			/* Use longer timeout for connect */
			if ( app_req->cmd_timeout_sec < DEFAULT_CTRL_RESP_CONNECT_AP_TIMEOUT )
				app_req->cmd_timeout_sec = DEFAULT_CTRL_RESP_CONNECT_AP_TIMEOUT;

			ret = spi_AT_app_send_command(app_req);
			got_ip = 0;
			app_req->resp_event_status = FAILURE;

			while ( ret==SUCCESS )
			{
				esp_free_mem(&resp_buf);
				rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
				resp_buf = rx_buf;
				if ( rx_buf && rx_buf_len )
				{
					if ( rx_uid != current_uid )
						continue;

					/* Check for "WIFI GOT IP" */
					if ( strstr(rx_buf, ESP32C6_AT_RES_WIFI_GOT_IP) )
						got_ip = 1;

					/* Check for error: +CWJAP:<error_code> */
					ok_buf = strstr(rx_buf, ESP32C6_AT_RES_CONNECT_AP_KEY);
					if ( ok_buf )
					{
						/* Parse error code */
						app_req->resp_event_status = strtol(ok_buf + strlen(ESP32C6_AT_RES_CONNECT_AP_KEY), NULL, 10);
					}

					/* Check for final "OK" or "FAIL" */
					ok_buf = strstr(rx_buf, ESP32C6_AT_RES_OK);
					if ( ok_buf )
					{
						if ( got_ip )
							app_req->resp_event_status = SUCCESS;
						break;
					}
					ok_buf = strstr(rx_buf, ESP32C6_AT_RES_FAIL);
					if ( ok_buf )
					{
						ret = ERROR;
						break;
					}

					tick_pass = HAL_GetTick() - tick_t0;
					tick_pass /= MILLISEC_TO_SEC;
					if ( tick_pass )
					{
						tick_t0 += MILLISEC_TO_SEC;
						if ( app_req->cmd_timeout_sec > tick_pass )
							app_req->cmd_timeout_sec -= tick_pass;
						else
							break;
					}
				}
				else
				{
					ret = ERROR;
				}
			} // while ( ret==SUCCESS )
		} // if ( ret==SUCCESS ) step 2
	} // if ( ret==SUCCESS ) step 1

	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS && app_req->resp_event_status==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
	}
	else
	{
		ret = ERROR;
		M1_LOG_E(TAG, "WiFi connect failed (status %ld)\r\n", app_req->resp_event_status);
	}

	return ret;
} // uint8_t wifi_connect_ap(ctrl_cmd_t *app_req)



uint8_t wifi_disconnect_ap(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *resp_buf = NULL;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);
	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_DISCONNECT_AP, ""));
	app_req->cmd_resp = strdup(ESP32C6_AT_RES_OK);
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass )
			{
				tick_t0 += MILLISEC_TO_SEC;
				if ( app_req->cmd_timeout_sec > tick_pass )
					app_req->cmd_timeout_sec -= tick_pass;
				else
					break;
			}
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			rx_buf = m1_resp_string_strip(rx_buf, CR_LF);
			if ( !rx_buf )
				continue;
			if ( rx_uid != current_uid )
				continue;
			if ( strcmp(rx_buf, app_req->cmd_resp) )
				continue;
			ret = SUCCESS;
			break;
		}
	}
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	}
	else
	{
		M1_LOG_E(TAG, "WiFi disconnect failed\r\n");
	}

	return ret;
} // uint8_t wifi_disconnect_ap(ctrl_cmd_t *app_req)



uint8_t wifi_get_ip(ctrl_cmd_t *app_req)
{
	char *rx_buf = NULL;
	char *resp_buf = NULL;
	char *index, *end_index;
	int rx_buf_len = 0;
	uint32_t rx_uid;
	uint8_t ret;
	uint32_t tick_t0, tick_pass;
	size_t cp_len;

	tick_t0 = HAL_GetTick();
	esp_queue_reset(ctrl_msg_Q);

	/* Clear output fields */
	memset(app_req->u.wifi_ap_config.status, 0, STATUS_LENGTH);
	memset(app_req->u.wifi_ap_config.out_mac, 0, MAX_MAC_STR_SIZE);

	app_req->at_cmd = strdup(CONCAT_CMD_PARAM(ESP32C6_AT_REQ_GET_IP, ""));
	app_req->cmd_resp = NULL;
	app_req->cmd_len = strlen(app_req->at_cmd);
	ret = spi_AT_app_send_command(app_req);
	if ( ret==SUCCESS )
	{
		ret = ERROR;
		while (true)
		{
			tick_pass = HAL_GetTick() - tick_t0;
			tick_pass /= MILLISEC_TO_SEC;
			if ( tick_pass )
			{
				tick_t0 += MILLISEC_TO_SEC;
				if ( app_req->cmd_timeout_sec > tick_pass )
					app_req->cmd_timeout_sec -= tick_pass;
				else
					break;
			}
			esp_free_mem(&resp_buf);
			rx_buf = spi_AT_app_get_response(&rx_buf_len, &rx_uid, app_req->cmd_timeout_sec);
			resp_buf = rx_buf;
			if ( !rx_buf || !rx_buf_len )
				continue;
			if ( rx_uid != current_uid )
				continue;

			/* Parse +CIFSR:STAIP,"x.x.x.x" */
			index = strstr(rx_buf, ESP32C6_AT_RES_STAIP_KEY);
			if ( index )
			{
				index += strlen(ESP32C6_AT_RES_STAIP_KEY);
				if ( *index == '\"' ) index++;
				end_index = strstr(index, "\"");
				if ( end_index )
				{
					cp_len = end_index - index;
					if ( cp_len >= STATUS_LENGTH ) cp_len = STATUS_LENGTH - 1;
					strncpy(app_req->u.wifi_ap_config.status, index, cp_len);
					app_req->u.wifi_ap_config.status[cp_len] = '\0';
				}
			}

			/* Parse +CIFSR:STAMAC,"xx:xx:xx:xx:xx:xx" */
			index = strstr(rx_buf, ESP32C6_AT_RES_STAMAC_KEY);
			if ( index )
			{
				index += strlen(ESP32C6_AT_RES_STAMAC_KEY);
				if ( *index == '\"' ) index++;
				end_index = strstr(index, "\"");
				if ( end_index )
				{
					cp_len = end_index - index;
					if ( cp_len >= MAX_MAC_STR_SIZE ) cp_len = MAX_MAC_STR_SIZE - 1;
					strncpy(app_req->u.wifi_ap_config.out_mac, index, cp_len);
					app_req->u.wifi_ap_config.out_mac[cp_len] = '\0';
				}
			}

			/* Check for final "OK" */
			if ( strstr(rx_buf, ESP32C6_AT_RES_OK) )
			{
				ret = SUCCESS;
				break;
			}
		}
	}
	esp_free_mem(&resp_buf);
	esp_free_mem(&app_req->at_cmd);
	esp_free_mem(&app_req->cmd_resp);
	if ( ret==SUCCESS )
	{
		app_req->msg_type = CTRL_RESP;
		app_req->resp_event_status = SUCCESS;
	}
	else
	{
		M1_LOG_E(TAG, "WiFi get IP failed\r\n");
	}

	return ret;
} // uint8_t wifi_get_ip(ctrl_cmd_t *app_req)

#endif /* M1_APP_WIFI_CONNECT_ENABLE */
