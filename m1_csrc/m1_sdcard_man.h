/* See COPYING.txt for license details. */

/*
*
* m1_sdcard_man.h
*
* Library for accessing SD card
*
* M1 Project
*
*/

#ifndef M1_SDCARD_MAN_H_
#define M1_SDCARD_MAN_H_

#define M1_DATALOG_IDLE          		(0x00)
#define M1_DATALOG_SD_STARTED    		(0x01)

#define M1_SDM_CMD_MASK                	(0x8000)
#define M1_SDM_DATA_READY_MASK         	(0x4000)

#define M1_SDM_START_STOP              	(0x0001|M1_SDM_CMD_MASK)
#define M1_SDM_WRITE_UCF_TO_ROOT       	(0x0002|M1_SDM_CMD_MASK)
#define M1_SDM_CHECK_SDCARD_USAGE      	(0x0003|M1_SDM_CMD_MASK)

#define M1_SDM_MAX_WRITE_TIME      	2
#define M1_SDM_BUFFER_RAM_USAGE    	458752
#define M1_SDM_MIN_BUFFER_SIZE     	4096//3072//10240//5120//2560//3072//3840
#define M1_SDM_BUFFER_ARRAY_SIZE	7

#define NFC_FILEPATH				"/NFC"
#define NFC_FILE_EXTENSION			".nfc"
#define NFC_FILE_PREFIX				"nfc_"

#define RFID_FILEPATH				"/RFID"
#define RFID_FILE_EXTENSION			".rfid"
#define RFID_FILE_PREFIX			"rfid_"

#define DATA_FILEPATH				"/DATA"
#define DATA_FILE_EXTENSION			".dat"
#define DATA_FILE_PREFIX			"data_"

#define CONCAT_FILEPATH_FILENAME(fpath, fname) fpath fname

typedef struct
{
	const char *dir_name;
	const char *file_prefix;
	const char *file_infix;
	const char *file_suffix;
	const char *file_ext;
	uint8_t dat_filename[192];
	FIL dat_file_hdl;
} S_M1_SDM_DatFileInfo_t;

typedef struct
{
	uint8_t *sd_write_buffer;
	uint32_t sd_write_buffer_idx;
} S_M1_SDM_Buffer_Info_t;

typedef struct
{
	uint8_t isActive;
	uint32_t sdWriteBufferSize;
	S_M1_SDM_Buffer_Info_t buff_info;
} S_M1_SDM_Device_Info_t;

void m1_sdm_task_init(void);
void m1_sdm_task_deinit(void);
void m1_sdm_task_start(void);
void m1_sdm_task_stop(void);

uint8_t m1_sdm_file_init(S_M1_SDM_DatFileInfo_t *pfileinfo);
uint8_t m1_sdm_close_files(void);
uint8_t m1_sdm_close_file(void);
uint8_t m1_sdm_write_buffer(uint8_t *buffer, uint32_t size);
uint8_t m1_sdm_flush_buffer(void);
uint8_t m1_sdm_fill_buffer(uint8_t *src, uint16_t srcSize);
uint8_t m1_sdm_get_logging_error(void);
uint32_t m1_sdm_getlastfilenumber(char *dirname, char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* M1_SDCARD_MAN_H_ */
