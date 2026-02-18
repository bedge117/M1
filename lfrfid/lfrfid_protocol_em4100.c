/* See COPYING.txt for license details. */

/*
 * lfrfid_protocol_em4100.c
 *
 *      Author: pgcho
 */

/*************************** I N C L U D E S **********************************/
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>

#include "app_freertos.h"
#include "cmsis_os.h"
#include "main.h"
#include "uiView.h"

#include "lfrfid.h"

/*************************** D E F I N E S ************************************/
#define EMUL_EM4100_CORR	(3)

#define OUTPUT_INVERT	0

#define M1_LOGDB_TAG	"RFID"

// --- 헬퍼 매크로 ---
//HALF_LOW  ---- HALF ---- HALF_HIGH (= MID) ---- FULL ---- FULL_HIGH
//                  \                     /
//                   \                   /
//                      MID_REFERENCE
// 엣지 시간 t_us가 기준 시간 base_us의 절반(T_h)인지 확인 (오차 허용)
#define HALF_TOLERANCE_RATIO (1.0f - 0.75f)	// 30%
#define FULL_TOLERANCE_RATIO (1.0f + 0.30f)	// 30%
#define MID_TOLERANCE_RATIO  (1.0f + 0.60f) // 30%
#define MID_TOLERANCE_RATIOx (1.0f + 0.20f) // 30%

#define MID_REFERENCE_VALUE(t) 	(t * MID_TOLERANCE_RATIO)		// t*1.6
#define HALF_LOWER_LIMIT(t)		(t * HALF_TOLERANCE_RATIO)		// t*0.7
#define FULL_UPPER_LIMIT(t)		((2 * t) * FULL_TOLERANCE_RATIO)// 2t*1.3
#define MID_REFERENCE_VALUEx(t) (t * MID_TOLERANCE_RATIOx)		// t*1.6

#define IS_HALF_BIT(t_us, base_us) \
		((t_us) > HALF_LOWER_LIMIT(base_us)) && ((t_us) < MID_REFERENCE_VALUE(base_us)) // ((t_us) > (base_us * HALF_TOLERANCE_RATIO)) && ((t_us) < (base_us / HALF_TOLERANCE_RATIO))

// 엣지 시간 t_us가 기준 시간 base_us의 2배(T_b)인지 확인 (오차 허용)
#define IS_FULL_BIT(t_us, base_us) \
		((t_us) > MID_REFERENCE_VALUE(base_us)) && ((t_us) < FULL_UPPER_LIMIT(base_us))//((t_us) > ((2 * base_us) * FULL_TOLERANCE_RATIO)) && ((t_us) < ((2 * base_us) / FULL_TOLERANCE_RATIO))

#define IS_FULL_BITx(t_us, base_us) \
		((t_us) > MID_REFERENCE_VALUEx(base_us)) && ((t_us) < FULL_UPPER_LIMIT(base_us))//((t_us) > ((2 * base_us) * FULL_TOLERANCE_RATIO)) && ((t_us) < ((2 * base_us) / FULL_TOLERANCE_RATIO))

// --- 내부 함수 ---
//#define VERIFY_EDGE_NUM (8) // 검증에 사용할 추가 엣지 개수 (2개 검증 후 8개 추가)

#define EM4100_MAX_STEPS   (64 * 2)  // 64비트 × 2 half-bit = 128

//************************** C O N S T A N T **********************************/

//************************** S T R U C T U R E S *******************************

typedef enum {
    DEC_RESET_PARTIAL = 0,      	// 비파괴적: 카운터/상태만 초기화
    DEC_RESET_FULL = 1 << 0, 		// 파괴적: frame_buffer까지 모두 초기화
    DEC_RESET_KEEP_TIMING = 1 << 1, // 옵션: detected_half_bit_us 유지
} dec_reset_mode_t;

/***************************** V A R I A B L E S ******************************/

// 전역 디코더 상태 구조체
static EM4100_Decoder_t g_em4100_dec;
static EM4100_Decoder_t g_em4100_32_dec;
static EM4100_Decoder_t g_em4100_16_dec;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static bool Check_Even_Parity(const uint8_t* data_bits, uint8_t length);
static uint8_t GetBitFromFrame(EM4100_Decoder_t* dec, uint8_t bit_index);
static bool em4100_decoder_execute(void* proto, uint16_t size, void* dec);

static uint8_t* protocol_em4100_get_data(void* proto);
void protocol_em4100_decoder_begin(void* proto);
bool protocol_em4100_decoder_execute(void* proto, uint16_t size);
bool protocol_em4100_encoder_begin(void* proto);
void protocol_em4100_encoder_send(void* proto);
void protocol_em4100_write_begin(void* protocol, void *data);
void protocol_em4100_write_send(void* proto);
void protocol_em4100_render_data(void* protocol, char *result);
void protocol_em4100_32_decoder_begin(void* proto);
bool protocol_em4100_32_decoder_execute(void* proto, uint16_t size);
void protocol_em4100_16_decoder_begin(void* proto);
bool protocol_em4100_16_decoder_execute(void* proto, uint16_t size);

//************************** C O N S T A N T **********************************/

const LFRFIDProtocolBase protocol_em4100 = {
    .name = "EM4100",
    .manufacturer = "EM-Micro",
    .data_size = EM4100_DECODED_DATA_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)protocol_em4100_get_data,
    .decoder =
    {
        .begin = (lfrfidProtocolDecoderBegin)protocol_em4100_decoder_begin,
        .execute = (lfrfidProtocolDecoderExecute)protocol_em4100_decoder_execute,
    },
    .encoder =
    {
        .begin = (lfrfidProtocolEncoderBegin)protocol_em4100_encoder_begin,
        .send = (lfrfidProtocolEncoderSend)protocol_em4100_encoder_send,
    },
    .write =
    {
        .begin = (lfrfidProtocolWriteBegin)protocol_em4100_write_begin,
        .send = (lfrfidProtocolWriteSend)protocol_em4100_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)protocol_em4100_render_data,
    //.write_data = (lfrfidProtocolWriteData)protocol_em4100_write_data,
};

const LFRFIDProtocolBase protocol_em4100_32 = {
    .name = "EM4100/32",
    .manufacturer = "EM-Micro",
    .data_size = EM4100_DECODED_DATA_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)protocol_em4100_get_data,
    .decoder =
    {
       .begin = (lfrfidProtocolDecoderBegin)protocol_em4100_32_decoder_begin,
       .execute = (lfrfidProtocolDecoderExecute)protocol_em4100_32_decoder_execute,
    },
    .encoder =
    {
        .begin = (lfrfidProtocolEncoderBegin)protocol_em4100_encoder_begin,
        .send = (lfrfidProtocolEncoderSend)protocol_em4100_encoder_send,
    },
    .write =
    {
        .begin = (lfrfidProtocolWriteBegin)protocol_em4100_write_begin,
        .send = (lfrfidProtocolWriteSend)protocol_em4100_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)protocol_em4100_render_data,
    //.write_data = (lfrfidProtocolWriteData)protocol_em4100_write_data,
};

const LFRFIDProtocolBase protocol_em4100_16 = {
    .name = "EM4100/16",
    .manufacturer = "EM-Micro",
    .data_size = EM4100_DECODED_DATA_SIZE,
    .features = LFRFIDFeatureASK,
    .get_data = (lfrfidProtocolGetData)protocol_em4100_get_data,
    .decoder =
    {
        .begin = (lfrfidProtocolDecoderBegin)protocol_em4100_16_decoder_begin,
        .execute = (lfrfidProtocolDecoderExecute)protocol_em4100_16_decoder_execute,
    },
    .encoder =
    {
        .begin = (lfrfidProtocolEncoderBegin)protocol_em4100_encoder_begin,
        .send = (lfrfidProtocolEncoderSend)protocol_em4100_encoder_send,
    },
    .write =
    {
        .begin = (lfrfidProtocolWriteBegin)protocol_em4100_write_begin,
        .send = (lfrfidProtocolWriteSend)protocol_em4100_write_send,
    },
    .render_data = (lfrfidProtocolRenderData)protocol_em4100_render_data,
    //.write_data = (lfrfidProtocolWriteData)protocol_em4100_write_data,
};

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

#if 0
/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void setEm4100_bitrate(int bitrate)
{
	//g_em4100_dec.detected_half_bit_us = bitrate;
}
#endif


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void EM4100_Decoder_Init_Full(EM4100_Decoder_t* dec)
{
	memset(dec->frame_buffer, 0, FRAME_BUFFER_BYTES);
	dec->bit_count = 0;
	//dec->state = DECODER_STATE_IDLE;
    //g_decoder.sync_bit_count = 0;
	//dec->bit_test = 0;
	dec->edge_count = 0;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void EM4100_Decoder_Init_Partial(EM4100_Decoder_t* dec)
{
	memset(dec->frame_buffer, 0, FRAME_BUFFER_BYTES);
	dec->bit_count = 0;
	//dec->state = DECODER_STATE_IDLE;
    //g_decoder.sync_bit_count = 0;
	//dec->bit_test = 0;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static bool em4100_extract_fields(EM4100_Decoder_t* dec)
{
    // TODO: 패리티 검사 및 디코딩 완료 이벤트 처리
	bool valid = true;
	uint8_t temp_bits[11];

	// --- 5. 최종 결과 처리 (동일) ---
	if (valid) {
	    // EM4100 Frame successfully decoded and verified.
	    // TODO: 최종 ID (비트 9부터 48까지의 40비트 데이터)를 사용자에게 전달
#if 1	// data parsing
	  	memset(temp_bits, 0, 10);

        for (uint8_t col = 0; col < 10; col++) {
            for (uint8_t row = 0; row < 4; row++) {
              	temp_bits[col] |= GetBitFromFrame(dec, 9 + (col * 5) + row)<<(3-row); // temp_bits[0] ~ temp_bits[3]
            }
        }

        for(int i = 0; i<5; i++)
        {
         	lfrfid_tag_info.uid[i] = MAKEBYTE(temp_bits[i<<1], temp_bits[(i<<1)+1]);
        }

	  	lfrfid_tag_info.bitrate = dec->detected_half_bit_us/4;
#endif


	 } else {
	        // EM4100 Frame verification failed.

	 }
	return valid;
}


/*============================================================================*/
/**
  * @brief T_b 간격을 T_h 두 개로 분할하여 엣지 배열을 정규화합니다.
  * @param
  * @retval
  */
/*============================================================================*/
static uint8_t manchester_symbol_feed(lfrfid_evt_t* stream, lfrfid_evt_t* stream2,uint8_t count, uint16_t Th_us)
{
    uint8_t output_count = 0;

    for (uint8_t i = 0; i < count; ++i) {
        lfrfid_evt_t current_evt = stream2[i];

        // T_b (풀 비트) 간격인 경우
        if (IS_FULL_BIT(current_evt.t_us, Th_us)) {

            // 1. 첫 번째 T_h 엣지: T_b의 시작 레벨은 반전
        	stream[output_count].t_us = Th_us;
        	stream[output_count].edge = current_evt.edge;
            output_count++;

            // 2. 두 번째 T_h 엣지: T_b의 끝 레벨 유지
            stream[output_count].t_us = Th_us;
            stream[output_count].edge = current_evt.edge;
            output_count++;

        } else {
            // T_h (하프 비트) 간격이거나 기타 간격: 그대로 유지
            //if (i != output_count) {
            	stream[output_count] = current_evt;
            //}
            output_count++;
        }
    }
    return output_count;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void manchester_bit_feed(EM4100_Decoder_t* dec, lfrfid_evt_t* e1, lfrfid_evt_t* e2, uint8_t* bit)
{
	uint16_t T_h = dec->detected_half_bit_us;
    //uint8_t bit = 2;

    // T_b 정규화가 완료되었으므로, T_h 패턴만 검사
    bool is_valid_pattern = IS_HALF_BIT(e1->t_us, T_h) && IS_HALF_BIT(e2->t_us, T_h);

    if (is_valid_pattern) {
    	// 비트 1 판별: {T_h, 0}, {T_h, 1}
        if (e1->edge == 0 && e2->edge == 1) {
        	if(IS_FULL_BITx((e1->t_us+e2->t_us), T_h))
        		*bit = 1;
        }
        // 비트 0 판별: {T_h, 1}, {T_h, 0}
        else if (e1->edge == 1 && e2->edge == 0) {
        	if(IS_FULL_BITx((e1->t_us+e2->t_us), T_h))
        		*bit = 0;
        }
    }
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static void bit_stream_push(EM4100_Decoder_t* dec, uint8_t bit)
{
    uint8_t carry = 0;
    uint8_t new_carry;

    // 바이트 단위 왼쪽 쉬프트 (carry propagate)
    for (int i = 7; i >= 0; i--) {
        new_carry = (dec->frame_buffer[i] >> 7) & 1;   // 다음 바이트로 올 비트
        dec->frame_buffer[i] <<= 1;                    // left shift 1bit
        dec->frame_buffer[i] |= carry;                 // 이전 carry 적용
        carry = new_carry;
    }

    // LSB(맨 오른쪽 바이트)의 최하위 비트에 새 비트 추가
    dec->frame_buffer[7] &= 0xFE;         // 비트0 클리어
    dec->frame_buffer[7] |= bit;       // 비트 삽입

    // 비트 카운터 증가 (64까지)
    //dec->bit_test++;
    if (dec->bit_count < FRAME_BITS)
        dec->bit_count++;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static uint8_t bit_stream_get(uint8_t *buf, uint16_t index)
{
    return (buf[index / 8] >> (7 - (index % 8))) & 1;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
/* 디코더가 64비트 frame을 모두 채웠는지 */
static inline bool decoder_is_full(const EM4100_Decoder_t *dec) {
    return dec && (dec->bit_count >= FRAME_BITS);
}


/*============================================================================*/
/**
  * @brief 공개: 64bit frame이 em4100인지 판별
  * @param
  * @retval
  */
/*============================================================================*/
bool em4100_is_valid(const EM4100_Decoder_t *dec)
{
	uint8_t* frame = dec->frame_buffer;

    if (!frame) return false;

    //const uint8_t *raw = (const uint8_t *)frame;
    const uint16_t preamble =  ((uint16_t)frame[0]<<1) + ((frame[1]>>7) & 1);

    if (preamble != 0b111111111) return false;

    // parity check
    // TODO: 패리티 검사 및 디코딩 완료 이벤트 처리
	bool valid = true;
	uint8_t temp_bits[11];

	// --- 2. 열 패리티 (Column Parity) 검사 ---
	// 4개의 데이터 블록 (R0-R3) 검사
	for (uint8_t row = 0; row < 4; row++) {
		// 💡 10개 비트(D0-D9) 추출 및 복사
	    for (uint8_t col = 0; col < 11; col++) {
	    	// 비트 인덱스: 9 + (row * 10) + col
	        uint8_t bit_index = 9 + (col * 5) + row;
	        temp_bits[col] = GetBitFromFrame(dec, bit_index); // temp_bits[0] ~ temp_bits[9]에 저장
	    }

	    // D0-D9 (총 10비트)에 대해 짝수 패리티 검사
	    if (Check_Even_Parity(temp_bits, 11)) {
	    	valid = false;
	        break;
	    }
	}

	// --- 3. 행 패리티 (Row Parity) 검사 ---
	if (valid) {
	    for (uint8_t col = 0; col < 10; col++) {
	    	// 💡 temp_bits 배열 초기화: 5개 요소만 사용하므로 필요 없는 값 제거
	        memset(temp_bits, 0, 5); // 5개 요소만 초기화

	        // 패리티 계산에 사용되는 4개의 데이터 비트 (R0-R3의 C_col) 추출
	        for (uint8_t row = 0; row < 5; row++) {
	            // 데이터 비트 위치: 9 + (row * 10) + col
	            temp_bits[row] = GetBitFromFrame(dec, 9 + (col * 5) + row); // temp_bits[0] ~ temp_bits[3]
	        }

	        //// 행 패리티 비트 P_i 추출 (5번째 요소)
	        //uint8_t parity_bit_index = 13 + col;
	        //temp_bits[4] = GetBitFromFrame(parity_bit_index); // temp_bits[4] (P_i)

	        // 5개의 비트(4 데이터 + 1 패리티)에 대해 짝수 패리티 검사
	        if (Check_Even_Parity(temp_bits, 5)) {
	        	valid = false;
	            break;
	        }
	    }
	}

	// --- 4. 스톱 비트 검사 (동일) ---
	// 스톱 비트는 항상 '0'이어야 합니다. (비트 63)
	if (valid) {
	   if (GetBitFromFrame(dec, 63) != 0) {
	      valid = false;
	   }
	}

    return valid;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
bool em4100_decoder_execute(void* proto, uint16_t size, void* dec)
{
	lfrfid_evt_t temp_stream[FRAME_CHUNK_SIZE];
    uint8_t normalized_count;
    lfrfid_evt_t* new_stream = (lfrfid_evt_t*)proto;
    EM4100_Decoder_t *pdec = (EM4100_Decoder_t*)dec;

    // 1. 엣지 전처리 및 정규화 (T_b -> T_h + T_h 분할)
    //memcpy(temp_stream, new_stream, size * sizeof(lfrfid_evt_t));

    //if (g_decoder.state != DECODER_STATE_IDLE && g_decoder.detected_half_bit_us != 0) {
    normalized_count = manchester_symbol_feed(temp_stream, new_stream, size, pdec->detected_half_bit_us);
    //}
    //p = &temp_stream[0];

    // 정규화된 엣지를 메인 버퍼 뒤에 추가
    if (pdec->edge_count + normalized_count > sizeof(pdec->edge_buffer) / sizeof(lfrfid_evt_t)) {
        EM4100_Decoder_Init_Full(pdec);
        return false;
    }

    //if(g_decoder.edge_count > 1)
    //	M1_LOG_I(M1_LOGDB_TAG,"g_decoder.edge_count=%d, %d\r\n",g_decoder.edge_count ,normalized_count);
#if 1
    memcpy(&pdec->edge_buffer[pdec->edge_count], temp_stream, normalized_count * sizeof(lfrfid_evt_t));
    pdec->edge_count += normalized_count;
#endif

    // 2. 엣지 버퍼를 순회하며 디코딩 (상태 머신)
    uint8_t consumed_idx = 0;

    while (pdec->edge_count - consumed_idx >= 2)
    {
        lfrfid_evt_t* e1 = &pdec->edge_buffer[consumed_idx];
        lfrfid_evt_t* e2 = &pdec->edge_buffer[consumed_idx + 1];
        consumed_idx += 2;
        // 3. 상태 머신 실행
       	uint8_t bit = 2;
       	manchester_bit_feed(pdec, e1, e2, &bit);

        if (bit != 2) {
           	bit_stream_push(pdec,bit);

           	if(decoder_is_full(pdec))
          	{
           		if(em4100_is_valid(pdec))
           		{
           			em4100_extract_fields(pdec);
           			return true;
           			//m1_app_send_q_message(lfrfid_q_hdl, Q_EVENT_LFRFID_TAG_DETECTED);
           		}
           	}
        } else {
            // 패턴 불일치 또는 T_h가 아님. 리셋.
          	//EM4100_Decoder_Init_Partial();

            //if(bit == 2 && is_valid_pattern)
           	if(bit == 2)
           	{
               	consumed_idx -= 1;
               	EM4100_Decoder_Init_Partial(pdec);
            }
        }
    } // end while

    // 4. 처리되지 않고 남은 엣지를 버퍼 앞으로 이동
#if 1
    uint8_t remaining_edges;
    if(pdec->edge_count && (pdec->edge_count >= consumed_idx))
    {
    	remaining_edges = pdec->edge_count - consumed_idx;

		if (consumed_idx > 0) {

			if((FRAME_CHUNK_SIZE-consumed_idx) > remaining_edges)
			{
				//M1_LOG_I(M1_LOGDB_TAG, "1->edge_count=%d, consumed_idx=%d, remaining_edges=%d normalized_count=%d\r\n", g_decoder.edge_count,consumed_idx,remaining_edges,normalized_count);

				memmove(pdec->edge_buffer, &pdec->edge_buffer[consumed_idx], remaining_edges * sizeof(lfrfid_evt_t));
				pdec->edge_count = remaining_edges;
			}
		}
	}
    else
    {
    	//M1_LOG_I(M1_LOGDB_TAG, "2->edge_count=%d, consumed_idx=%d, remaining_edges=%d normalized_count=%d\r\n", g_decoder.edge_count,consumed_idx,remaining_edges,normalized_count);
    }
#endif
    return false;
}


/*============================================================================*/
/**
 * @brief 주어진 비트 배열에서 짝수 패리티(Even Parity)를 확인합니다.
 * @param data_bits 비트 배열 (LBS 0)
 * @param length 확인할 비트 개수
 * @return 패리티가 올바르면 true (1의 개수가 짝수), 아니면 false
 */
/*============================================================================*/
static bool Check_Even_Parity(const uint8_t* data_bits, uint8_t length)
{
    uint8_t parity = data_bits[0];

    for (uint8_t i = 1; i < length; i++) {
        parity = parity ^ data_bits[i];
    }

    // 짝수 패리티: 1의 개수가 짝수여야 함
    return (parity); // 💡 수정된 부분
}


/*============================================================================*/
/**
 * @brief 디코딩된 64비트 버퍼에서 특정 위치의 비트 값을 추출합니다.
 * @param bit_index 추출할 비트의 전체 인덱스 (0부터 63까지)
 * @return 해당 비트의 값 (0 또는 1)
 */
/*============================================================================*/
static uint8_t GetBitFromFrame(EM4100_Decoder_t*dec, uint8_t bit_index)
{
    // 비트가 저장된 바이트의 인덱스를 계산
    uint8_t byte_idx = bit_index / 8;

    // 바이트 내에서 비트의 위치를 계산 (0~7)
    uint8_t bit_idx = bit_index % 8;

    // MSB first (가장 중요한 비트부터 먼저 저장) 방식으로 저장되었으므로,
    // (7 - bit_idx)를 사용하여 가장 왼쪽 비트(MSB)부터 0, 1, 2... 순서로 접근합니다.
    if (dec->frame_buffer[byte_idx] & (1 << (7 - bit_idx))) {
        return 1;
    } else {
        return 0;
    }
}


/*============================================================================*/
/**
 * @brief 가변 길이 UID 바이트를 EM4100용 10 nibble로 정규화
 *
 * @param uid_bytes   UID 바이트 배열 (예: 1~5바이트)
 * @param uid_len     uid_bytes 길이 (바이트 수)
 * @param out_nibs    길이 10짜리 nibble 배열 (출력)
 *
 * 규칙:
 *  - 전체 10 nibble 중 '하위 nibble' 쪽에 UID를 채움 (우측 정렬)
 *  - 남는 상위 nibble은 0으로 패딩
 *  - uid_len > 5 이면, 마지막 5바이트만 사용 (LSB 기준)
 */
/*============================================================================*/
void em4100_uid_bytes_to_nibbles(const uint8_t *uid_bytes,
                                 uint8_t uid_len,
                                 uint8_t out_nibs[10])
{
    memset(out_nibs, 0, 10);

    if (uid_bytes == NULL || uid_len == 0)
        return;

    /* 최대 5바이트(=10 nibble)만 사용 */
    if (uid_len > 5)
        uid_len = 5;

    /* uid_bytes의 LSB 쪽부터 nibble 추출 → out_nibs의 뒤쪽에 채움 */
    int nib_index = 9;  // 마지막 nibble 인덱스
    for (int i = uid_len - 1; i >= 0 && nib_index >= 1; i--)
    {
        uint8_t b = uid_bytes[i];
        /* 하위 nibble 먼저 넣고, 그 앞에 상위 nibble 넣기 */
        out_nibs[nib_index--] = (b & 0x0F);       // low nibble
        out_nibs[nib_index--] = (b >> 4) & 0x0F;  // high nibble
    }
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static inline void set_bit(uint8_t frame[8], uint16_t bitpos, uint8_t bit)
{
    uint16_t byte = bitpos >> 3;      // bitpos / 8
    uint8_t  bpos = 7 - (bitpos & 7); // MSB first

    if (bit)
        frame[byte] |=  (1U << bpos);
    else
        frame[byte] &= ~(1U << bpos);
}


/*============================================================================*/
/**
  * @brief UID nibble 10개 → 8바이트 EM4100 프레임 생성
  * @param
  * @retval
  */
/*============================================================================*/
void em4100_build_frame8(uint8_t frame[8], const uint8_t uid_nibs[10])
{
    uint8_t col_ones[4] = {0};
    uint16_t bitpos = 0;

    memset(frame, 0, 8);

    // 1) preamble: 9비트 1
    for (int i = 0; i < 9; i++)
        set_bit(frame, bitpos++, 1);

    // 2) 10열 × (4bit data + 1 parity)
    for (int row = 0; row < 10; row++)
    {
        uint8_t nib = uid_nibs[row] & 0x0F;
        uint8_t row_ones = 0;

        for (int b = 0; b < 4; b++)
        {
            uint8_t bit = (nib >> (3 - b)) & 1;
            set_bit(frame, bitpos++, bit);

            if (bit)
            {
                row_ones++;
                col_ones[b]++;
            }
        }

        //uint8_t p = (row_ones & 1) ? 0 : 1; // row odd parity
        uint8_t p = (row_ones & 1);	// row even parity
        set_bit(frame, bitpos++, p);
    }

    // 3) column parity (4bit)
    for (int c = 0; c < 4; c++)
    {
        //uint8_t p = (col_ones[c] & 1) ? 0 : 1;
    	uint8_t p = (col_ones[c] & 1);
        set_bit(frame, bitpos++, p);
    }

    // 4) stop bit = 0
    set_bit(frame, bitpos++, 0);
}


/*============================================================================*/
/**
 * @brief 가변 길이 UID 바이트 → 8바이트 EM4100 프레임 생성
 *
 * @param frame8    출력: 길이 8바이트 (EM4100 64bit 프레임)
 * @param uid_bytes UID 바이트 배열 (1~N바이트, 최대 5바이트 사용)
 * @param uid_len   uid_bytes 길이
 */
/*============================================================================*/
void em4100_build_frame8_from_uid(uint8_t frame8[8],
                                  const uint8_t *uid_bytes,
                                  uint8_t uid_len)
{
    uint8_t nibs[10];
    em4100_uid_bytes_to_nibbles(uid_bytes, uid_len, nibs);
    em4100_build_frame8(frame8, nibs);
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
// MSB-first: bit_index = 0..63
//static inline uint8_t em4100_frame_get_bit(const uint8_t frame[8], uint16_t bit_index)
static uint8_t em4100_frame_get_bit(const uint8_t frame[8], uint16_t bit_index)
{
    uint16_t byte = bit_index >> 3;      // /8
    uint8_t  bpos = 7 - (bit_index & 7); // MSB-first
    return (frame[byte] >> bpos) & 0x01;
}


/*============================================================================*/
/**
 * @brief EM4100 64bit 프레임(8바이트)을 맨체스터 파형으로 변환하여 Encoded_Data_t 배열에 채운다.
 *
 * @param frame8        8바이트 EM4100 프레임 (MSB-first)
 * @param steps         출력: Encoded_Data_t 배열
 * @param max_steps     steps 배열의 최대 크기(원소 개수)
 * @param half_bit_us   half-bit 길이 (us)
 *                      예) EM4100_64 → bit=512us → half_bit_us=256
 * @param gpio_pin      사용할 GPIO 핀 번호 (0~15), 예: PA2 → 2
 * @param start_level   첫 half-bit 시작 레벨 (0=LOW, 1=HIGH)
 *
 * @return 실제로 채워진 step 개수 (에러 시 0)
 *
 * 맨체스터 규칙:
 *  bit=1 → [HIGH, LOW]
 *  bit=0 → [LOW, HIGH]
 * 각 half-bit마다 BSRR에 SET/RESET를 써서 레벨을 강제.
 */
/*============================================================================*/
uint16_t em4100_build_manchester_wave(
        const uint8_t frame8[8],
		Encoded_Data_t *steps,
        uint16_t max_steps,
        uint16_t half_bit_us,
        uint8_t  gpio_pin,
        uint8_t  start_level)
{
    if (!frame8 || !steps || max_steps == 0 || half_bit_us == 0 || gpio_pin > 15)
        return 0;

    const uint32_t bsrr_set   = (1U << gpio_pin);         // GPIOx_BSRR SET
    const uint32_t bsrr_reset = (1U << (gpio_pin + 16U)); // GPIOx_BSRR RESET

    uint16_t step_idx = 0;
    uint8_t level;

    // 현재 출력 레벨은 start_level로 시작
    level = start_level;

    // EM4100은 64bit 프레임 고정
    for (uint16_t bit = 0; bit < 64; bit++)
    {
        uint8_t v = em4100_frame_get_bit(frame8, bit);

        // bit=1 → [HIGH, LOW]
        // bit=0 → [LOW, HIGH]
#if OUTPUT_INVERT
        uint8_t first  = (v ? 0U : 1U);
        uint8_t second = (v ? 1U : 0U);
#else
        uint8_t first  = (v ? 1U : 0U);
        uint8_t second = (v ? 0U : 1U);
#endif
        // --- 첫 half-bit ---
        if (step_idx >= max_steps) break;
        level = first;
        steps[step_idx].bsrr    = level ? bsrr_set : bsrr_reset;
        steps[step_idx].time_us = half_bit_us;
        step_idx++;

        // --- 둘째 half-bit ---
        if (step_idx >= max_steps) break;
        level = second;
        steps[step_idx].bsrr    = level ? bsrr_set : bsrr_reset;
        steps[step_idx].time_us = half_bit_us;
        step_idx++;
    }

    return step_idx;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
uint32_t protocol_em4100_get_t5577_bitrate(int bitrate) {
    switch(bitrate) {
    case 64:
        return T5577_BITRATE_RF_64;
    case 32:
        return T5577_BITRATE_RF_32;
    case 16:
    	return T5577_BITRATE_RF_16;
    default:
        return T5577_BITRATE_RF_64;
    }
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void protocol_em4100_decoder_begin(void* proto)
{
	EM4100_Decoder_Init_Full(&g_em4100_dec);
	g_em4100_dec.detected_half_bit_us = T_256_US;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void protocol_em4100_32_decoder_begin(void* proto)
{
	EM4100_Decoder_Init_Full(&g_em4100_32_dec);
	g_em4100_32_dec.detected_half_bit_us = T_128_US;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void protocol_em4100_16_decoder_begin(void* proto)
{
	EM4100_Decoder_Init_Full(&g_em4100_16_dec);
	g_em4100_16_dec.detected_half_bit_us = T_64_US;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
bool protocol_em4100_decoder_execute(void* proto, uint16_t size)
{
	lfrfid_evt_t* p = (lfrfid_evt_t*)proto;

   //if(lfrfidProtocolManager((const lfrfid_evt_t*)p, size) != LFRFIDStateActive)
   // 	return false;

	return em4100_decoder_execute(p, size, &g_em4100_dec);
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
bool protocol_em4100_32_decoder_execute(void* proto, uint16_t size)
{
	lfrfid_evt_t* p = (lfrfid_evt_t*)proto;

    //if(lfrfidProtocolManager((const lfrfid_evt_t*)p, size) != LFRFIDStateActive)
    //	return false;

	return em4100_decoder_execute(p, size, &g_em4100_32_dec);
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
bool protocol_em4100_16_decoder_execute(void* proto, uint16_t size)
{
	lfrfid_evt_t* p = (lfrfid_evt_t*)proto;

    //if(lfrfidProtocolManager((const lfrfid_evt_t*)p, size) != LFRFIDStateActive)
    //	return false;

	return em4100_decoder_execute(p, size, &g_em4100_16_dec);
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void protocol_em4100_render_data(void* protocol, char *result)
{

	char* data = (char*)protocol_em4100_get_data(NULL);

    sprintf(
        result,
		"Hex: %02X %02X %02X %02X %02X\n"
        "FC: %03u\n"
		"Card: %05hu (RF/%02hu)",
		data[0],data[1],data[2],data[3],data[4],
        data[2],
       	MAKEWORD(data[4],data[3]),
		lfrfid_tag_info.bitrate);
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
static uint8_t* protocol_em4100_get_data(void* proto)
{
    return lfrfid_tag_info.uid;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
bool protocol_em4100_encoder_begin(void* proto)
{
	LFRFID_TAG_INFO* tag_data = (LFRFID_TAG_INFO*)proto;
	uint8_t frame[8];

	em4100_build_frame8_from_uid(frame,tag_data->uid,5);

	uint16_t half_bit_us = (tag_data->bitrate*4);

	uint16_t nsteps = em4100_build_manchester_wave(
	    frame,
		lfrfid_encoded_data.data,
	    EM4100_MAX_STEPS,
	    half_bit_us-EMUL_EM4100_CORR,
	    /* gpio_pin   */ 2,   // 예: PA2
	    /* start_level*/ 0    // 처음은 LOW에서 시작
	);

    if (nsteps == 0)
        return false; // 에러 처리
#if 0
    WaveTx_Data_t data = {
        .steps  = gEncoded_data,
        .length = nsteps,
    };
#endif
    return true;
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void protocol_em4100_encoder_send(void* proto)
{
	lfrfid_encoded_data.index = 0;
	lfrfid_encoded_data.length = 128;
	lfrfid_emul_hw_init();
#if 0
	App_WaveTx_Init();

    if (!App_WaveTx_Start())
    {
        // BUSY or ERROR
    }
#endif
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void protocol_em4100_write_begin(void* protocol, void *data)
{
	LFRFID_TAG_INFO* tag_data = (LFRFID_TAG_INFO*)protocol;
	LFRFIDProgram* write = (LFRFIDProgram*)data;
	uint8_t encoded_data[8];

	// encoder begin
	em4100_build_frame8_from_uid(encoded_data,tag_data->uid,5);

#if 1
	if(write){
		if(write->type == LFRFIDProgramTypeT5577) {
			write->t5577.block_data[0] = (T5577_MOD_MANCHESTER | protocol_em4100_get_t5577_bitrate(tag_data->bitrate) | T5577_TRANS_BL_1_2);

			bytes_to_u32_array(BIT_ORDER_MSB_FIRST, encoded_data, &write->t5577.block_data[1], 2);
			write->t5577.max_blocks = 3;
			//result = true;
		}
	}
#endif
}


/*============================================================================*/
/**
  * @brief
  * @param
  * @retval
  */
/*============================================================================*/
void protocol_em4100_write_send(void* proto)
{

	t5577_execute_write(lfrfid_program, 0);
}

