/* See COPYING.txt for license details. */

/*
*
*  m1_sub_ghz_decenc.c
*
*  M1 sub-ghz decoding encoding
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "stm32h5xx_hal.h"
#include <m1_sub_ghz_decenc.h>
#include "subghz_protocol_registry.h"
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_api.h"
#include "si446x_cmd.h"
#include "m1_io_defs.h" // Test only
#include "m1_log_debug.h"

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG	"SUBGHZ_DECENC"

//************************** C O N S T A N T **********************************/

/*
 * Static legacy arrays — kept for backward compatibility with existing code
 * that indexes subghz_protocols_list[p] and protocol_text[p] directly.
 *
 * IMPORTANT: These arrays are now GENERATED from the master registry in
 * subghz_protocol_registry.c.  To add a new protocol, edit ONLY the
 * registry — these arrays are rebuilt at startup by subghz_decenc_init().
 *
 * NOTE: These arrays are populated once during subghz_decenc_init() and are
 * only accessed from the Sub-GHz decode task (single-threaded).  They are NOT
 * safe for concurrent access from ISRs or other RTOS tasks.
 */

/* Max protocols we can hold in the legacy arrays (must be >= registry count) */
#define LEGACY_PROTOCOL_MAX  128

static SubGHz_protocol_t _subghz_protocols_list_storage[LEGACY_PROTOCOL_MAX];
static const char *_protocol_text_storage[LEGACY_PROTOCOL_MAX];

/* Expose as non-const pointers so existing code can index them */
SubGHz_protocol_t *subghz_protocols_list_ptr = _subghz_protocols_list_storage;
const char **protocol_text_ptr = _protocol_text_storage;

/* Legacy externs — redirect to pointer (see updated header) */
/* These are populated during subghz_decenc_init() */

/*
 * protocol_text[] — now populated from registry at init time.
 * The static storage is in _protocol_text_storage above.
 * Legacy code accesses protocol_text[i] via the extern pointer.
 */


enum {
   n_protocol = LEGACY_PROTOCOL_MAX   /* Upper bound; actual count is subghz_protocol_registry_count */
};


//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/

SubGHz_DecEnc_t subghz_decenc_ctl;
SubGHz_Weather_Data_t weather_data;

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

inline uint16_t get_diff(uint16_t n_a, uint16_t n_b);
uint8_t subghz_pulse_handler(uint16_t duration);
static bool subghz_decode_protocol(uint16_t p, uint16_t pulsecount);
bool subghz_decenc_read(SubGHz_Dec_Info_t *received, bool raw);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
inline uint16_t get_diff(uint16_t n_a, uint16_t n_b)
{
	return abs(n_a - n_b);
}



/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
bool subghz_data_ready()
{
  return (subghz_decenc_ctl.n64_decodedvalue != 0);
}


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
bool subghz_raw_data_ready()
{
  return (subghz_decenc_ctl.pulse_times[0] != 0);
}


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
void subghz_reset_data()
{
	subghz_decenc_ctl.n64_decodedvalue = 0;
	subghz_decenc_ctl.ndecodedbitlength = 0;
	subghz_decenc_ctl.n32_serialnumber = 0;
	subghz_decenc_ctl.n32_rollingcode = 0;
	subghz_decenc_ctl.n8_buttonid = 0;
	memset(subghz_decenc_ctl.pulse_times, 0, sizeof(subghz_decenc_ctl.pulse_times));
}


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
uint64_t subghz_get_decoded_value()
{
	return subghz_decenc_ctl.n64_decodedvalue;
}


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
uint16_t subghz_get_decoded_bitlength()
{
	return subghz_decenc_ctl.ndecodedbitlength;
}


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
uint16_t subghz_get_decoded_delay()
{
	return subghz_decenc_ctl.ndecodeddelay;
}


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
uint16_t subghz_get_decoded_protocol()
{
	return subghz_decenc_ctl.ndecodedprotocol;
}


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
int16_t subghz_get_decoded_rssi()
{
	return subghz_decenc_ctl.ndecodedrssi;
}


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
uint16_t *subghz_get_rawdata()
{
	return subghz_decenc_ctl.pulse_times;
}



/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
const SubGHz_Weather_Data_t* subghz_get_weather_data(void)
{
    return &weather_data;
}

/*
 * Protocol dispatch — now driven by the registry instead of a switch-case.
 *
 * The registry stores a decode function pointer for each protocol.
 * This eliminates the need to add a case for every new protocol.
 */
static bool subghz_decode_protocol(uint16_t p, uint16_t pulsecount)
{
    if (p >= subghz_protocol_registry_count)
        return 1;  /* out of range */

    const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
    if (!proto->decode)
        return 1;  /* no decoder implemented */

    return proto->decode(p, pulsecount);
}


/* File-level static so subghz_pulse_handler_reset() can clear it */
static uint32_t pulse_handler_interpacket_gap = 0;

/*============================================================================*/
/**
  * @brief  Reset the pulse handler's accumulated state.
  *         Call before (re)starting RX to ensure a clean decode session.
  */
/*============================================================================*/
void subghz_pulse_handler_reset(void)
{
    pulse_handler_interpacket_gap = 0;
    subghz_decenc_ctl.npulsecount = 0;
    subghz_decenc_ctl.n64_decodedvalue = 0;
    subghz_decenc_ctl.ndecodedbitlength = 0;
    subghz_decenc_ctl.ndecodedprotocol = 0;
    subghz_decenc_ctl.n32_serialnumber = 0;
    subghz_decenc_ctl.n32_rollingcode = 0;
    subghz_decenc_ctl.n8_buttonid = 0;
}

/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
uint8_t subghz_pulse_handler(uint16_t duration)
{
	  uint8_t i;
	  int16_t rssi;
	  struct si446x_reply_GET_MODEM_STATUS_map *pmodemstat;

	  if (duration >= PACKET_PULSE_TIME_MIN)
	  {
		  if (duration >= INTERPACKET_GAP_MIN) // Possible gap between packets?
		  {
			  subghz_decenc_ctl.pulse_times[subghz_decenc_ctl.npulsecount++] = duration; // End bit

			  M1_LOG_D(M1_LOGDB_TAG, "Valid gap: %d, pulses:%d\r\n", duration, subghz_decenc_ctl.npulsecount);
			  if ( subghz_decenc_ctl.npulsecount >= PACKET_PULSE_COUNT_MIN ) // Potential packet received?
			  {
				  for(i = 0; i < subghz_protocol_registry_count; i++)
				  {
					  if ( !subghz_decode_protocol(i, subghz_decenc_ctl.npulsecount) )
					  {
						  // receive successfully for protocol i
						  break;
					  }
				  } // for(i = 0; i < subghz_protocol_registry_count; i++)
			  } // if ( subghz_decenc_ctl.npulsecount >= PACKET_PULSE_COUNT_MIN )
			  pulse_handler_interpacket_gap = duration; // update
			  subghz_decenc_ctl.npulsecount = 0;
			  // A potential interpacket gap has been detected, so it's not required to check for this condition for the next packet, if any.
			  return PULSE_DET_EOP; // error or end of packet has been met
		  } // if (duration >= INTERPACKET_GAP_MIN)
	  } // if (duration >= PACKET_PULSE_TIME_MIN)
	  else
	  {
		  subghz_decenc_ctl.npulsecount = 0; // reset
		  pulse_handler_interpacket_gap += duration;
		  // Interpacket gap has been timeout for a potential packet
		  if ( pulse_handler_interpacket_gap > INTERPACKET_GAP_MAX )
		  {
			  pulse_handler_interpacket_gap = 0; // reset
			  return PULSE_DET_IDLE; // error
		  }
		  else
		  {
			  return PULSE_DET_NORMAL;
		  }
	  } // else
	  // detect overflow
	  if (subghz_decenc_ctl.npulsecount >= PACKET_PULSE_COUNT_MAX)
	  {
		  subghz_decenc_ctl.npulsecount = 0; // Reset rx buffer
		  return PULSE_DET_IDLE; // error
	  }
	  subghz_decenc_ctl.pulse_times[subghz_decenc_ctl.npulsecount++] = duration;
	  // Read RSSI when half of this potential packet has been received
	  if ( subghz_decenc_ctl.npulsecount==PACKET_PULSE_COUNT_MIN/2 )
	  {
		  // Read INTs, clear pending ones
		  SI446x_Get_IntStatus(0, 0, 0);
		  pmodemstat = SI446x_Get_ModemStatus(0x00);
		  // RF_Input_Level_dBm = (RSSI_value / 2) – MODEM_RSSI_COMP – 70
		  rssi = pmodemstat->CURR_RSSI/2 - MODEM_RSSI_COMP - 70;
		  subghz_decenc_ctl.ndecodedrssi = rssi;
	  } // if ( subghz_decenc_ctl.npulsecount==PACKET_PULSE_COUNT_MIN/2 )

	  return PULSE_DET_NORMAL;
} // uint8_t subghz_pulse_handler(uint16_t duration)



/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
bool subghz_decenc_read(SubGHz_Dec_Info_t *received, bool raw)
{
    bool ret = false;

	if ( subghz_decenc_ctl.subghz_data_ready() )
    {
        uint64_t value = subghz_decenc_ctl.subghz_get_decoded_value();
        if (value)
        {
            received->frequency = 0;
            received->key = subghz_decenc_ctl.subghz_get_decoded_value();
            received->protocol = subghz_decenc_ctl.subghz_get_decoded_protocol();
            received->rssi = subghz_decenc_ctl.subghz_get_decoded_rssi();
            received->te = subghz_decenc_ctl.subghz_get_decoded_delay();
            received->bit_len = subghz_decenc_ctl.subghz_get_decoded_bitlength();
            /* Fallback: if the protocol decoder did not record a per-capture TE
             * (most non-Princeton decoders leave ndecodeddelay = 0 and rely on
             * the fixed registry te_short), populate it from the protocol
             * registry so the Read/Info display, saved-file `TE:` field, and
             * Flipper-compatible export all show a non-zero, meaningful value.
             * Replay of decoded static entries uses the same fallback in
             * subghz_transmit_static_signal() when resolving te_short for TX,
             * so this change does not alter TX timing — it only fixes
             * display/save parity with Flipper for protocols whose decoders
             * never populated TE. */
            if (received->te == 0 && received->protocol < LEGACY_PROTOCOL_MAX)
            {
                received->te = subghz_protocols_list[received->protocol].te_short;
            }
            /* Copy extended fields set by protocol-specific decoders */
            received->serial_number = subghz_decenc_ctl.n32_serialnumber;
            received->rolling_code  = subghz_decenc_ctl.n32_rollingcode;
            received->button_id     = subghz_decenc_ctl.n8_buttonid;
        } // if (value)
        subghz_decenc_ctl.subghz_reset_data();
        ret = true;
    } // if ( subghz_decenc_ctl.subghz_data_ready() )

    if (raw && subghz_decenc_ctl.subghz_raw_data_ready())
    {
    	received->raw = true;
    	received->raw_data = subghz_decenc_ctl.subghz_get_rawdata();
        subghz_decenc_ctl.subghz_reset_data();
        ret = true;
    } // if (raw && subghz_decenc_ctl.subghz_raw_data_ready())

    return ret;
} // bool subghz_decenc_read(bool raw)


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
void subghz_decenc_init(void)
{
    subghz_decenc_ctl.subghz_data_ready = subghz_data_ready;
    subghz_decenc_ctl.subghz_raw_data_ready = subghz_raw_data_ready;
    subghz_decenc_ctl.subghz_reset_data = subghz_reset_data;

    subghz_decenc_ctl.subghz_get_decoded_value = subghz_get_decoded_value;
    subghz_decenc_ctl.subghz_get_decoded_bitlength = subghz_get_decoded_bitlength;
    subghz_decenc_ctl.subghz_get_decoded_delay = subghz_get_decoded_delay;
    subghz_decenc_ctl.subghz_get_decoded_protocol = subghz_get_decoded_protocol;
    subghz_decenc_ctl.subghz_get_decoded_rssi = subghz_get_decoded_rssi;
    subghz_decenc_ctl.subghz_get_rawdata = subghz_get_rawdata;
    subghz_decenc_ctl.subghz_pulse_handler = subghz_pulse_handler;

	subghz_decenc_ctl.n64_decodedvalue = 0;
	subghz_decenc_ctl.ndecodedbitlength = 0;
	subghz_decenc_ctl.ndecodedrssi = 0;
	subghz_decenc_ctl.ndecodeddelay = 0;
	subghz_decenc_ctl.ndecodedprotocol = 0;
	subghz_decenc_ctl.npulsecount = 0;
	subghz_decenc_ctl.pulse_det_stat = PULSE_DET_IDLE;
	memset(subghz_decenc_ctl.pulse_times, 0, sizeof(subghz_decenc_ctl.pulse_times));
	subghz_decenc_ctl.n64_decodedvalue = 0;

	/* Populate legacy compatibility arrays from the protocol registry */
	uint16_t count = subghz_protocol_registry_count;
	if (count > LEGACY_PROTOCOL_MAX) {
	    M1_LOG_D(M1_LOGDB_TAG, "WARN: registry (%u protos) exceeds legacy max (%u), truncated\r\n",
	             count, LEGACY_PROTOCOL_MAX);
	    count = LEGACY_PROTOCOL_MAX;
	}
	for (uint16_t i = 0; i < count; i++) {
	    const SubGhzBlockConst *t = &subghz_protocol_registry[i].timing;
	    _subghz_protocols_list_storage[i].te_short      = t->te_short;
	    _subghz_protocols_list_storage[i].te_long        = t->te_long;
	    _subghz_protocols_list_storage[i].te_tolerance   = t->te_tolerance_pct;
	    _subghz_protocols_list_storage[i].preamble_bits  = t->preamble_bits;
	    _subghz_protocols_list_storage[i].data_bits      = t->min_count_bit_for_found;
	    _protocol_text_storage[i] = subghz_protocol_registry[i].name;
	}
} // void subghz_decenc_init(void)


/*============================================================================*/
/* Signal History Ring Buffer Implementation                                   */
/*============================================================================*/

void subghz_history_reset(SubGHz_History_t *hist)
{
	hist->count = 0;
	hist->head  = 0;
}

uint8_t subghz_history_add(SubGHz_History_t *hist, const SubGHz_Dec_Info_t *info, uint32_t freq_hz)
{
	/* Default semantics: dedup against last entry, evict oldest on overflow. */
	return subghz_history_add_ex(hist, info, freq_hz, true, true);
}

uint8_t subghz_history_add_ex(SubGHz_History_t *hist, const SubGHz_Dec_Info_t *info,
                              uint32_t freq_hz,
                              bool remove_duplicates, bool delete_old_signals)
{
	/* Phase 12: when remove_duplicates is true, check against the most
	 * recent entry on (protocol, key, bit_len) and merge instead of
	 * creating a new row.  When false, every reception adds a new entry. */
	if (remove_duplicates && hist->count > 0)
	{
		uint8_t last_idx = (hist->head == 0) ? SUBGHZ_HISTORY_MAX - 1 : hist->head - 1;
		SubGHz_History_Entry_t *last = &hist->entries[last_idx];
		if (last->info.protocol == info->protocol &&
		    last->info.key      == info->key &&
		    last->info.bit_len  == info->bit_len)
		{
			/* Duplicate — increment count, update RSSI to latest */
			if (last->count < 255)
				last->count++;
			last->info.rssi = info->rssi;
			return hist->count;
		}
	}

	/* Phase 12: when delete_old_signals is false and the ring is full,
	 * drop the new entry instead of evicting the oldest. */
	if (!delete_old_signals && hist->count >= SUBGHZ_HISTORY_MAX)
		return hist->count;

	/* Write new entry at head position */
	SubGHz_History_Entry_t *entry = &hist->entries[hist->head];
	entry->info          = *info;
	entry->info.raw_data = NULL;   /* Do not store raw data pointer */
	entry->info.raw      = false;
	entry->frequency     = freq_hz;
	entry->count         = 1;

	hist->head = (hist->head + 1) % SUBGHZ_HISTORY_MAX;
	if (hist->count < SUBGHZ_HISTORY_MAX)
		hist->count++;

	return hist->count;
}

const SubGHz_History_Entry_t *subghz_history_get(const SubGHz_History_t *hist, uint8_t idx)
{
	if (idx >= hist->count)
		return NULL;
	/* Index 0 = most recent, index (count-1) = oldest */
	uint8_t pos;
	if (hist->head >= (idx + 1))
		pos = hist->head - idx - 1;
	else
		pos = SUBGHZ_HISTORY_MAX - (idx + 1 - hist->head);
	return &hist->entries[pos];
}
