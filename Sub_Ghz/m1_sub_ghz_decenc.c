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
#include "m1_sub_ghz.h"
#include "m1_sub_ghz_api.h"
#include "si446x_cmd.h"
#include "m1_io_defs.h" // Test only
#include "m1_log_debug.h"

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG	"SUBGHZ_DECENC"

//************************** C O N S T A N T **********************************/

const SubGHz_protocol_t subghz_protocols_list[] =
{
	/*{160, 470, PACKET_PULSE_TIME_TOLERANCE20, 0, 24}, // Princeton: bit 0 |^|___, bit 1 |^^^|_*/
	{370, 1140, PACKET_PULSE_TIME_TOLERANCE20, 0, 24},  // Princeton: bit 0 |^|___, bit 1 |^^^|_
	{250, 500, PACKET_PULSE_TIME_TOLERANCE20, 16, 46},  // Security+ 2.0
	{320, 640, PACKET_PULSE_TIME_TOLERANCE20, 0, 12},   // CAME 12-bit
	{700, 1400, PACKET_PULSE_TIME_TOLERANCE20, 0, 12},  // Nice FLO 12-bit
	{500, 1500, PACKET_PULSE_TIME_TOLERANCE25, 0, 10},  // Linear 10-bit
	{340, 1020, PACKET_PULSE_TIME_TOLERANCE20, 0, 12},  // Holtek HT12E
	{400, 800, PACKET_PULSE_TIME_TOLERANCE20, 0, 66},   // KeeLoq
	{488, 976, PACKET_PULSE_TIME_TOLERANCE25, 24, 64},  // Oregon Scientific v2.1 (Manchester)
	{200, 600, PACKET_PULSE_TIME_TOLERANCE25, 4, 56},   // Acurite
	{550, 1100, PACKET_PULSE_TIME_TOLERANCE25, 0, 44},  // LaCrosse TX
	{255, 510, PACKET_PULSE_TIME_TOLERANCE20, 0, 64},   // FAAC SLH (Manchester)
	{500, 1000, PACKET_PULSE_TIME_TOLERANCE20, 0, 44},  // Hormann BiSecur
	{800, 1600, PACKET_PULSE_TIME_TOLERANCE20, 0, 12},  // Marantec
	{640, 1280, PACKET_PULSE_TIME_TOLERANCE25, 4, 56},  // Somfy Telis (Manchester)
	{400, 800, PACKET_PULSE_TIME_TOLERANCE20, 0, 64},   // Star Line (KeeLoq variant)
	{350, 700, PACKET_PULSE_TIME_TOLERANCE20, 0, 24},   // Gate TX
	{300, 900, PACKET_PULSE_TIME_TOLERANCE25, 0, 25},   // SMC5326
	{225, 675, PACKET_PULSE_TIME_TOLERANCE25, 0, 16},   // Power Smart
	{450, 1350, PACKET_PULSE_TIME_TOLERANCE20, 0, 48},  // iDo
	{555, 1110, PACKET_PULSE_TIME_TOLERANCE20, 0, 12},  // Ansonic
	{500, 1500, PACKET_PULSE_TIME_TOLERANCE25, 4, 40},  // Infactory (weather)
	{120, 240, PACKET_PULSE_TIME_TOLERANCE25, 8, 40},   // Schrader TPMS (Manchester)
	/* --- New protocols --- */
	{1000, 3000, PACKET_PULSE_TIME_TOLERANCE20, 0, 40}, // Chamberlain Security+ 1.0
	{385, 1155, PACKET_PULSE_TIME_TOLERANCE20, 0, 18},  // Clemsa
	{450, 900, PACKET_PULSE_TIME_TOLERANCE20, 0, 37},   // Doitrand
	{340, 680, PACKET_PULSE_TIME_TOLERANCE20, 0, 18},   // BETT
	{330, 990, PACKET_PULSE_TIME_TOLERANCE20, 0, 36},   // Nero Sketch/Radio
	{300, 900, PACKET_PULSE_TIME_TOLERANCE20, 0, 10},   // FireFly
	{260, 520, PACKET_PULSE_TIME_TOLERANCE20, 0, 54},   // CAME Twee
	{200, 400, PACKET_PULSE_TIME_TOLERANCE20, 0, 62},   // CAME Atomo (rolling code)
	{500, 1000, PACKET_PULSE_TIME_TOLERANCE20, 0, 52},  // Nice Flor S
	{400, 800, PACKET_PULSE_TIME_TOLERANCE20, 0, 72},   // Alutech AT-4N
	{336, 672, PACKET_PULSE_TIME_TOLERANCE20, 0, 24},   // Centurion
	{400, 1200, PACKET_PULSE_TIME_TOLERANCE20, 0, 60},  // Kinggates Stylo 4K
	{1000, 2000, PACKET_PULSE_TIME_TOLERANCE20, 0, 24}, // Megacode
	{500, 1500, PACKET_PULSE_TIME_TOLERANCE20, 0, 36},  // Mastercode
	{2000, 6000, PACKET_PULSE_TIME_TOLERANCE25, 0, 7},  // Chamberlain 7-bit
	{2000, 6000, PACKET_PULSE_TIME_TOLERANCE25, 0, 8},  // Chamberlain 8-bit
	{2000, 6000, PACKET_PULSE_TIME_TOLERANCE25, 0, 9},  // Chamberlain 9-bit
	{1000, 3000, PACKET_PULSE_TIME_TOLERANCE25, 0, 10}, // Liftmaster 10-bit
	{400, 1200, PACKET_PULSE_TIME_TOLERANCE20, 0, 40},  // Dooya
	{250, 750, PACKET_PULSE_TIME_TOLERANCE25, 0, 48},   // Honeywell
	{250, 750, PACKET_PULSE_TIME_TOLERANCE25, 0, 32},   // Intertechno
	{330, 990, PACKET_PULSE_TIME_TOLERANCE25, 0, 32},   // Elro
	{500, 1000, PACKET_PULSE_TIME_TOLERANCE25, 0, 40},  // Ambient Weather (Manchester)
	{250, 500, PACKET_PULSE_TIME_TOLERANCE25, 0, 40},   // Bresser 3ch
	{250, 500, PACKET_PULSE_TIME_TOLERANCE25, 0, 56},   // Bresser 5in1
	{250, 500, PACKET_PULSE_TIME_TOLERANCE25, 0, 104},  // Bresser 6in1
	{500, 1000, PACKET_PULSE_TIME_TOLERANCE25, 0, 48},  // TFA Dostmann
	{500, 1000, PACKET_PULSE_TIME_TOLERANCE25, 0, 36},  // Nexus-TH
	{250, 500, PACKET_PULSE_TIME_TOLERANCE25, 0, 37},   // ThermoPro TX-2
	{500, 1000, PACKET_PULSE_TIME_TOLERANCE25, 0, 40},  // GT-WT03
	{400, 800, PACKET_PULSE_TIME_TOLERANCE20, 0, 64},   // Scher-Khan Magicar
	{400, 1200, PACKET_PULSE_TIME_TOLERANCE20, 0, 64},  // Scher-Khan Logicar
	{250, 750, PACKET_PULSE_TIME_TOLERANCE20, 0, 56},   // Toyota
	{100, 300, PACKET_PULSE_TIME_TOLERANCE30, 0, 64},   // BinRAW (generic fallback)
};

const char *protocol_text[] =
{
	"Princeton",
	"Security+ 2.0",
	"CAME",
	"Nice FLO",
	"Linear",
	"Holtek",
	"KeeLoq",
	"Oregon v2",
	"Acurite",
	"LaCrosse TX",
	"FAAC SLH",
	"Hormann",
	"Marantec",
	"Somfy Telis",
	"Star Line",
	"Gate TX",
	"SMC5326",
	"Power Smart",
	"iDo",
	"Ansonic",
	"Infactory",
	"Schrader TPMS",
	/* --- New protocols --- */
	"Chamberlain",
	"Clemsa",
	"Doitrand",
	"BETT",
	"Nero Radio",
	"FireFly",
	"CAME Twee",
	"CAME Atomo",
	"Nice Flor S",
	"Alutech AT-4N",
	"Centurion",
	"Kinggates Stylo",
	"Megacode",
	"Mastercode",
	"Chamberlain 7",
	"Chamberlain 8",
	"Chamberlain 9",
	"Liftmaster",
	"Dooya",
	"Honeywell",
	"Intertechno",
	"Elro",
	"Ambient Weather",
	"Bresser 3ch",
	"Bresser 5in1",
	"Bresser 6in1",
	"TFA Dostmann",
	"Nexus-TH",
	"ThermoPro TX-2",
	"GT-WT03",
	"Scher-Khan Magicar",
	"Scher-Khan Logicar",
	"Toyota",
	"BinRAW"
};


enum {
   n_protocol = sizeof(subghz_protocols_list) / sizeof(subghz_protocols_list[0])
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

bool subghz_decode_protocol(uint16_t p, uint16_t pulsecount)
{
    uint8_t ret = false;

    switch ( p )
    {
    	case PRINCETON:
    		ret = subghz_decode_princeton(p, pulsecount);
    		break;

    	case SECURITY_PLUS_20:
    		ret = subghz_decode_security_plus_20(p, pulsecount);
    		break;

    	case CAME_12BIT:
    		ret = subghz_decode_came(p, pulsecount);
    		break;

    	case NICE_FLO:
    		ret = subghz_decode_nice_flo(p, pulsecount);
    		break;

    	case LINEAR_10BIT:
    		ret = subghz_decode_linear(p, pulsecount);
    		break;

    	case HOLTEK_HT12E:
    		ret = subghz_decode_holtek(p, pulsecount);
    		break;

    	case KEELOQ:
    		ret = subghz_decode_keeloq(p, pulsecount);
    		break;

    	case OREGON_V2:
    		ret = subghz_decode_oregon_v2(p, pulsecount);
    		break;

    	case ACURITE:
    		ret = subghz_decode_acurite(p, pulsecount);
    		break;

    	case LACROSSE_TX:
    		ret = subghz_decode_lacrosse_tx(p, pulsecount);
    		break;

    	case FAAC_SLH:
    		ret = subghz_decode_faac_slh(p, pulsecount);
    		break;

    	case HORMANN:
    		ret = subghz_decode_hormann(p, pulsecount);
    		break;

    	case MARANTEC:
    		ret = subghz_decode_marantec(p, pulsecount);
    		break;

    	case SOMFY_TELIS:
    		ret = subghz_decode_somfy_telis(p, pulsecount);
    		break;

    	case STAR_LINE:
    		ret = subghz_decode_starline(p, pulsecount);
    		break;

    	case GATE_TX:
    		ret = subghz_decode_gate_tx(p, pulsecount);
    		break;

    	case SMC5326:
    		ret = subghz_decode_smc5326(p, pulsecount);
    		break;

    	case POWER_SMART:
    		ret = subghz_decode_power_smart(p, pulsecount);
    		break;

    	case IDO:
    		ret = subghz_decode_ido(p, pulsecount);
    		break;

    	case ANSONIC:
    		ret = subghz_decode_ansonic(p, pulsecount);
    		break;

    	case INFACTORY:
    		ret = subghz_decode_infactory(p, pulsecount);
    		break;

    	case SCHRADER_TPMS:
    		ret = subghz_decode_schrader(p, pulsecount);
    		break;

    	/* --- New protocols --- */
    	case CHAMBERLAIN:
    		ret = subghz_decode_chamberlain(p, pulsecount);
    		break;

    	case CLEMSA:
    		ret = subghz_decode_clemsa(p, pulsecount);
    		break;

    	case DOITRAND:
    		ret = subghz_decode_doitrand(p, pulsecount);
    		break;

    	case BETT:
    		ret = subghz_decode_bett(p, pulsecount);
    		break;

    	case NERO_RADIO:
    		ret = subghz_decode_nero(p, pulsecount);
    		break;

    	case FIREFLY:
    		ret = subghz_decode_firefly(p, pulsecount);
    		break;

    	case CAME_TWEE:
    		ret = subghz_decode_came_twee(p, pulsecount);
    		break;

    	case CAME_ATOMO:
    		ret = subghz_decode_came_atomo(p, pulsecount);
    		break;

    	case NICE_FLOR_S:
    		ret = subghz_decode_nice_flor_s(p, pulsecount);
    		break;

    	case ALUTECH_AT4N:
    		ret = subghz_decode_alutech(p, pulsecount);
    		break;

    	case CENTURION:
    		ret = subghz_decode_centurion(p, pulsecount);
    		break;

    	case KINGGATES_STYLO:
    		ret = subghz_decode_kinggates(p, pulsecount);
    		break;

    	case MEGACODE:
    		ret = subghz_decode_megacode(p, pulsecount);
    		break;

    	case MASTERCODE:
    		ret = subghz_decode_mastercode(p, pulsecount);
    		break;

    	case CHAMBERLAIN_7BIT:
    		ret = subghz_decode_chamberlain_7bit(p, pulsecount);
    		break;

    	case CHAMBERLAIN_8BIT:
    		ret = subghz_decode_chamberlain_8bit(p, pulsecount);
    		break;

    	case CHAMBERLAIN_9BIT:
    		ret = subghz_decode_chamberlain_9bit(p, pulsecount);
    		break;

    	case LIFTMASTER_10BIT:
    		ret = subghz_decode_liftmaster(p, pulsecount);
    		break;

    	case DOOYA:
    		ret = subghz_decode_dooya(p, pulsecount);
    		break;

    	case HONEYWELL:
    		ret = subghz_decode_honeywell(p, pulsecount);
    		break;

    	case INTERTECHNO:
    		ret = subghz_decode_intertechno(p, pulsecount);
    		break;

    	case ELRO:
    		ret = subghz_decode_elro(p, pulsecount);
    		break;

    	case AMBIENT_WEATHER:
    		ret = subghz_decode_ambient_weather(p, pulsecount);
    		break;

    	case BRESSER_3CH:
    		ret = subghz_decode_bresser_3ch(p, pulsecount);
    		break;

    	case BRESSER_5IN1:
    		ret = subghz_decode_bresser_5in1(p, pulsecount);
    		break;

    	case BRESSER_6IN1:
    		ret = subghz_decode_bresser_6in1(p, pulsecount);
    		break;

    	case TFA_DOSTMANN:
    		ret = subghz_decode_tfa_dostmann(p, pulsecount);
    		break;

    	case NEXUS_TH:
    		ret = subghz_decode_nexus_th(p, pulsecount);
    		break;

    	case THERMOPRO_TX2:
    		ret = subghz_decode_thermopro_tx2(p, pulsecount);
    		break;

    	case GT_WT03:
    		ret = subghz_decode_gt_wt03(p, pulsecount);
    		break;

    	case SCHER_KHAN_MAGICAR:
    		ret = subghz_decode_scher_khan_magicar(p, pulsecount);
    		break;

    	case SCHER_KHAN_LOGICAR:
    		ret = subghz_decode_scher_khan_logicar(p, pulsecount);
    		break;

    	case TOYOTA:
    		ret = subghz_decode_toyota(p, pulsecount);
    		break;

    	case BIN_RAW:
    		ret = subghz_decode_bin_raw(p, pulsecount);
    		break;

    	default:
    		break;
    } // switch ( p )

    return ret;
} // bool subghz_decode_protocol(uint16_t p, uint16_t subghz_decenc_ctl.npulsecount)


/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
uint8_t subghz_pulse_handler(uint16_t duration)
{
	  static uint32_t interpacket_gap = 0;
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
				  for(i = 0; i < n_protocol; i++)
				  {
					  if ( !subghz_decode_protocol(i, subghz_decenc_ctl.npulsecount) )
					  {
						  // receive successfully for protocol i
						  break;
					  }
				  } // for(i = 0; i < n_protocol; i++)
			  } // if ( subghz_decenc_ctl.npulsecount >= PACKET_PULSE_COUNT_MIN )
			  interpacket_gap = duration; // update
			  subghz_decenc_ctl.npulsecount = 0;
			  // A potential interpacket gap has been detected, so it's not required to check for this condition for the next packet, if any.
			  return PULSE_DET_EOP; // error or end of packet has been met
		  } // if (duration >= INTERPACKET_GAP_MIN)
	  } // if (duration >= PACKET_PULSE_TIME_MIN)
	  else
	  {
		  subghz_decenc_ctl.npulsecount = 0; // reset
		  interpacket_gap += duration;
		  // Interpacket gap has been timeout for a potential packet
		  if ( interpacket_gap > INTERPACKET_GAP_MAX )
		  {
			  interpacket_gap = 0; // reset
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
} // void subghz_decenc_init(void)
