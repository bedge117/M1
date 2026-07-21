/* See COPYING.txt for license details. */

/*
*
*  m1_princeton_decode.c
*
*  M1 sub-ghz Princeton decoding
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/
#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_hal.h"
#include "bit_util.h"
#include "m1_sub_ghz_decenc.h"
#include "m1_log_debug.h"

/*************************** D E F I N E S ************************************/

#define M1_LOGDB_TAG	"SUBGHZ_PRINCETON"



//************************** C O N S T A N T **********************************/



//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/


/********************* F U N C T I O N   P R O T O T Y P E S ******************/

uint8_t subghz_decode_princeton(uint16_t p, uint16_t pulsecount);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
  * @brief
  * @param  None
  * @retval None
  */
/*============================================================================*/
uint8_t subghz_decode_princeton(uint16_t p, uint16_t pulsecount)
{
    uint32_t code;
    uint16_t te_long, te_short, tolerance_long, tolerance_short;
    uint16_t i;
    uint8_t tolerance, odd_bit, pre_bit_one, bits_count, max_bits;

    tolerance = subghz_protocols_list[p].te_tolerance;
    max_bits = subghz_protocols_list[p].data_bits;

    /* Auto-detect te_short and te_long from pulse data.
     *
     * Previous approach used only pulse_times[2] and [3], which is fragile
     * when individual pulses have significant jitter (common with low-TE
     * remotes such as Princeton PT2262 with TE≈125 µs).  Instead, scan the
     * first TE_SCAN_COUNT pulses, sort them, and average the lower and upper
     * halves to get robust estimates of te_short and te_long. */
#define TE_SCAN_COUNT 8
    {
        uint16_t scan[TE_SCAN_COUNT];
        uint16_t scan_n = (pulsecount < TE_SCAN_COUNT) ? pulsecount : TE_SCAN_COUNT;
        uint16_t a;
        int16_t  b;

        for (a = 0; a < scan_n; a++)
            scan[a] = subghz_decenc_ctl.pulse_times[a];

        /* Insertion sort (N is tiny) */
        for (a = 1; a < scan_n; a++) {
            uint16_t val = scan[a];
            b = (int16_t)a - 1;
            while (b >= 0 && scan[b] > val) {
                scan[b + 1] = scan[b];
                b--;
            }
            scan[b + 1] = val;
        }

        /* Lower half → te_short, upper half → te_long */
        uint16_t half = scan_n / 2;
        if (half == 0) half = 1; /* safety: need at least 1 in each group */
        uint32_t sum_lo = 0, sum_hi = 0;
        for (a = 0; a < half; a++)
            sum_lo += scan[a];
        for (a = half; a < scan_n; a++)
            sum_hi += scan[a];

        te_short = (uint16_t)(sum_lo / half);
        te_long  = (uint16_t)(sum_hi / (scan_n - half));
    }
#undef TE_SCAN_COUNT
    tolerance_short = te_short*3; // Assuming this is a valid long bit
    tolerance_long = (tolerance_short*tolerance)/100;
    if ( get_diff(te_long, tolerance_short) > tolerance_long ) // Not a valid bit for this protocol?
    {
    	return 1;
    }

    tolerance_short = (te_short*tolerance)/100;
    tolerance_long = (te_long*tolerance)/100;
    code = 0;
    odd_bit = 0;
    pre_bit_one = 0;
    bits_count = 0;

    for (i = 0; i<pulsecount; i++)
    {
    	if ( !odd_bit )
    	{
    		if ( get_diff(subghz_decenc_ctl.pulse_times[i], te_short) < tolerance_short )
    		{
    			pre_bit_one = 0;
    		}
    		else if ( get_diff(subghz_decenc_ctl.pulse_times[i], te_long) < tolerance_long )
    		{
    			pre_bit_one = 1;
    		}
    		else
    		{
    			break;
    		}
    		if ( i >= 2*max_bits )
    			break;
    	} // if ( !odd_bit )
    	else // odd bit
    	{
    		code <<= 1;
    		if ( pre_bit_one )
    		{
    			if ( get_diff(subghz_decenc_ctl.pulse_times[i], te_short) < tolerance_short )
    			{
    				code |= 1; // Bit 1
    			}
    			else
    			{
    				break; // Error
    			}
    		} // if ( pre_bit_one )
    		else
    		{
        		if ( get_diff(subghz_decenc_ctl.pulse_times[i], te_long) < tolerance_long )
        		{
        			; // Bit 0
        		}
        		else
        		{
        			break; // Error
        		}
    		} // else
    		bits_count++;
    	} // else
        odd_bit ^= 1;
    } // for (i = 0; i<pulsecount; i++)

    if (bits_count >= max_bits ) // Let take this packet if all bits have been decoded
    {
        subghz_decenc_ctl.n64_decodedvalue = code;
        subghz_decenc_ctl.ndecodedbitlength = bits_count;
        /* Store the detected te_short (in µs) as the signal TE.  Princeton
         * remotes vary widely in TE (≈100–500 µs depending on the Rt resistor
         * on the PT2262/SC5262 encoder), so the actually-observed timing is
         * required for accurate replay and for display parity with Flipper. */
        subghz_decenc_ctl.ndecodeddelay = te_short;
        subghz_decenc_ctl.ndecodedprotocol = p;
        return 0;
    }
    else
    {
        M1_LOG_E(M1_LOGDB_TAG, "E: rx:%d, decoded:%d, te[i] %d\r\n", pulsecount, bits_count, subghz_decenc_ctl.pulse_times[i]);
    	for (i=0; i<10; i++)
    		M1_LOG_N(M1_LOGDB_TAG, "%d ", subghz_decenc_ctl.pulse_times[i]);
    	M1_LOG_N(M1_LOGDB_TAG, "\r\n");
    }

    return 1;
} // uint8_t subghz_decode_princeton(uint16_t p, uint16_t pulsecount)

