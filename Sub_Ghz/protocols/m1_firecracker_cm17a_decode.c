/* See COPYING.txt for license details. */
/*
 * m1_firecracker_cm17a_decode.c
 *
 * FireCracker / CM17A home-automation decoder.
 *
 * The CM17A ("FireCracker") is an X10-compatible RF transmitter controlled
 * via an RS-232 serial port (RTS/DTR signals).  Its over-the-air packets use
 * the same OOK physical layer as standard X10 RF but with a 40-bit frame
 * instead of the 32-bit complementary-pair format:
 *
 *   [16-bit header: 0xD5AA] [16-bit data] [8-bit footer: 0xAD]
 *
 * Physical layer (identical to X10 RF):
 *   Frequency  : 310 MHz (North America) / 433.92 MHz (Europe)
 *   Modulation : OOK AM
 *   Preamble   : ~9 ms HIGH, ~4.5 ms LOW
 *   Bit '1'    : ~562 µs HIGH, ~1688 µs LOW
 *   Bit '0'    : ~562 µs HIGH, ~562 µs LOW
 *
 * Data byte layout (from the CM17A communications specification):
 *   house_byte bits[7:4] — house code (A-P, not a simple index)
 *   house_byte bit[2]    — 0 = units 1-8; 1 = units 9-16
 *   cmd_byte bit[7]      — 1 = DIM/BRIGHT (special), 0 = ON/OFF/unit
 *   cmd_byte bit[5]      — 1 = OFF (when bit 7 is 0)
 *   cmd_byte bits[6,4,3] — unit code within the group
 *
 * Reference: https://github.com/evilpete/flipper_toolbox/raw/refs/heads/main/subghz/firecracker_spec.txt
 */

#include "m1_sub_ghz_decenc.h"
#include "subghz_protocol_registry.h"

/* CM17A packet constants */
#define CM17A_HEADER  0xD5AAU  /* 16-bit fixed header  */
#define CM17A_FOOTER  0xADU    /* 8-bit  fixed footer  */

#define CM17A_TOTAL_BITS   40u
#define CM17A_HEADER_BITS  16u
#define CM17A_DATA_BITS    16u
#define CM17A_FOOTER_BITS  8u

/*
 * House-code lookup table.
 *
 * Maps house_byte[7:4] (4 bits, values 0-15) → house-code letter index (A-P).
 * The CM17A house code is NOT a simple binary index — the bit pattern is
 * deliberately non-sequential (the same encoding as X10 powerline).
 *
 * Index = (house_byte >> 4) & 0x0F
 * Value = 0-based house-code letter (0='A', 1='B', …, 15='P')
 *         or 0xFF if the nibble is not a valid house code.
 */
static const uint8_t cm17a_house_nibble_to_letter[16] = {
    /*  nibble : house letter (0=A…15=P) */
    /* 0x0 */ 12,  /* M */
    /* 0x1 */ 13,  /* N */
    /* 0x2 */ 14,  /* O */
    /* 0x3 */ 15,  /* P */
    /* 0x4 */  2,  /* C */
    /* 0x5 */  3,  /* D */
    /* 0x6 */  0,  /* A */
    /* 0x7 */  1,  /* B */
    /* 0x8 */  4,  /* E */
    /* 0x9 */  5,  /* F */
    /* 0xA */  6,  /* G */
    /* 0xB */  7,  /* H */
    /* 0xC */ 10,  /* K */
    /* 0xD */ 11,  /* L */
    /* 0xE */  8,  /* I */
    /* 0xF */  9,  /* J */
};

/*
 * Unit-code lookup table.
 *
 * Maps cmd_byte bits [6,4,3] (3 bits) → unit offset within the group (0-7).
 * Add 1 for the 1-based unit number.
 * If house_byte bit 2 is set, add 8 for units 9-16.
 *
 * The 3-bit index is formed as: ((cmd_byte >> 4) & 0x04) |
 *                                ((cmd_byte >> 3) & 0x03)
 * (bit 6 → result bit 2, bit 4 → result bit 1, bit 3 → result bit 0)
 */
static const uint8_t cm17a_unit_code_to_offset[8] = {
    /* unit bits [6,4,3] → unit offset (0-based) */
    /* 0b000 */ 0,  /* unit 1 */
    /* 0b001 */ 2,  /* unit 3 */
    /* 0b010 */ 1,  /* unit 2 */
    /* 0b011 */ 3,  /* unit 4 */
    /* 0b100 */ 4,  /* unit 5 */
    /* 0b101 */ 6,  /* unit 7 */
    /* 0b110 */ 5,  /* unit 6 */
    /* 0b111 */ 7,  /* unit 8 */
};

uint8_t subghz_decode_firecracker_cm17a(uint16_t p, uint16_t pulsecount)
{
    const SubGhzProtocolDef *proto = &subghz_protocol_registry[p];
    const uint16_t te_short = proto->timing.te_short;
    const uint16_t te_long  = proto->timing.te_long;
    const uint16_t te_delta = proto->timing.te_delta
        ? proto->timing.te_delta
        : (uint16_t)(te_short * proto->timing.te_tolerance_pct / 100);

    SubGhzBlockDecoder decoder = {0};
    uint16_t i = 0;

    /* Skip preamble: look for ~9 ms HIGH followed by ~4.5 ms LOW.
     * Accept within 7× te_delta for the HIGH and 5× te_delta for the LOW
     * of the nominal 9000 / 4500 µs values (matching X10 decoder tolerances). */
    for (; i + 1 < pulsecount; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        if (DURATION_DIFF(t_hi, (uint16_t)(te_short * 16u)) < (uint16_t)(te_delta * 7u) &&
            DURATION_DIFF(t_lo, (uint16_t)(te_short * 8u))  < (uint16_t)(te_delta * 5u))
        {
            i += 2;  /* advance past preamble */
            break;
        }
    }

    /* Decode 40 data bits using standard X10 OOK PWM encoding:
     *   bit '1': te_short HIGH + te_long  LOW
     *   bit '0': te_short HIGH + te_short LOW */
    for (; i + 1 < pulsecount && decoder.decode_count_bit < CM17A_TOTAL_BITS; i += 2)
    {
        uint16_t t_hi = subghz_decenc_ctl.pulse_times[i];
        uint16_t t_lo = subghz_decenc_ctl.pulse_times[i + 1];

        if (DURATION_DIFF(t_hi, te_short) >= te_delta)
            break;

        if (DURATION_DIFF(t_lo, te_short) < te_delta)
        {
            subghz_protocol_blocks_add_bit(&decoder, 0);
        }
        else if (DURATION_DIFF(t_lo, te_long) < (uint16_t)(te_delta * 2u))
        {
            subghz_protocol_blocks_add_bit(&decoder, 1);
        }
        else
        {
            break;
        }
    }

    if (decoder.decode_count_bit < CM17A_TOTAL_BITS)
        return 1;

    /* Validate header (bits 39-24) and footer (bits 7-0).
     * decode_data holds the 40-bit value right-aligned (bit 0 = last bit). */
    uint64_t raw = decoder.decode_data;
    uint16_t header = (uint16_t)((raw >> 24) & 0xFFFFu);
    uint8_t  footer = (uint8_t)(raw & 0xFFu);

    if (header != CM17A_HEADER || footer != CM17A_FOOTER)
        return 1;

    /* Extract the 16-bit data field (bits 23-8) */
    uint16_t data = (uint16_t)((raw >> 8) & 0xFFFFu);
    uint8_t  house_byte = (uint8_t)(data >> 8);
    uint8_t  cmd_byte   = (uint8_t)(data & 0xFFu);

    /* Decode house code (A-P, stored as 0-based index) */
    uint8_t house_nibble = (house_byte >> 4) & 0x0Fu;
    uint8_t house_letter = cm17a_house_nibble_to_letter[house_nibble];  /* 0=A..15=P */

    /* Decode unit number.
     * bit 2 of house_byte selects the group (0 = units 1-8, 1 = units 9-16).
     * The 3-bit unit index comes from cmd_byte bits [6, 4, 3]. */
    uint8_t unit_group_high = (house_byte >> 2) & 0x01u;  /* 0 or 1 */
    uint8_t unit_bits = (uint8_t)(((cmd_byte >> 4) & 0x04u) |
                                   ((cmd_byte >> 3) & 0x03u));
    uint8_t unit_offset = cm17a_unit_code_to_offset[unit_bits & 0x07u];
    uint8_t unit_number = (uint8_t)(unit_group_high * 8u + unit_offset + 1u);  /* 1-16 */

    /* Decode command.
     * bit 7 of cmd_byte distinguishes DIM/BRIGHT from ON/OFF.
     * Encoding (see spec data table):
     *   0x88 = BRIGHT, 0x98 = DIM, 0x00 = ON, 0x20 = OFF */
    uint8_t cmd_class = 0;
    if (cmd_byte & 0x80u)
    {
        /* DIM / BRIGHT */
        cmd_class = (cmd_byte & 0x10u) ? 2u : 3u;  /* 2=DIM, 3=BRIGHT */
    }
    else
    {
        /* ON / OFF */
        cmd_class = (cmd_byte & 0x20u) ? 1u : 0u;  /* 0=ON, 1=OFF */
    }

    /*
     * Pack decoded fields into SubGhzBlockGeneric:
     *   data           = full 40-bit raw packet
     *   serial         = house_byte (upper nibble encodes house code)
     *   btn            = cmd_byte   (full command byte from spec)
     *   cnt            = (house_letter << 8) | (unit_number << 2) | cmd_class
     *                    (convenience field for display: house A-P, unit 1-16, cmd 0-3)
     */
    SubGhzBlockGeneric generic = {0};
    generic.data           = raw;
    generic.data_count_bit = CM17A_TOTAL_BITS;
    generic.serial         = house_byte;
    generic.btn            = cmd_byte;
    generic.cnt            = ((uint32_t)house_letter << 8)
                           | ((uint32_t)unit_number   << 2)
                           | (uint32_t)cmd_class;
    subghz_block_generic_commit_to_m1(&generic, p);
    return 0;
}
