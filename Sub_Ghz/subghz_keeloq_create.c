/* See COPYING.txt for license details. */

/**
 * @file   subghz_keeloq_create.c
 * @brief  KeeLoq-family Create-from-scratch key assembler (Phase 8c-3).
 *
 * Pure-logic implementation — no hardware, no FAT FS, no RTOS.
 */

#include "subghz_keeloq_create.h"

#include <ctype.h>
#include <string.h>

#include "subghz_keeloq.h"

/*============================================================================*/
/* Field masks                                                                 */
/*============================================================================*/

#define KEELOQ_SERIAL_MASK_28       0x0FFFFFFFUL
#define KEELOQ_SERIAL_DISCRIM_MASK  0x3FUL        /* low 6 bits of serial */
#define KEELOQ_BUTTON_MASK_4        0x0FU
#define KEELOQ_COUNTER_MASK_16      0xFFFFUL

/*============================================================================*/
/* Protocol detection (mirrors subghz_keeloq_encoder.c::extract_fields rule)  */
/*============================================================================*/

static bool is_star_line(const char *protocol)
{
    if (!protocol) return false;
    const char *a = protocol;
    const char *b = "Star Line";
    while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b))
    {
        ++a; ++b;
    }
    return (*b == '\0');
}

static bool is_supported_keeloq(const char *protocol)
{
    if (!protocol) return false;
    /* Accept "KeeLoq", "Star Line", "Jarolift" (case-insensitive, prefix
     * match terminated by NUL or space — matches the encoder's
     * `keeloq_is_keeloq_protocol`). */
    const char *names[] = { "KeeLoq", "Star Line", "Jarolift" };
    for (size_t i = 0; i < sizeof(names)/sizeof(names[0]); ++i)
    {
        const char *a = protocol;
        const char *b = names[i];
        while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b))
        {
            ++a; ++b;
        }
        if (*b == '\0' && (*a == '\0' || *a == ' '))
            return true;
    }
    return false;
}

/*============================================================================*/
/* Plaintext HOP builder                                                       */
/*============================================================================*/

uint32_t subghz_keeloq_create_plaintext(uint32_t serial, uint8_t button,
                                         uint32_t counter)
{
    uint32_t plain  = 0U;
    plain |= ((counter & KEELOQ_COUNTER_MASK_16) << 16);          /* [31:16] */
    plain |= ((uint32_t)(button & KEELOQ_BUTTON_MASK_4) << 12);   /* [15:12] */
    /* [11:10] VLOW = 0 */
    plain |= ((serial & KEELOQ_SERIAL_DISCRIM_MASK) << 4);        /* [ 9: 4] */
    /* [ 3: 0] overflow = 0 */
    return plain;
}

/*============================================================================*/
/* 64-bit key assembler                                                        */
/*============================================================================*/

static uint64_t assemble_flipper_key(const char *protocol, uint32_t hop_enc,
                                      uint32_t serial, uint8_t button)
{
    const uint32_t s = serial & KEELOQ_SERIAL_MASK_28;
    const uint8_t  b = button & KEELOQ_BUTTON_MASK_4;

    if (is_star_line(protocol))
    {
        return ((uint64_t)hop_enc << 32) |
               ((uint64_t)s << 4) |
               (uint64_t)b;
    }
    /* KeeLoq / Jarolift */
    return ((uint64_t)b << 60) |
           ((uint64_t)s << 32) |
           (uint64_t)hop_enc;
}

KeeLoqCreateResult subghz_keeloq_create_key(const KeeLoqCreateParams *params,
                                             uint64_t                 *key_out)
{
    /* Honour the documented contract: on every failure path that has a
     * usable out pointer, @p *key_out is set to 0.  Initialise BEFORE
     * the @p params NULL-check so the only remaining failure mode that
     * cannot zero @p *key_out is the one where @p key_out itself is
     * NULL (i.e. the caller has nothing to receive the value). */
    if (!key_out)
        return KEELOQ_CREATE_BAD_ARG;

    *key_out = 0ULL;

    if (!params)
        return KEELOQ_CREATE_BAD_ARG;

    if (!is_supported_keeloq(params->protocol))
        return KEELOQ_CREATE_BAD_PROTOCOL;

    /* Derive the device key from the manufacturer master key. */
    uint64_t device_key;
    switch (params->learn_type)
    {
        case KEELOQ_LEARN_NORMAL:
            device_key = keeloq_learn_normal(params->serial & KEELOQ_SERIAL_MASK_28,
                                              params->mfr_key);
            break;
        case KEELOQ_LEARN_SIMPLE:
            device_key = keeloq_learn_simple(params->serial & KEELOQ_SERIAL_MASK_28,
                                              params->mfr_key);
            break;
        case KEELOQ_LEARN_SECURE:
        default:
            /* Secure Learning needs a seed (which is not available in a
             * Create-from-scratch flow).  Refuse rather than silently
             * fall back to a different learn mode.
             * FAAC SLH (type 5) also requires a seed. */
            return KEELOQ_CREATE_BAD_LEARN;
        case KEELOQ_LEARN_MAGIC_XOR_TYPE1:
            device_key = keeloq_learn_magic_xor_type1(
                             params->serial & KEELOQ_SERIAL_MASK_28,
                             params->mfr_key);
            break;
    }

    /* Build the plaintext HOP word and encrypt it with the device key. */
    uint32_t plain = subghz_keeloq_create_plaintext(params->serial,
                                                     params->button,
                                                     params->counter);
    uint32_t hop_enc = keeloq_encrypt(plain, device_key);

    *key_out = assemble_flipper_key(params->protocol, hop_enc,
                                     params->serial, params->button);
    return KEELOQ_CREATE_OK;
}
