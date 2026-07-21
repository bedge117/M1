/* See COPYING.txt for license details. */

/*
 * subghz_protocol_registry.c
 *
 * Master protocol registry for M1 Sub-GHz.
 *
 * This single table replaces the three manually-synchronised data structures
 * that previously defined each protocol:
 *
 *   1. The enum in m1_sub_ghz_decenc.h   (protocol indices)
 *   2. subghz_protocols_list[]            (timing parameters)
 *   3. protocol_text[]                    (display names)
 *   4. The switch-case in subghz_decode_protocol()  (dispatch)
 *
 * To add a new protocol you now:
 *   (a) Write the decode function in Sub_Ghz/protocols/m1_<name>_decode.c
 *   (b) Add one entry to the array below
 *   (c) Add the .c file to CMakeLists.txt
 *
 * That's it — no header edits, no switch cases, no manual enum numbering.
 *
 * M1 Project — Hapax fork
 */

#include <string.h>
#include <ctype.h>
#include "subghz_protocol_registry.h"
#include "subghz_signal_format.h"
#include "m1_sub_ghz_decenc.h"
#include "subghz_raw_decoder.h"

/*============================================================================*/
/* Portable case-insensitive compare (replaces POSIX strcasecmp)              */
/*============================================================================*/

int subghz_ascii_strcasecmp(const char *a, const char *b)
{
    unsigned char ca, cb;
    while (*a && *b) {
        ca = (unsigned char)tolower((unsigned char)*a);
        cb = (unsigned char)tolower((unsigned char)*b);
        if (ca != cb)
            return (int)ca - (int)cb;
        a++;
        b++;
    }
    ca = (unsigned char)tolower((unsigned char)*a);
    cb = (unsigned char)tolower((unsigned char)*b);
    return (int)ca - (int)cb;
}

/*============================================================================*/
/* Forward-declare ALL decoder functions so the table compiles.               */
/*                                                                            */
/* These match the existing signatures in m1_sub_ghz_decenc.h — nothing is    */
/* renamed.  When porting a new Flipper/Momentum protocol, add the forward    */
/* declaration here and the entry in the registry below.                       */
/*============================================================================*/

/* Original Monstatek protocols */
extern uint8_t subghz_decode_princeton(uint16_t, uint16_t);
extern uint8_t subghz_decode_security_plus_20(uint16_t, uint16_t);
extern uint8_t subghz_decode_came(uint16_t, uint16_t);
extern uint8_t subghz_decode_nice_flo(uint16_t, uint16_t);
extern uint8_t subghz_decode_linear(uint16_t, uint16_t);
extern uint8_t subghz_decode_holtek(uint16_t, uint16_t);
extern uint8_t subghz_decode_keeloq(uint16_t, uint16_t);
extern uint8_t subghz_decode_oregon_v2(uint16_t, uint16_t);
extern uint8_t subghz_decode_acurite(uint16_t, uint16_t);
extern uint8_t subghz_decode_lacrosse_tx(uint16_t, uint16_t);
extern uint8_t subghz_decode_faac_slh(uint16_t, uint16_t);
extern uint8_t subghz_decode_hormann(uint16_t, uint16_t);
extern uint8_t subghz_decode_marantec(uint16_t, uint16_t);
extern uint8_t subghz_decode_somfy_telis(uint16_t, uint16_t);
extern uint8_t subghz_decode_starline(uint16_t, uint16_t);
extern uint8_t subghz_decode_gate_tx(uint16_t, uint16_t);
extern uint8_t subghz_decode_smc5326(uint16_t, uint16_t);
extern uint8_t subghz_decode_power_smart(uint16_t, uint16_t);
extern uint8_t subghz_decode_ido(uint16_t, uint16_t);
extern uint8_t subghz_decode_ansonic(uint16_t, uint16_t);
extern uint8_t subghz_decode_infactory(uint16_t, uint16_t);
extern uint8_t subghz_decode_schrader(uint16_t, uint16_t);

/* Community-contributed protocols */
extern uint8_t subghz_decode_chamberlain(uint16_t, uint16_t);
extern uint8_t subghz_decode_clemsa(uint16_t, uint16_t);
extern uint8_t subghz_decode_doitrand(uint16_t, uint16_t);
extern uint8_t subghz_decode_bett(uint16_t, uint16_t);
extern uint8_t subghz_decode_nero(uint16_t, uint16_t);
extern uint8_t subghz_decode_firefly(uint16_t, uint16_t);
extern uint8_t subghz_decode_came_twee(uint16_t, uint16_t);
extern uint8_t subghz_decode_came_atomo(uint16_t, uint16_t);
extern uint8_t subghz_decode_nice_flor_s(uint16_t, uint16_t);
extern uint8_t subghz_decode_alutech(uint16_t, uint16_t);
extern uint8_t subghz_decode_centurion(uint16_t, uint16_t);
extern uint8_t subghz_decode_kinggates(uint16_t, uint16_t);
extern uint8_t subghz_decode_megacode(uint16_t, uint16_t);
extern uint8_t subghz_decode_mastercode(uint16_t, uint16_t);
extern uint8_t subghz_decode_chamberlain_7bit(uint16_t, uint16_t);
extern uint8_t subghz_decode_chamberlain_8bit(uint16_t, uint16_t);
extern uint8_t subghz_decode_chamberlain_9bit(uint16_t, uint16_t);
extern uint8_t subghz_decode_liftmaster(uint16_t, uint16_t);
extern uint8_t subghz_decode_dooya(uint16_t, uint16_t);
extern uint8_t subghz_decode_honeywell(uint16_t, uint16_t);
extern uint8_t subghz_decode_intertechno(uint16_t, uint16_t);
extern uint8_t subghz_decode_elro(uint16_t, uint16_t);
extern uint8_t subghz_decode_ambient_weather(uint16_t, uint16_t);
extern uint8_t subghz_decode_bresser_3ch(uint16_t, uint16_t);
extern uint8_t subghz_decode_bresser_5in1(uint16_t, uint16_t);
extern uint8_t subghz_decode_bresser_6in1(uint16_t, uint16_t);
extern uint8_t subghz_decode_tfa_dostmann(uint16_t, uint16_t);
extern uint8_t subghz_decode_nexus_th(uint16_t, uint16_t);
extern uint8_t subghz_decode_thermopro_tx2(uint16_t, uint16_t);
extern uint8_t subghz_decode_gt_wt03(uint16_t, uint16_t);
extern uint8_t subghz_decode_scher_khan_magicar(uint16_t, uint16_t);
extern uint8_t subghz_decode_scher_khan_logicar(uint16_t, uint16_t);
extern uint8_t subghz_decode_toyota(uint16_t, uint16_t);
extern uint8_t subghz_decode_bin_raw(uint16_t, uint16_t);

/* Flipper compatibility protocols */
extern uint8_t subghz_decode_dickert_mahs(uint16_t, uint16_t);
extern uint8_t subghz_decode_feron(uint16_t, uint16_t);
extern uint8_t subghz_decode_gangqi(uint16_t, uint16_t);
extern uint8_t subghz_decode_hay21(uint16_t, uint16_t);
extern uint8_t subghz_decode_hollarm(uint16_t, uint16_t);
extern uint8_t subghz_decode_holtek_base(uint16_t, uint16_t);
extern uint8_t subghz_decode_intertechno_v3(uint16_t, uint16_t);
extern uint8_t subghz_decode_kia_seed(uint16_t, uint16_t);
extern uint8_t subghz_decode_legrand(uint16_t, uint16_t);
extern uint8_t subghz_decode_linear_delta3(uint16_t, uint16_t);
extern uint8_t subghz_decode_magellan(uint16_t, uint16_t);
extern uint8_t subghz_decode_marantec24(uint16_t, uint16_t);
extern uint8_t subghz_decode_nero_sketch(uint16_t, uint16_t);
extern uint8_t subghz_decode_phoenix_v2(uint16_t, uint16_t);
extern uint8_t subghz_decode_revers_rb2(uint16_t, uint16_t);
extern uint8_t subghz_decode_roger(uint16_t, uint16_t);
extern uint8_t subghz_decode_somfy_keytis(uint16_t, uint16_t);

/* Specialty protocols */
extern uint8_t subghz_decode_treadmill37(uint16_t, uint16_t);
extern uint8_t subghz_decode_pocsag(uint16_t, uint16_t);
extern uint8_t subghz_decode_tpms_generic(uint16_t, uint16_t);
extern uint8_t subghz_decode_pcsg_generic(uint16_t, uint16_t);

/* Momentum Phase 3: Weather/Sensor protocols */
extern uint8_t subghz_decode_auriol_ahfl(uint16_t, uint16_t);
extern uint8_t subghz_decode_auriol_hg0601a(uint16_t, uint16_t);
extern uint8_t subghz_decode_gt_wt02(uint16_t, uint16_t);
extern uint8_t subghz_decode_kedsum_th(uint16_t, uint16_t);
extern uint8_t subghz_decode_solight_te44(uint16_t, uint16_t);
extern uint8_t subghz_decode_thermopro_tx4(uint16_t, uint16_t);
extern uint8_t subghz_decode_vauno_en8822c(uint16_t, uint16_t);
extern uint8_t subghz_decode_acurite_606tx(uint16_t, uint16_t);
extern uint8_t subghz_decode_acurite_609txc(uint16_t, uint16_t);
extern uint8_t subghz_decode_emos_e601x(uint16_t, uint16_t);
extern uint8_t subghz_decode_lacrosse_tx141thbv2(uint16_t, uint16_t);
extern uint8_t subghz_decode_wendox_w6726(uint16_t, uint16_t);

/* Momentum Phase 4: Remote/Gate/Automation protocols */
extern uint8_t subghz_decode_ditec_gol4(uint16_t, uint16_t);
extern uint8_t subghz_decode_elplast(uint16_t, uint16_t);
extern uint8_t subghz_decode_honeywell_wdb(uint16_t, uint16_t);
extern uint8_t subghz_decode_keyfinder(uint16_t, uint16_t);
extern uint8_t subghz_decode_x10(uint16_t, uint16_t);

/* Momentum Phase 5: Advanced weather protocols */
extern uint8_t subghz_decode_acurite_592txr(uint16_t, uint16_t);
extern uint8_t subghz_decode_acurite_986(uint16_t, uint16_t);
extern uint8_t subghz_decode_acurite_5n1(uint16_t, uint16_t);
extern uint8_t subghz_decode_tx_8300(uint16_t, uint16_t);
extern uint8_t subghz_decode_oregon_v1(uint16_t, uint16_t);
extern uint8_t subghz_decode_oregon3(uint16_t, uint16_t);

/* Phase 6: Rolling code parity additions */
extern uint8_t subghz_decode_jarolift(uint16_t, uint16_t);
extern uint8_t subghz_decode_beninca_arc(uint16_t, uint16_t);
extern uint8_t subghz_decode_hormann_bisecur(uint16_t, uint16_t);

/* FireCracker / CM17A home-automation RF */
extern uint8_t subghz_decode_firecracker_cm17a(uint16_t, uint16_t);
extern uint8_t subghz_decode_nord_ice(uint16_t, uint16_t);

/* Generic decoders (used as delegates by simple protocols) */
extern uint8_t subghz_decode_generic_pwm(uint16_t, uint16_t);
extern uint8_t subghz_decode_generic_manchester(uint16_t, uint16_t);
extern uint8_t subghz_decode_generic_ppm(uint16_t, uint16_t);

/*============================================================================*/
/* Shorthand flags for common protocol profiles                                */
/*============================================================================*/

#define F_STATIC_433  (SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM | \
                       SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save | \
                       SubGhzProtocolFlag_Send)

/* 300 MHz only — Linear and LinearDelta3 operate exclusively at 300 MHz */
#define F_STATIC_300  (SubGhzProtocolFlag_300 | SubGhzProtocolFlag_AM | \
                       SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save | \
                       SubGhzProtocolFlag_Send)

/* 315 + 433 MHz dual-band — no 868 variant (e.g. Magellan/GE sensors) */
#define F_STATIC_315_433 (SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 | \
                          SubGhzProtocolFlag_AM | SubGhzProtocolFlag_Decodable | \
                          SubGhzProtocolFlag_Save | SubGhzProtocolFlag_Send)

/* 315 + 433 + 868 MHz multi-band — protocols genuinely active on all three */
#define F_STATIC_MULTI (SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 | \
                        SubGhzProtocolFlag_868 | SubGhzProtocolFlag_AM | \
                        SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save | \
                        SubGhzProtocolFlag_Send)

/* 300 + 315 + 433 + 868 MHz multi-band (e.g. Princeton, KeeLoq) */
#define F_STATIC_300_MULTI (SubGhzProtocolFlag_300 | SubGhzProtocolFlag_315 | \
                            SubGhzProtocolFlag_433 | SubGhzProtocolFlag_868 | \
                            SubGhzProtocolFlag_AM | SubGhzProtocolFlag_Decodable | \
                            SubGhzProtocolFlag_Save | SubGhzProtocolFlag_Send)

#define F_ROLLING_433 (SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM | \
                       SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save)

/* OOK-PWM-replayable rolling-code protocols (no cipher, no Manchester, no FSK).
 * These macros include SubGhzProtocolFlag_PwmKeyReplay, which tells the key
 * encoder that a saved Key: value can be faithfully retransmitted using the
 * registry te_short/te_long timings.  Use these instead of F_ROLLING_433 for
 * protocols whose rolling counter carries no per-code encryption. */
#define F_ROLLING_433_PWM (F_ROLLING_433 | SubGhzProtocolFlag_PwmKeyReplay)
#define F_ROLLING_MULTI_PWM (SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 | \
                             SubGhzProtocolFlag_868 | SubGhzProtocolFlag_AM | \
                             SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save | \
                             SubGhzProtocolFlag_PwmKeyReplay)

#define F_ROLLING_MULTI (SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 | \
                         SubGhzProtocolFlag_868 | SubGhzProtocolFlag_AM | \
                         SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save)

/* 300 + 315 + 433 + 868 MHz multi-band rolling (e.g. KeeLoq) */
#define F_ROLLING_300_MULTI (SubGhzProtocolFlag_300 | SubGhzProtocolFlag_315 | \
                             SubGhzProtocolFlag_433 | SubGhzProtocolFlag_868 | \
                             SubGhzProtocolFlag_AM | SubGhzProtocolFlag_Decodable | \
                             SubGhzProtocolFlag_Save)

#define F_WEATHER     (SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM | \
                       SubGhzProtocolFlag_Decodable)

/*============================================================================*/
/* MASTER REGISTRY                                                             */
/*                                                                             */
/* ORDER MATTERS — protocol indices are positional.  For backwards             */
/* compatibility with existing .sub files and history, new protocols must be   */
/* appended at the end.  Never reorder or remove entries.                       */
/*                                                                             */
/* Each entry maps 1:1 with what was previously spread across:                 */
/*   - the anonymous enum in m1_sub_ghz_decenc.h                              */
/*   - subghz_protocols_list[] timing values                                   */
/*   - protocol_text[] name strings                                            */
/*   - the switch-case in subghz_decode_protocol()                             */
/*============================================================================*/

const SubGhzProtocolDef subghz_protocol_registry[] = {

    /* ── Original Monstatek protocols (indices 0..21) ───────────────────── */

    [PRINCETON] = {
        .name   = "Princeton",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_300_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=370, .te_long=1140, .te_tolerance_pct=30, .min_count_bit_for_found=24 },
        .decode = subghz_decode_princeton,
    },
    [SECURITY_PLUS_20] = {
        .name   = "Security+ 2.0",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=250, .te_long=500, .te_tolerance_pct=20, .preamble_bits=16, .min_count_bit_for_found=46 },
        .decode = subghz_decode_security_plus_20,
    },
    [CAME_12BIT] = {
        .name   = "CAME",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433 | SubGhzProtocolFlag_868,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=320, .te_long=640, .te_tolerance_pct=20, .min_count_bit_for_found=12 },
        .decode = subghz_decode_came,
    },
    [NICE_FLO] = {
        .name   = "Nice FLO",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433 | SubGhzProtocolFlag_868,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=700, .te_long=1400, .te_tolerance_pct=20, .min_count_bit_for_found=12 },
        .decode = subghz_decode_nice_flo,
    },
    [LINEAR_10BIT] = {
        .name   = "Linear",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_300,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=500, .te_long=1500, .te_tolerance_pct=25, .min_count_bit_for_found=10 },
        .decode = subghz_decode_linear,
    },
    [HOLTEK_HT12E] = {
        .name   = "Holtek_HT12X",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=340, .te_long=1020, .te_tolerance_pct=25, .min_count_bit_for_found=12 },
        .decode = subghz_decode_holtek,
    },
    [KEELOQ] = {
        .name   = "KeeLoq",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_300_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=800, .te_tolerance_pct=20, .min_count_bit_for_found=66 },
        .decode = subghz_decode_keeloq,
        .get_string = subghz_signal_format_keeloq_info,
    },
    [OREGON_V2] = {
        .name   = "Oregon v2",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=488, .te_long=976, .te_tolerance_pct=25, .preamble_bits=24, .min_count_bit_for_found=64 },
        .decode = subghz_decode_oregon_v2,
    },
    [ACURITE] = {
        .name   = "Acurite",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=200, .te_long=600, .te_tolerance_pct=25, .preamble_bits=4, .min_count_bit_for_found=56 },
        .decode = subghz_decode_acurite,
    },
    [LACROSSE_TX] = {
        .name   = "LaCrosse TX",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=550, .te_long=1100, .te_tolerance_pct=25, .min_count_bit_for_found=44 },
        .decode = subghz_decode_lacrosse_tx,
    },
    [FAAC_SLH] = {
        .name   = "FAAC SLH",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=255, .te_long=510, .te_tolerance_pct=20, .min_count_bit_for_found=64 },
        .decode = subghz_decode_faac_slh,
    },
    [HORMANN] = {
        .name   = "Hormann HSM",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433 | SubGhzProtocolFlag_868,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=500, .te_long=1000, .te_tolerance_pct=20, .min_count_bit_for_found=44 },
        .decode = subghz_decode_hormann,
    },
    [MARANTEC] = {
        .name   = "Marantec",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433 | SubGhzProtocolFlag_868,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=800, .te_long=1600, .te_tolerance_pct=20, .min_count_bit_for_found=12 },
        .decode = subghz_decode_marantec,
    },
    [SOMFY_TELIS] = {
        .name   = "Somfy Telis",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=640, .te_long=1280, .te_tolerance_pct=25, .preamble_bits=4, .min_count_bit_for_found=56 },
        .decode = subghz_decode_somfy_telis,
    },
    [STAR_LINE] = {
        .name   = "Star Line",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=800, .te_tolerance_pct=20, .min_count_bit_for_found=64 },
        .decode = subghz_decode_starline,
        .get_string = subghz_signal_format_keeloq_info,
    },
    [GATE_TX] = {
        .name   = "GateTX",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=350, .te_long=700, .te_tolerance_pct=20, .min_count_bit_for_found=24 },
        .decode = subghz_decode_gate_tx,
    },
    [SMC5326] = {
        .name   = "SMC5326",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=300, .te_long=900, .te_tolerance_pct=25, .min_count_bit_for_found=25 },
        .decode = subghz_decode_smc5326,
    },
    [POWER_SMART] = {
        .name   = "Power Smart",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=225, .te_long=675, .te_tolerance_pct=25, .min_count_bit_for_found=16 },
        .decode = subghz_decode_power_smart,
    },
    [IDO] = {
        .name   = "iDo 117/111",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=450, .te_long=1350, .te_tolerance_pct=25, .min_count_bit_for_found=48 },
        .decode = subghz_decode_ido,
    },
    [ANSONIC] = {
        .name   = "Ansonic",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=555, .te_long=1110, .te_tolerance_pct=20, .min_count_bit_for_found=12 },
        .decode = subghz_decode_ansonic,
    },
    [INFACTORY] = {
        .name   = "Infactory",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=1500, .te_tolerance_pct=25, .preamble_bits=4, .min_count_bit_for_found=40 },
        .decode = subghz_decode_infactory,
    },
    [SCHRADER_TPMS] = {
        .name   = "Schrader TPMS",
        .type   = SubGhzProtocolTypeTPMS,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_TPMS,
        .timing = { .te_short=120, .te_long=240, .te_tolerance_pct=25, .preamble_bits=8, .min_count_bit_for_found=40 },
        .decode = subghz_decode_schrader,
    },

    /* ── Community-contributed protocols (indices 22..56) ─────────────── */

    [CHAMBERLAIN] = {
        .name   = "Cham_Code",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=500, .te_long=1500, .te_tolerance_pct=20, .min_count_bit_for_found=40 },
        .decode = subghz_decode_chamberlain,
    },
    [CLEMSA] = {
        .name   = "Clemsa",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=385, .te_long=1155, .te_tolerance_pct=25, .min_count_bit_for_found=18 },
        .decode = subghz_decode_clemsa,
    },
    [DOITRAND] = {
        .name   = "Doitrand",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=450, .te_long=900, .te_tolerance_pct=20, .min_count_bit_for_found=37 },
        .decode = subghz_decode_doitrand,
    },
    [BETT] = {
        .name   = "BETT",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=340, .te_long=680, .te_tolerance_pct=20, .min_count_bit_for_found=18 },
        .decode = subghz_decode_bett,
    },
    [NERO_RADIO] = {
        .name   = "Nero Radio",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=330, .te_long=990, .te_tolerance_pct=25, .min_count_bit_for_found=36 },
        .decode = subghz_decode_nero,
    },
    [FIREFLY] = {
        .name   = "FireFly",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=300, .te_long=900, .te_tolerance_pct=25, .min_count_bit_for_found=10 },
        .decode = subghz_decode_firefly,
    },
    [CAME_TWEE] = {
        .name   = "CAME TWEE",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=260, .te_long=520, .te_tolerance_pct=20, .min_count_bit_for_found=54 },
        .decode = subghz_decode_came_twee,
    },
    [CAME_ATOMO] = {
        .name   = "CAME Atomo",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=200, .te_long=400, .te_tolerance_pct=20, .min_count_bit_for_found=62 },
        .decode = subghz_decode_came_atomo,
    },
    [NICE_FLOR_S] = {
        .name   = "Nice FloR-S",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=500, .te_long=1000, .te_tolerance_pct=20, .min_count_bit_for_found=52 },
        .decode = subghz_decode_nice_flor_s,
    },
    [ALUTECH_AT4N] = {
        .name   = "Alutech AT-4N",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=800, .te_tolerance_pct=20, .min_count_bit_for_found=72 },
        .decode = subghz_decode_alutech,
    },
    [CENTURION] = {
        .name   = "Centurion",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=336, .te_long=672, .te_tolerance_pct=20, .min_count_bit_for_found=24 },
        .decode = subghz_decode_centurion,
    },
    [KINGGATES_STYLO] = {
        .name   = "KingGates Stylo4k",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=1200, .te_tolerance_pct=20, .min_count_bit_for_found=60 },
        .decode = subghz_decode_kinggates,
    },
    [MEGACODE] = {
        .name   = "MegaCode",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=1000, .te_long=2000, .te_tolerance_pct=20, .min_count_bit_for_found=24 },
        .decode = subghz_decode_megacode,
    },
    [MASTERCODE] = {
        .name   = "Mastercode",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=500, .te_long=1500, .te_tolerance_pct=20, .min_count_bit_for_found=36 },
        .decode = subghz_decode_mastercode,
    },
    [CHAMBERLAIN_7BIT] = {
        .name   = "Cham_Code",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=2000, .te_long=6000, .te_tolerance_pct=25, .min_count_bit_for_found=7 },
        .decode = subghz_decode_chamberlain_7bit,
    },
    [CHAMBERLAIN_8BIT] = {
        .name   = "Cham_Code",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=2000, .te_long=6000, .te_tolerance_pct=25, .min_count_bit_for_found=8 },
        .decode = subghz_decode_chamberlain_8bit,
    },
    [CHAMBERLAIN_9BIT] = {
        .name   = "Cham_Code",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=2000, .te_long=6000, .te_tolerance_pct=25, .min_count_bit_for_found=9 },
        .decode = subghz_decode_chamberlain_9bit,
    },
    [LIFTMASTER_10BIT] = {
        .name   = "Security+ 1.0",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=1000, .te_long=3000, .te_tolerance_pct=25, .min_count_bit_for_found=10 },
        .decode = subghz_decode_liftmaster,
    },
    [DOOYA] = {
        .name   = "Dooya",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=1200, .te_tolerance_pct=20, .min_count_bit_for_found=40 },
        .decode = subghz_decode_dooya,
    },
    [HONEYWELL] = {
        .name   = "Honeywell",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=250, .te_long=750, .te_tolerance_pct=25, .min_count_bit_for_found=48 },
        .decode = subghz_decode_honeywell,
    },
    [INTERTECHNO] = {
        .name   = "Intertechno",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=250, .te_long=750, .te_tolerance_pct=25, .min_count_bit_for_found=32 },
        .decode = subghz_decode_intertechno,
    },
    [ELRO] = {
        .name   = "Elro",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=330, .te_long=990, .te_tolerance_pct=25, .min_count_bit_for_found=32 },
        .decode = subghz_decode_elro,
    },
    [AMBIENT_WEATHER] = {
        .name   = "Ambient Weather",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=1000, .te_tolerance_pct=25, .min_count_bit_for_found=40 },
        .decode = subghz_decode_ambient_weather,
    },
    [BRESSER_3CH] = {
        .name   = "Bresser 3ch",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=250, .te_long=500, .te_tolerance_pct=25, .min_count_bit_for_found=40 },
        .decode = subghz_decode_bresser_3ch,
    },
    [BRESSER_5IN1] = {
        .name   = "Bresser 5in1",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=250, .te_long=500, .te_tolerance_pct=25, .min_count_bit_for_found=56 },
        .decode = subghz_decode_bresser_5in1,
    },
    [BRESSER_6IN1] = {
        .name   = "Bresser 6in1",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=250, .te_long=500, .te_tolerance_pct=25, .min_count_bit_for_found=104 },
        .decode = subghz_decode_bresser_6in1,
    },
    [TFA_DOSTMANN] = {
        .name   = "TFA Dostmann",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=1000, .te_tolerance_pct=25, .min_count_bit_for_found=48 },
        .decode = subghz_decode_tfa_dostmann,
    },
    [NEXUS_TH] = {
        .name   = "Nexus-TH",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=1000, .te_tolerance_pct=25, .min_count_bit_for_found=36 },
        .decode = subghz_decode_nexus_th,
    },
    [THERMOPRO_TX2] = {
        .name   = "ThermoPro TX-2",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=250, .te_long=500, .te_tolerance_pct=25, .min_count_bit_for_found=37 },
        .decode = subghz_decode_thermopro_tx2,
    },
    [GT_WT03] = {
        .name   = "GT-WT03",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=1000, .te_tolerance_pct=25, .min_count_bit_for_found=40 },
        .decode = subghz_decode_gt_wt03,
    },
    [SCHER_KHAN_MAGICAR] = {
        .name   = "Scher-Khan",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=800, .te_tolerance_pct=20, .min_count_bit_for_found=64 },
        .decode = subghz_decode_scher_khan_magicar,
    },
    [SCHER_KHAN_LOGICAR] = {
        .name   = "Scher-Khan",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=1200, .te_tolerance_pct=20, .min_count_bit_for_found=64 },
        .decode = subghz_decode_scher_khan_logicar,
    },
    [TOYOTA] = {
        .name   = "Toyota",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=250, .te_long=750, .te_tolerance_pct=20, .min_count_bit_for_found=56 },
        .decode = subghz_decode_toyota,
    },
    [BIN_RAW] = {
        .name   = "BinRAW",
        .type   = SubGhzProtocolTypeRAW,
        .flags  = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM | SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=100, .te_long=300, .te_tolerance_pct=30, .min_count_bit_for_found=64 },
        .decode = subghz_decode_bin_raw,
    },

    /* ── Flipper compatibility protocols (indices 57..73) ────────────── */

    [DICKERT_MAHS] = {
        .name   = "Dickert_MAHS",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=800, .te_tolerance_pct=20, .min_count_bit_for_found=36 },
        .decode = subghz_decode_dickert_mahs,
    },
    [FERON] = {
        .name   = "Feron",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=350, .te_long=750, .te_tolerance_pct=20, .min_count_bit_for_found=32 },
        .decode = subghz_decode_feron,
    },
    [GANGQI] = {
        .name   = "GangQi",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=500, .te_long=1200, .te_tolerance_pct=20, .min_count_bit_for_found=34 },
        .decode = subghz_decode_gangqi,
    },
    [HAY21] = {
        .name   = "Hay21",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=300, .te_long=700, .te_tolerance_pct=20, .min_count_bit_for_found=21 },
        .decode = subghz_decode_hay21,
    },
    [HOLLARM] = {
        .name   = "Hollarm",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=200, .te_long=1000, .te_tolerance_pct=20, .min_count_bit_for_found=42 },
        .decode = subghz_decode_hollarm,
    },
    [HOLTEK_BASE] = {
        .name   = "Holtek",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=430, .te_long=870, .te_tolerance_pct=20, .min_count_bit_for_found=40 },
        .decode = subghz_decode_holtek_base,
    },
    [INTERTECHNO_V3] = {
        .name   = "Intertechno_V3",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=275, .te_long=1375, .te_tolerance_pct=20, .min_count_bit_for_found=32 },
        .decode = subghz_decode_intertechno_v3,
    },
    [KIA_SEED] = {
        .name   = "KIA Seed",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=250, .te_long=500, .te_tolerance_pct=20, .min_count_bit_for_found=61 },
        .decode = subghz_decode_kia_seed,
    },
    [LEGRAND] = {
        .name   = "Legrand",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=375, .te_long=1125, .te_tolerance_pct=25, .min_count_bit_for_found=18 },
        .decode = subghz_decode_legrand,
    },
    [LINEAR_DELTA3] = {
        .name   = "LinearDelta3",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_300,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=500, .te_long=2000, .te_tolerance_pct=20, .min_count_bit_for_found=8 },
        .decode = subghz_decode_linear_delta3,
    },
    [MAGELLAN] = {
        .name   = "Magellan",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_315_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=200, .te_long=400, .te_tolerance_pct=20, .min_count_bit_for_found=32 },
        .decode = subghz_decode_magellan,
    },
    [MARANTEC24] = {
        .name   = "Marantec24",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433 | SubGhzProtocolFlag_868,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=800, .te_long=1600, .te_tolerance_pct=20, .min_count_bit_for_found=24 },
        .decode = subghz_decode_marantec24,
    },
    [NERO_SKETCH] = {
        .name   = "Nero Sketch",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=330, .te_long=660, .te_tolerance_pct=20, .min_count_bit_for_found=40 },
        .decode = subghz_decode_nero_sketch,
    },
    [PHOENIX_V2] = {
        .name   = "Phoenix_V2",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=427, .te_long=853, .te_tolerance_pct=20, .min_count_bit_for_found=52 },
        .decode = subghz_decode_phoenix_v2,
    },
    [REVERS_RB2] = {
        .name   = "Revers_RB2",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=250, .te_long=500, .te_tolerance_pct=25, .min_count_bit_for_found=64 },
        .decode = subghz_decode_revers_rb2,
    },
    [ROGER] = {
        .name   = "Roger",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=500, .te_long=1000, .te_tolerance_pct=25, .min_count_bit_for_found=28 },
        .decode = subghz_decode_roger,
    },
    [SOMFY_KEYTIS] = {
        .name   = "Somfy Keytis",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=640, .te_long=1280, .te_tolerance_pct=25, .min_count_bit_for_found=80 },
        .decode = subghz_decode_somfy_keytis,
    },

    /* ── Momentum Phase 3: Weather/Sensor protocols ────────────────────── */

    [AURIOL_AHFL] = {
        .name   = "Auriol_AHFL",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=2000, .te_delta=150, .min_count_bit_for_found=42 },
        .decode = subghz_decode_auriol_ahfl,
    },
    [AURIOL_HG0601A] = {
        .name   = "Auriol_HG0601A",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=2000, .te_delta=150, .min_count_bit_for_found=37 },
        .decode = subghz_decode_auriol_hg0601a,
    },
    [GT_WT02] = {
        .name   = "GT-WT02",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=2000, .te_delta=150, .min_count_bit_for_found=37 },
        .decode = subghz_decode_gt_wt02,
    },
    [KEDSUM_TH] = {
        .name   = "Kedsum-TH",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=2000, .te_delta=150, .min_count_bit_for_found=42 },
        .decode = subghz_decode_kedsum_th,
    },
    [SOLIGHT_TE44] = {
        .name   = "Solight_TE44",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=490, .te_long=1960, .te_delta=150, .min_count_bit_for_found=36 },
        .decode = subghz_decode_solight_te44,
    },
    [THERMOPRO_TX4] = {
        .name   = "ThermoPro TX-4",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=2000, .te_delta=150, .min_count_bit_for_found=37 },
        .decode = subghz_decode_thermopro_tx4,
    },
    [VAUNO_EN8822C] = {
        .name   = "Vauno_EN8822C",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=1940, .te_delta=150, .min_count_bit_for_found=42 },
        .decode = subghz_decode_vauno_en8822c,
    },
    [ACURITE_606TX] = {
        .name   = "Acurite_606TX",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=2000, .te_delta=150, .min_count_bit_for_found=32 },
        .decode = subghz_decode_acurite_606tx,
    },
    [ACURITE_609TXC] = {
        .name   = "Acurite_609TXC",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=1000, .te_delta=150, .min_count_bit_for_found=40 },
        .decode = subghz_decode_acurite_609txc,
    },
    [EMOS_E601X] = {
        .name   = "Emos_E601x",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=260, .te_long=800, .te_delta=100, .min_count_bit_for_found=24 },
        .decode = subghz_decode_emos_e601x,
    },
    [LACROSSE_TX141THBV2] = {
        .name   = "LaCrosse_TX141THBv2",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=208, .te_long=417, .te_delta=120, .min_count_bit_for_found=40 },
        .decode = subghz_decode_lacrosse_tx141thbv2,
    },
    [WENDOX_W6726] = {
        .name   = "Wendox_W6726",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=1955, .te_long=5865, .te_delta=300, .min_count_bit_for_found=29 },
        .decode = subghz_decode_wendox_w6726,
    },

    /* ── Momentum Phase 4: Remote/Gate/Automation protocols ────────────── */

    [DITEC_GOL4] = {
        .name   = "DITEC_GOL4",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433_PWM,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=1100, .te_delta=200, .min_count_bit_for_found=54 },
        .decode = subghz_decode_ditec_gol4,
    },
    [ELPLAST] = {
        .name   = "Elplast",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=230, .te_long=1550, .te_delta=160, .min_count_bit_for_found=18 },
        .decode = subghz_decode_elplast,
    },
    [HONEYWELL_WDB] = {
        .name   = "Honeywell_WDB",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_MULTI,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=160, .te_long=320, .te_delta=60, .min_count_bit_for_found=48 },
        .decode = subghz_decode_honeywell_wdb,
    },
    [KEYFINDER] = {
        .name   = "KeyFinder",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=400, .te_long=1200, .te_delta=150, .min_count_bit_for_found=24 },
        .decode = subghz_decode_keyfinder,
    },
    [X10] = {
        .name   = "X10",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = SubGhzProtocolFlag_315 | SubGhzProtocolFlag_AM |
                  SubGhzProtocolFlag_Decodable,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=600, .te_long=1800, .te_delta=100, .min_count_bit_for_found=32 },
        .decode = subghz_decode_x10,
    },

    /* --- Specialty protocols --- */
    [TREADMILL37] = {
        .name   = "Treadmill37",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = F_STATIC_433,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=300, .te_long=900, .te_tolerance_pct=25, .min_count_bit_for_found=37 },
        .decode = subghz_decode_treadmill37,
    },
    [POCSAG] = {
        .name   = "POCSAG",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_FM |
                  SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save,
        .filter = SubGhzProtocolFilter_Industrial,
        .timing = { .te_short=833, .te_long=833, .te_tolerance_pct=15, .min_count_bit_for_found=32 },
        .decode = subghz_decode_pocsag,
    },
    [TPMS_GENERIC] = {
        .name   = "TPMS Generic",
        .type   = SubGhzProtocolTypeTPMS,
        .flags  = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_315 |
                  SubGhzProtocolFlag_AM | SubGhzProtocolFlag_Decodable,
        .filter = SubGhzProtocolFilter_TPMS,
        .timing = { .te_short=52, .te_long=104, .te_tolerance_pct=30, .min_count_bit_for_found=32 },
        .decode = subghz_decode_tpms_generic,
    },
    [PCSG_GENERIC] = {
        .name   = "PCSG Generic",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_FM |
                  SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save,
        .filter = SubGhzProtocolFilter_Industrial,
        .timing = { .te_short=833, .te_long=1666, .te_tolerance_pct=20, .min_count_bit_for_found=32 },
        .decode = subghz_decode_pcsg_generic,
    },

    /* --- Momentum Phase 5: Advanced weather protocols --- */
    [ACURITE_592TXR] = {
        .name   = "Acurite_592TXR",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=200, .te_long=400, .te_delta=90, .min_count_bit_for_found=56 },
        .decode = subghz_decode_acurite_592txr,
    },
    [ACURITE_986] = {
        .name   = "Acurite_986",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=800, .te_long=1750, .te_delta=50, .min_count_bit_for_found=40 },
        .decode = subghz_decode_acurite_986,
    },
    [ACURITE_5N1] = {
        .name   = "Acurite_5n1",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=200, .te_long=400, .te_delta=90, .min_count_bit_for_found=64 },
        .decode = subghz_decode_acurite_5n1,
    },
    [TX_8300] = {
        .name   = "TX_8300",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=1940, .te_long=3880, .te_delta=250, .min_count_bit_for_found=72 },
        .decode = subghz_decode_tx_8300,
    },
    [OREGON_V1] = {
        .name   = "Oregon_V1",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=1465, .te_long=2930, .te_delta=350, .min_count_bit_for_found=32 },
        .decode = subghz_decode_oregon_v1,
    },
    [OREGON3] = {
        .name   = "Oregon3",
        .type   = SubGhzProtocolTypeWeather,
        .flags  = F_WEATHER,
        .filter = SubGhzProtocolFilter_Weather,
        .timing = { .te_short=500, .te_long=1100, .te_delta=300, .min_count_bit_for_found=32 },
        .decode = subghz_decode_oregon3,
    },

    /* ── Phase 6: Rolling code parity additions ─────────────────────────── */

    [JAROLIFT] = {
        .name   = "Jarolift",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433,
        .filter = SubGhzProtocolFilter_Auto,
        /* KeeLoq-based: 72-bit OOK PWM, te=400 µs, 1:2 ratio, 12 preamble pulses */
        .timing = { .te_short=400, .te_long=800, .te_delta=167,
                    .preamble_bits=12, .min_count_bit_for_found=72 },
        .decode = subghz_decode_jarolift,
        .get_string = subghz_signal_format_keeloq_info,
    },
    [BENINCA_ARC] = {
        .name   = "Beninca ARC",
        .type   = SubGhzProtocolTypeDynamic,
        .flags  = F_ROLLING_433,
        .filter = SubGhzProtocolFilter_Auto,
        /* AES-128 encrypted: 128-bit OOK PWM, te=300 µs, 1:2 ratio */
        .timing = { .te_short=300, .te_long=600, .te_delta=155,
                    .preamble_bits=0, .min_count_bit_for_found=128 },
        .decode = subghz_decode_beninca_arc,
    },
    [HORMANN_BISECUR] = {
        .name   = "Hormann BiSecur",
        .type   = SubGhzProtocolTypeDynamic,
        /* BiSecur operates at 868 MHz (OOK-Manchester) */
        .flags  = SubGhzProtocolFlag_868 | SubGhzProtocolFlag_AM |
                  SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Save,
        .filter = SubGhzProtocolFilter_Auto,
        /* Manchester: 176-bit, te=208 µs, te_long=416 µs */
        .timing = { .te_short=208, .te_long=416, .te_delta=104,
                    .preamble_bits=0, .min_count_bit_for_found=176 },
        .decode = subghz_decode_hormann_bisecur,
    },

    /* ── FireCracker / CM17A home-automation RF ─────────────────────── */

    [FIRECRACKER_CM17A] = {
        /*
         * FireCracker (CM17A) X10 home-automation RF protocol.
         * 40-bit packet: [0xD5AA header][16-bit data][0xAD footer]
         * Physical layer matches standard X10 RF (same OOK PWM timing).
         * Frequency: 310 MHz (North America) / 433.92 MHz (Europe)
         * Ref: https://github.com/evilpete/flipper_toolbox/raw/refs/heads/main/subghz/firecracker_spec.txt
         *
         * Note: SubGhzProtocolFlag_Send is intentionally omitted. The generic
         * static TX path (subghz_encode_ook_pwm) uses standard OOK PWM where
         * bit-0 = short-HIGH + long-LOW and bit-1 = long-HIGH + short-LOW.
         * CM17A/X10 requires a constant short HIGH with only the LOW duration
         * varying (bit-0 = short-LOW, bit-1 = long-LOW). A generic send would
         * produce an incorrect waveform until a protocol-specific TX encoder
         * is implemented. FireCracker is decode/save-only for now.
         */
        .name   = "FireCracker",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 |
                  SubGhzProtocolFlag_AM  | SubGhzProtocolFlag_Decodable |
                  SubGhzProtocolFlag_Save,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=562, .te_long=1688, .te_delta=100,
                    .min_count_bit_for_found=40 },
        .decode = subghz_decode_firecracker_cm17a,
    },

    /* ── Nord ICE gate/garage remote (Momentum Firmware, 2026-03) ──────── */

    [NORD_ICE] = {
        /*
         * Nord ICE 33-bit OOK PWM gate/garage remote.
         * Frequency: 433.92 MHz
         * Packet: 33 bits, no fixed preamble — GAP-triggered (7500 µs).
         *   bits 32-15: 18-bit serial (upper)
         *   bits 14- 9:  6-bit button code
         *   bits  8- 0:  9-bit serial (lower)
         * Reference: Momentum Firmware nord_ice.c, @xMasterX (MMX) 2026-03
         */
        .name   = "Nord ICE",
        .type   = SubGhzProtocolTypeStatic,
        .flags  = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM |
                  SubGhzProtocolFlag_Decodable | SubGhzProtocolFlag_Load |
                  SubGhzProtocolFlag_Save | SubGhzProtocolFlag_Send,
        .filter = SubGhzProtocolFilter_Auto,
        .timing = { .te_short=300, .te_long=800, .te_delta=150,
                    .min_count_bit_for_found=33 },
        .decode = subghz_decode_nord_ice,
    },
};

const uint16_t subghz_protocol_registry_count =
    sizeof(subghz_protocol_registry) / sizeof(subghz_protocol_registry[0]);

/*
 * Build-time check: ensure registry does not exceed legacy array capacity.
 * LEGACY_PROTOCOL_MAX is defined in m1_sub_ghz_decenc.c as 128.
 * If the registry grows beyond this, increase LEGACY_PROTOCOL_MAX or remove
 * legacy array support.
 */
_Static_assert(
    sizeof(subghz_protocol_registry) / sizeof(subghz_protocol_registry[0]) <= 128,
    "Protocol registry exceeds LEGACY_PROTOCOL_MAX (128) — increase the limit");

/*============================================================================*/
/* Registry Lookup Implementations                                             */
/*============================================================================*/

int16_t subghz_protocol_find_by_name(const char *name)
{
    if (!name) return -1;
    for (uint16_t i = 0; i < subghz_protocol_registry_count; i++) {
        if (subghz_protocol_registry[i].name &&
            subghz_ascii_strcasecmp(subghz_protocol_registry[i].name, name) == 0) {
            return (int16_t)i;
        }
    }
    return -1;
}

const SubGhzProtocolDef* subghz_protocol_get(uint16_t index)
{
    if (index >= subghz_protocol_registry_count) return NULL;
    return &subghz_protocol_registry[index];
}

const char* subghz_protocol_get_name(uint16_t index)
{
    if (index >= subghz_protocol_registry_count) return NULL;
    return subghz_protocol_registry[index].name;
}

/* subghz_registry_decode_try_fn() has been extracted to subghz_decode_try_fn.c
 * to enable isolated unit testing with a minimal stub registry.  The function
 * is declared in subghz_raw_decoder.h and linked from either this production
 * build or from subghz_decode_try_fn.c. */
