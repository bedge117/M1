/* See COPYING.txt for license details. */

/**
 * @file   subghz_proto_pirate_timing.c
 * @brief  Proto Pirate — pure-logic timing analysis (hardware-independent).
 *
 * Implements pptime_analyze() and pptime_match() for use by the M1
 * Timing Tuner scene.  No hardware registers, RTOS, or display state
 * are touched here — the entire file is host-testable.
 *
 * Protocol timing values are sourced from ProtoPirate's protocol registry
 * and cross-referenced with M1's subghz_protocol_registry timings.
 */

#include "subghz_proto_pirate_timing.h"
#include <string.h>

/*============================================================================*/
/* Automotive protocol reference table                                        */
/*                                                                            */
/* Values sourced from:                                                       */
/*   - ProtoPirate protocol definitions (te_short / te_long / te_delta)       */
/*   - M1 subghz_protocol_registry.c for cross-validation                    */
/*                                                                            */
/* Only OOK/AM protocols with clear short+long timing are listed; some entries
 * use Manchester/biphase AM encoding (Honda V1, Kia V1, PSA AM) but still
 * have distinct short+long pulse widths that the tuner can measure.         */
/*============================================================================*/

const pptime_proto_ref_t pptime_proto_table[] = {
    /* name                  te_short  te_long  te_delta */
    { "KeeLoq",                400,     800,     100  },
    { "Star Line",             400,     800,     100  },
    { "Princeton",             370,    1140,     120  },
    { "CAME",                  320,     640,      80  },
    { "Nice FLO",              700,    1400,     140  },
    { "Holtek HT12X",          340,    1020,     100  },
    { "GateTX",                350,     700,      80  },
    { "SMC5326",               300,     900,     100  },
    { "Chrysler V0",           270,     540,      80  },  /* PWM AM650, ProtoPirate */
    { "Subaru",                620,    1620,     160  },  /* PPM AM650, ProtoPirate */
    { "Honda V1",              400,     800,     100  },  /* Manchester AM650, ProtoPirate */
    { "Kia V1",                400,     800,     100  },  /* Manchester AM650, ProtoPirate */
    { "PSA AM",                400,     800,     100  },  /* Manchester AM650, ProtoPirate */
    { "CAME TWEE",             260,     520,      70  },
    { "CAME Atomo",            400,     800,     100  },
    { "Alutech AT-4N",         400,     800,     100  },
    { "Nice FloR-S",           500,    1000,     120  },
    { "Ansonic",               555,    1110,     130  },
    { "iDo 117",               450,    1350,     130  },
    { "BETT",                  340,     680,      80  },
    { "Nero Radio",            330,     990,     100  },
    { "Clemsa",                385,    1155,     120  },
    { "FireFly",               300,     900,     100  },
    { "Linear",                500,    1500,     150  },
};

const uint8_t pptime_proto_table_count =
    (uint8_t)(sizeof(pptime_proto_table) / sizeof(pptime_proto_table[0]));

/*============================================================================*/
/* pptime_analyze                                                              */
/*============================================================================*/

void pptime_analyze(const uint16_t *durations, size_t count,
                    const pptime_proto_ref_t *ref,
                    pptime_stats_t *out)
{
    if (!out)
        return;

    memset(out, 0, sizeof(*out));

    if (!durations || count == 0)
        return;

    int32_t te_short, te_long, te_delta;
    if (ref && ref->te_short > 0 && ref->te_long > 0)
    {
        te_short  = (int32_t)ref->te_short;
        te_long   = (int32_t)ref->te_long;
        te_delta  = (int32_t)ref->te_delta;
    }
    else
    {
        /* Reasonable defaults when no reference is supplied */
        te_short  = 400;
        te_long   = 800;
        te_delta  = 150;
    }

    /* Threshold = midpoint between expected short and long */
    int32_t threshold = (te_short + te_long) / 2;

    /* Valid range = ±2×te_delta around the short/long extremes */
    int32_t min_valid = te_short - (te_delta * 2);
    if (min_valid < 50) min_valid = 50;
    int32_t max_valid = te_long + (te_delta * 2);

    int64_t short_sum = 0;
    int64_t long_sum  = 0;

    int32_t min_short = INT32_MAX;
    int32_t max_short = 0;
    int32_t min_long  = INT32_MAX;
    int32_t max_long  = 0;

    size_t n_short = 0;
    size_t n_long  = 0;
    size_t n_total = 0;

    for (size_t i = 0; i < count; i++)
    {
        int32_t dur = (int32_t)durations[i];

        if (dur < min_valid || dur > max_valid)
            continue;

        n_total++;

        if (dur < threshold)
        {
            short_sum += dur;
            n_short++;
            if (dur < min_short) min_short = dur;
            if (dur > max_short) max_short = dur;
        }
        else
        {
            long_sum += dur;
            n_long++;
            if (dur < min_long) min_long = dur;
            if (dur > max_long) max_long = dur;
        }
    }

    out->n_total = n_total;
    out->n_short = n_short;
    out->n_long  = n_long;

    if (n_short > 0)
    {
        out->avg_short = (int32_t)(short_sum / (int64_t)n_short);
        out->min_short = min_short;
        out->max_short = max_short;
    }

    if (n_long > 0)
    {
        out->avg_long = (int32_t)(long_sum / (int64_t)n_long);
        out->min_long = min_long;
        out->max_long = max_long;
    }
}

/*============================================================================*/
/* pptime_match                                                                */
/*============================================================================*/

pptime_match_result_t pptime_match(const pptime_stats_t *stats,
                                   const pptime_proto_ref_t *ref)
{
    if (!stats || !ref || ref->te_short == 0 || ref->te_long == 0)
        return PPTIME_MATCH_NO_DATA;

    if (stats->n_short < 4 || stats->n_long < 4)
        return PPTIME_MATCH_NO_DATA;

    int32_t te_s = (int32_t)ref->te_short;
    int32_t te_l = (int32_t)ref->te_long;
    int32_t tol  = (int32_t)ref->te_delta;

    bool short_ok = (stats->avg_short >= te_s - tol) &&
                    (stats->avg_short <= te_s + tol);
    bool long_ok  = (stats->avg_long  >= te_l - tol) &&
                    (stats->avg_long  <= te_l + tol);

    if (short_ok && long_ok)
        return PPTIME_MATCH_OK;

    if (!short_ok && !long_ok)
        return PPTIME_MATCH_MISMATCH;

    if (!short_ok)
    {
        return (stats->avg_short > te_s) ? PPTIME_MATCH_SHORT_HI
                                         : PPTIME_MATCH_SHORT_LO;
    }

    /* long not OK */
    return (stats->avg_long > te_l) ? PPTIME_MATCH_LONG_HI
                                    : PPTIME_MATCH_LONG_LO;
}

/*============================================================================*/
/* pptime_match_str                                                            */
/*============================================================================*/

const char *pptime_match_str(pptime_match_result_t result)
{
    switch (result)
    {
        case PPTIME_MATCH_OK:        return "MATCH";
        case PPTIME_MATCH_SHORT_HI:  return "Short too long";
        case PPTIME_MATCH_SHORT_LO:  return "Short too short";
        case PPTIME_MATCH_LONG_HI:   return "Long too long";
        case PPTIME_MATCH_LONG_LO:   return "Long too short";
        case PPTIME_MATCH_MISMATCH:  return "Mismatch";
        case PPTIME_MATCH_NO_DATA:
        default:                     return "No data";
    }
}
