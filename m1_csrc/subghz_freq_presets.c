/* See COPYING.txt for license details. */

/*
 * subghz_freq_presets.c
 *
 * Sub-GHz frequency preset table — hardware-independent pure data.
 * No HAL, RTOS, or peripheral headers are included here; this file
 * compiles cleanly on the host for unit testing.
 *
 * MAINTENANCE RULES (enforced by tests/test_subghz_freq_presets.c):
 *   • Keep entries sorted in ascending freq_hz order.
 *   • No duplicate freq_hz values.
 *   • All entries must be within [SUBGHZ_MIN_FREQ_HZ, SUBGHZ_MAX_FREQ_HZ].
 *   • Entry count must equal SUBGHZ_FREQ_PRESET_COUNT.
 *   • SUBGHZ_FREQ_DEFAULT_IDX must index the 433.92 MHz entry.
 *   • SUBGHZ_FREQ_PRESET_CUSTOM must equal SUBGHZ_FREQ_PRESET_COUNT.
 *
 * When adding a new Sub-GHz protocol that operates at an uncommon
 * frequency (e.g. 319.5 MHz for Magellan/GE NA sensors), insert the
 * new entry here in sorted order, increment SUBGHZ_FREQ_PRESET_COUNT,
 * and update SUBGHZ_FREQ_DEFAULT_IDX if 433.92 MHz shifts.
 */

#include "subghz_freq_presets.h"

/* ─────────────────────────────────────────────────────────────────────────────
 * Per-region hopper frequency tables
 *
 * MAINTENANCE: every entry here MUST also exist in subghz_freq_presets[].
 * The unit test test_subghz_hopper_freqs_in_presets() enforces this.
 * ───────────────────────────────────────────────────────────────────────────── */

/* North America: NA garage/gate remotes (315/345/390 MHz), 433.92, 434.42, NA ISM 915 */
const uint32_t subghz_hopper_freqs_NA[SUBGHZ_HOPPER_FREQ_COUNT] = {
	315000000, 345000000, 390000000, 433920000, 434420000, 915000000
};

/* Europe: EU SRD 433 cluster + EU 868 MHz SRD band */
const uint32_t subghz_hopper_freqs_EU[SUBGHZ_HOPPER_FREQ_COUNT] = {
	433420000, 433920000, 434075000, 434420000, 868350000, 868950000
};

/* Asia/APAC: gate remotes (330/345 MHz), global 433, 868 */
const uint32_t subghz_hopper_freqs_ASIA[SUBGHZ_HOPPER_FREQ_COUNT] = {
	315000000, 330000000, 345000000, 433920000, 434420000, 868350000
};

/* Off (region disabled): wide cross-region fallback — every major global band */
const uint32_t subghz_hopper_freqs_OFF[SUBGHZ_HOPPER_FREQ_COUNT] = {
	315000000, 390000000, 433920000, 434420000, 868350000, 915000000
};

const uint32_t *subghz_get_hopper_freqs(uint8_t ism_region)
{
	switch (ism_region) {
	case 0:  return subghz_hopper_freqs_NA;    /* SUBGHZ_ISM_BAND_REGION_NORTH_AMERICA  */
	case 1:  return subghz_hopper_freqs_EU;    /* SUBGHZ_ISM_BAND_REGION_EUROPE_REGION_1 */
	case 2:  return subghz_hopper_freqs_ASIA;  /* SUBGHZ_ISM_BAND_REGION_ASIA           */
	default: return subghz_hopper_freqs_OFF;   /* Off or unknown — wide cross-region     */
	}
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Frequency preset table
 * ───────────────────────────────────────────────────────────────────────────── */

const SubGhzFreqPreset subghz_freq_presets[SUBGHZ_FREQ_PRESET_COUNT] = {
	/* 300 – 350 MHz */
	{ 300000000, "300.00"  },
	{ 302757000, "302.75"  },
	{ 303000000, "303.00"  },
	{ 303875000, "303.87"  },
	{ 303900000, "303.90"  },
	{ 304250000, "304.25"  },
	{ 307000000, "307.00"  },
	{ 307500000, "307.50"  },
	{ 307800000, "307.80"  },
	{ 309000000, "309.00"  },
	{ 310000000, "310.00"  },
	{ 312000000, "312.00"  },
	{ 312100000, "312.10"  },
	{ 312200000, "312.20"  },
	{ 313000000, "313.00"  },
	{ 313850000, "313.85"  },
	{ 314000000, "314.00"  },
	{ 314350000, "314.35"  },
	{ 314980000, "314.98"  },
	{ 315000000, "315.00"  },
	{ 318000000, "318.00"  },
	{ 319500000, "319.50"  },   /* Magellan/GE/Interlogix NA security sensors */
	{ 320000000, "320.00"  },
	{ 320150000, "320.15"  },
	{ 330000000, "330.00"  },
	{ 345000000, "345.00"  },
	{ 348000000, "348.00"  },
	{ 350000000, "350.00"  },
	/* 387 – 468 MHz */
	{ 387000000, "387.00"  },
	{ 390000000, "390.00"  },
	{ 418000000, "418.00"  },
	{ 430000000, "430.00"  },
	{ 430500000, "430.50"  },
	{ 431000000, "431.00"  },
	{ 431500000, "431.50"  },
	{ 433075000, "433.07"  },
	{ 433220000, "433.22"  },
	{ 433420000, "433.42"  },
	{ 433657070, "433.65"  },
	{ 433889000, "433.88"  },
	{ 433920000, "433.92"  },   /* index 40 = SUBGHZ_FREQ_DEFAULT_IDX */
	{ 434075000, "434.07"  },
	{ 434176948, "434.17"  },
	{ 434190000, "434.19"  },
	{ 434390000, "434.39"  },
	{ 434420000, "434.42"  },
	{ 434620000, "434.62"  },
	{ 434775000, "434.77"  },
	{ 438900000, "438.90"  },
	{ 440175000, "440.17"  },
	{ 462750000, "462.75"  },
	{ 464000000, "464.00"  },
	{ 467750000, "467.75"  },
	/* 779 – 928 MHz */
	{ 779000000, "779.00"  },
	{ 868350000, "868.35"  },
	{ 868400000, "868.40"  },
	{ 868460000, "868.46"  },
	{ 868800000, "868.80"  },
	{ 868950000, "868.95"  },
	{ 906400000, "906.40"  },
	{ 915000000, "915.00"  },
	{ 925000000, "925.00"  },
	{ 928000000, "928.00"  }
};
