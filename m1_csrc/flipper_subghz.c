/* See COPYING.txt for license details. */

/*
 * flipper_subghz.c
 *
 * Flipper Zero .sub file format parser for Sub-GHz signals
 *
 * M1 Project
 */

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "flipper_subghz.h"
#include "m1_sub_ghz.h"

/*************************** D E F I N E S ************************************/

/* FLIPPER_SUBGHZ_RAW_FILETYPE is defined in flipper_subghz.h */
#define FLIPPER_SUBGHZ_KEY_FILETYPE      "Flipper SubGhz Key File"
#define FLIPPER_SUBGHZ_MIN_VERSION       1

/* M1 native .sgh format — filetype prefix and type keywords */
#define M1_SUBGHZ_FILETYPE_PREFIX        "M1 SubGHz"
#define M1_SUBGHZ_NOISE_TYPE             "NOISE"    /* RAW recording */
#define M1_SUBGHZ_PACKET_TYPE            "PACKET"   /* Decoded/key file */

/* Maximum number of int32 samples to parse from a single RAW_Data line */
#define SUBGHZ_LINE_SAMPLES_MAX          256

//************************** S T R U C T U R E S *******************************

/***************************** V A R I A B L E S ******************************/

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static int subghz_strcasecmp(const char *a, const char *b);
static bool subghz_parse_raw(flipper_file_t *ctx, flipper_subghz_signal_t *out);
static bool subghz_parse_key(flipper_file_t *ctx, flipper_subghz_signal_t *out);
static bool m1sgh_parse_raw(flipper_file_t *ctx, flipper_subghz_signal_t *out);
static bool m1sgh_parse_packet(flipper_file_t *ctx, flipper_subghz_signal_t *out);
static const char *stristr(const char *haystack, const char *needle);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/

/*============================================================================*/
/**
 * @brief  Case-insensitive string comparison (portable)
 */
static int subghz_strcasecmp(const char *a, const char *b)
{
	while (*a && *b)
	{
		int ca = tolower((unsigned char)*a);
		int cb = tolower((unsigned char)*b);
		if (ca != cb)
			return ca - cb;
		a++;
		b++;
	}
	return (unsigned char)*a - (unsigned char)*b;
}

/*============================================================================*/
/**
 * @brief  Parse a raw Sub-GHz file body after header validation
 */
static bool subghz_parse_raw(flipper_file_t *ctx, flipper_subghz_signal_t *out)
{
	int32_t line_samples[SUBGHZ_LINE_SAMPLES_MAX];
	uint16_t line_count;
	uint16_t i;

	out->type = FLIPPER_SUBGHZ_TYPE_RAW;
	out->raw_count = 0;

	while (ff_read_line(ctx))
	{
		if (ff_is_separator(ctx))
			continue;

		if (!ff_parse_kv(ctx))
			continue;

		if (subghz_strcasecmp(ff_get_key(ctx), "Frequency") == 0)
		{
			out->frequency = (uint32_t)strtoul(ff_get_value(ctx), NULL, 10);
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Preset") == 0)
		{
			strncpy(out->preset, ff_get_value(ctx), FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1);
			out->preset[FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1] = '\0';
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Protocol") == 0)
		{
			strncpy(out->protocol, ff_get_value(ctx), FLIPPER_SUBGHZ_PROTO_MAX_LEN - 1);
			out->protocol[FLIPPER_SUBGHZ_PROTO_MAX_LEN - 1] = '\0';
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "RAW_Data") == 0)
		{
			/* Parse the int32 samples from this line and append to raw_data */
			line_count = ff_parse_int32_array(ff_get_value(ctx),
			                                  line_samples,
			                                  SUBGHZ_LINE_SAMPLES_MAX);

			for (i = 0; i < line_count && out->raw_count < FLIPPER_SUBGHZ_RAW_MAX_SAMPLES; i++)
			{
				/* Clamp int32 values to int16 range */
				int32_t val = line_samples[i];
				if (val > 32767)
					val = 32767;
				else if (val < -32768)
					val = -32768;

				out->raw_data[out->raw_count++] = (int16_t)val;
			}
		}
	}

	return (out->raw_count > 0);
}

/*============================================================================*/
/**
 * @brief  Parse a key/parsed Sub-GHz file body after header validation
 */
static bool subghz_parse_key(flipper_file_t *ctx, flipper_subghz_signal_t *out)
{
	out->type = FLIPPER_SUBGHZ_TYPE_PARSED;

	while (ff_read_line(ctx))
	{
		if (ff_is_separator(ctx))
			continue;

		if (!ff_parse_kv(ctx))
			continue;

		if (subghz_strcasecmp(ff_get_key(ctx), "Frequency") == 0)
		{
			out->frequency = (uint32_t)strtoul(ff_get_value(ctx), NULL, 10);
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Preset") == 0)
		{
			strncpy(out->preset, ff_get_value(ctx), FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1);
			out->preset[FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1] = '\0';
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Protocol") == 0)
		{
			strncpy(out->protocol, ff_get_value(ctx), FLIPPER_SUBGHZ_PROTO_MAX_LEN - 1);
			out->protocol[FLIPPER_SUBGHZ_PROTO_MAX_LEN - 1] = '\0';
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Bit") == 0)
		{
			out->bit_count = (uint32_t)strtoul(ff_get_value(ctx), NULL, 10);
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Key") == 0)
		{
			/* Key is in format "00 00 00 00 07 04 D0 03" (8 hex bytes, big-endian) */
			uint8_t key_bytes[8];
			uint8_t count;
			uint8_t i;

			memset(key_bytes, 0, sizeof(key_bytes));
			count = ff_parse_hex_bytes(ff_get_value(ctx), key_bytes, 8);

			out->key = 0;
			for (i = 0; i < count && i < 8; i++)
			{
				out->key = (out->key << 8) | key_bytes[i];
			}
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "TE") == 0)
		{
			out->te = (uint32_t)strtoul(ff_get_value(ctx), NULL, 10);
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Manufacture") == 0)
		{
			/* KeeLoq-family manufacturer-key name — Phase 9b carries it
			 * through load → SignalSettings → save_key_with_manufacture
			 * so the saved file's Manufacture line is preserved across
			 * round-trip edits. */
			strncpy(out->manufacture, ff_get_value(ctx),
			        FLIPPER_SUBGHZ_MANUF_MAX_LEN - 1);
			out->manufacture[FLIPPER_SUBGHZ_MANUF_MAX_LEN - 1] = '\0';
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "CounterMode") == 0)
		{
			/* Phase 9d-2: optional KeeLoq-family replay policy override.
			 * "Static" → STATIC replay (re-emit captured hop verbatim).
			 * Anything else (including missing field, empty string, or
			 * an unknown value such as "Bogus") falls through to the
			 * INCREMENT default that was already set by memset() at the
			 * caller — matching the historical behaviour for every
			 * existing .sub file. */
			if (subghz_strcasecmp(ff_get_value(ctx), "Static") == 0)
				out->counter_mode = FLIPPER_SUBGHZ_COUNTER_MODE_STATIC;
			else
				out->counter_mode = FLIPPER_SUBGHZ_COUNTER_MODE_INCREMENT;
		}
	}

	return (out->frequency > 0 && out->protocol[0] != '\0');
}

/*============================================================================*/
/**
 * @brief  Parse an M1 native .sgh RAW (NOISE) file body.
 *
 * M1 native RAW files store unsigned pulse durations in "Data:" lines.
 * Values are converted to alternating signed format (mark/space) so the
 * offline decode engine (which expects Flipper-style alternating int16_t)
 * can process them.  The first sample is assumed to be a mark (positive).
 */
static bool m1sgh_parse_raw(flipper_file_t *ctx, flipper_subghz_signal_t *out)
{
	int32_t line_samples[SUBGHZ_LINE_SAMPLES_MAX];
	uint16_t line_count;
	uint16_t i;
	bool positive = true; /* first pulse assumed to be mark (high) */

	out->type = FLIPPER_SUBGHZ_TYPE_RAW;
	out->raw_count = 0;

	while (ff_read_line(ctx))
	{
		if (ff_is_separator(ctx))
			continue;

		if (!ff_parse_kv(ctx))
			continue;

		if (subghz_strcasecmp(ff_get_key(ctx), "Frequency") == 0)
		{
			out->frequency = (uint32_t)strtoul(ff_get_value(ctx), NULL, 10);
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Modulation") == 0)
		{
			/* Store modulation in preset field for info screen display */
			strncpy(out->preset, ff_get_value(ctx), FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1);
			out->preset[FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1] = '\0';
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Data") == 0)
		{
			/* Parse unsigned pulse durations; convert to alternating signed */
			line_count = ff_parse_int32_array(ff_get_value(ctx),
			                                  line_samples,
			                                  SUBGHZ_LINE_SAMPLES_MAX);

			for (i = 0; i < line_count && out->raw_count < FLIPPER_SUBGHZ_RAW_MAX_SAMPLES; i++)
			{
				int32_t val = line_samples[i];
				if (val < 0) val = -val;   /* ensure positive (M1 stores unsigned) */
				if (val > 32767) val = 32767;

				/* Alternate sign: positive = mark (high), negative = space (low) */
				out->raw_data[out->raw_count++] = positive ? (int16_t)val : -(int16_t)val;
				positive = !positive;
			}
		}
	}

	return (out->raw_count > 0);
}

/*============================================================================*/
/**
 * @brief  Parse an M1 native .sgh PACKET file body.
 *
 * M1 packet files use "Modulation:" (not Flipper's "Preset:"), "Bits:" (not
 * "Bit:"), and a "Payload:" field (0x-prefixed hex key) and "BT:" (bit time).
 * All fields are extracted so callers can use key and te from the result.
 */
static bool m1sgh_parse_packet(flipper_file_t *ctx, flipper_subghz_signal_t *out)
{
	out->type = FLIPPER_SUBGHZ_TYPE_PARSED;

	while (ff_read_line(ctx))
	{
		if (ff_is_separator(ctx))
			continue;

		if (!ff_parse_kv(ctx))
			continue;

		if (subghz_strcasecmp(ff_get_key(ctx), "Frequency") == 0)
		{
			out->frequency = (uint32_t)strtoul(ff_get_value(ctx), NULL, 10);
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Modulation") == 0)
		{
			/* M1 uses "Modulation:" where Flipper uses "Preset:" */
			strncpy(out->preset, ff_get_value(ctx), FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1);
			out->preset[FLIPPER_SUBGHZ_PRESET_MAX_LEN - 1] = '\0';
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Protocol") == 0)
		{
			strncpy(out->protocol, ff_get_value(ctx), FLIPPER_SUBGHZ_PROTO_MAX_LEN - 1);
			out->protocol[FLIPPER_SUBGHZ_PROTO_MAX_LEN - 1] = '\0';
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Bits") == 0)
		{
			/* M1 uses "Bits:" where Flipper uses "Bit:" */
			out->bit_count = (uint32_t)strtoul(ff_get_value(ctx), NULL, 10);
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "Payload") == 0)
		{
			/* 0x-prefixed big-endian hex key, e.g. "0x0000000052A12E00" */
			out->key = (uint64_t)strtoull(ff_get_value(ctx), NULL, 16);
		}
		else if (subghz_strcasecmp(ff_get_key(ctx), "BT") == 0)
		{
			/* M1 uses "BT:" (bit time) where Flipper uses "TE:" */
			out->te = (uint32_t)strtoul(ff_get_value(ctx), NULL, 10);
		}
	}

	return (out->frequency > 0);
}

/*============================================================================*/
/**
 * @brief  Load a .sub file (either RAW or Key format)
 * @param  path  file path on FatFs filesystem
 * @param  out   output signal structure
 * @return true on success
 */
bool flipper_subghz_load(const char *path, flipper_subghz_signal_t *out)
{
	flipper_file_t ff;
	bool result = false;
	char filetype_val[48]; /* saved before version read overwrites ff.value */
	char version_val[16];  /* saved before next read may overwrite it */

	if (path == NULL || out == NULL)
		return false;

	memset(out, 0, sizeof(flipper_subghz_signal_t));

	if (!ff_open(&ff, path))
		return false;

	/* Read and check Filetype line */
	if (!ff_read_line(&ff) || !ff_parse_kv(&ff))
	{
		ff_close(&ff);
		return false;
	}

	if (strcmp(ff_get_key(&ff), "Filetype") != 0)
	{
		ff_close(&ff);
		return false;
	}

	/* Save filetype value — the next read will overwrite ff.value */
	strncpy(filetype_val, ff_get_value(&ff), sizeof(filetype_val) - 1);
	filetype_val[sizeof(filetype_val) - 1] = '\0';

	/* Check Version line */
	if (!ff_read_line(&ff) || !ff_parse_kv(&ff))
	{
		ff_close(&ff);
		return false;
	}

	if (strcmp(ff_get_key(&ff), "Version") != 0)
	{
		ff_close(&ff);
		return false;
	}

	/* Save version value — ff.value may be overwritten on next read */
	strncpy(version_val, ff_get_value(&ff), sizeof(version_val) - 1);
	version_val[sizeof(version_val) - 1] = '\0';

	/* Determine file format from filetype + version strings.
	 * The cursor is now positioned after Version:, ready for body parsers.
	 * No close/reopen needed — filetype_val already carries the sub-type. */
	if (strcmp(version_val, "1") == 0 || strcmp(version_val, "2") == 0)
	{
		/* Flipper .sub format — RAW vs Key is encoded in filetype_val */
		if (strstr(filetype_val, "RAW") != NULL)
			result = subghz_parse_raw(&ff, out);
		else
			result = subghz_parse_key(&ff, out);
	}
	else if (flipper_subghz_is_m1_native_header(filetype_val, version_val))
	{
		/* M1 native .sgh format — sub-type (NOISE/PACKET) is in filetype_val */
		if (strstr(filetype_val, M1_SUBGHZ_NOISE_TYPE) != NULL)
			result = m1sgh_parse_raw(&ff, out);
		else if (strstr(filetype_val, M1_SUBGHZ_PACKET_TYPE) != NULL)
			result = m1sgh_parse_packet(&ff, out);
		/* else: unknown M1 format variant — result stays false */

		/* Mark as native so callers can bypass the conversion/temp-file path */
		if (result)
			out->is_m1_native = true;
	}

	ff_close(&ff);
	return result;
}

/*============================================================================*/
/**
 * @brief  Test whether header strings indicate an M1 native .sgh file.
 *
 * This is a pure-logic helper extracted for testability.  Flipper .sub files
 * use integer version strings ("1", "2"); M1 native .sgh files use the
 * firmware version (e.g., "0.8", "0.9") and always contain "M1 SubGHz" in
 * the Filetype value.
 *
 * @param  filetype_val  Value string from the Filetype: line
 * @param  version_val   Value string from the Version: line
 * @return true if the header indicates M1 native .sgh format
 */
bool flipper_subghz_is_m1_native_header(const char *filetype_val,
                                         const char *version_val)
{
	if (filetype_val == NULL || version_val == NULL)
		return false;

	/* Flipper format uses integer version strings "1" or "2" */
	if (strcmp(version_val, "1") == 0 || strcmp(version_val, "2") == 0)
		return false;

	/* M1 native format always has "M1 SubGHz" in the filetype */
	return strstr(filetype_val, M1_SUBGHZ_FILETYPE_PREFIX) != NULL;
}

/*============================================================================*/
/**
 * @brief  Lightweight header probe — reads only the first few header lines.
 *
 * Opens the file, reads Filetype/Version (to detect M1-native vs Flipper),
 * then scans subsequent lines for Frequency and Preset/Modulation.  Closes
 * the file immediately after; no Data: lines are loaded.
 *
 * This is used by the playlist scene so that each entry can be classified
 * without the 16 KB overhead of a full flipper_subghz_signal_t.
 *
 * @param  path  file path on FatFs filesystem
 * @param  out   probe result (set to zero on failure)
 * @return true if the file was opened and a valid frequency was found
 */
bool flipper_subghz_probe(const char *path, flipper_subghz_probe_t *out)
{
	flipper_file_t ff;
	char filetype_val[48];
	char version_val[16];
	bool done_freq = false;
	bool done_mod  = false;

	if (path == NULL || out == NULL)
		return false;

	memset(out, 0, sizeof(*out));
	out->modulation = MODULATION_OOK; /* safe default */

	if (!ff_open(&ff, path))
		return false;

	/* Filetype line */
	if (!ff_read_line(&ff) || !ff_parse_kv(&ff) ||
	    strcmp(ff_get_key(&ff), "Filetype") != 0)
	{
		ff_close(&ff);
		return false;
	}
	strncpy(filetype_val, ff_get_value(&ff), sizeof(filetype_val) - 1);
	filetype_val[sizeof(filetype_val) - 1] = '\0';

	/* Version line */
	if (!ff_read_line(&ff) || !ff_parse_kv(&ff) ||
	    strcmp(ff_get_key(&ff), "Version") != 0)
	{
		ff_close(&ff);
		return false;
	}
	strncpy(version_val, ff_get_value(&ff), sizeof(version_val) - 1);
	version_val[sizeof(version_val) - 1] = '\0';

	out->is_m1_native = flipper_subghz_is_m1_native_header(filetype_val, version_val);
	/* NOISE: M1 "NOISE" or Flipper "RAW" */
	out->is_noise = (strstr(filetype_val, M1_SUBGHZ_NOISE_TYPE) != NULL ||
	                 strstr(filetype_val, "RAW") != NULL);

	/* Scan remaining header lines for Frequency and Preset/Modulation.
	 * Stop at the first Data/RAW_Data/Key line (body data). */
	while (ff_read_line(&ff) && !(done_freq && done_mod))
	{
		if (ff_is_separator(&ff))
			continue;

		if (!ff_parse_kv(&ff))
			continue;

		const char *key = ff_get_key(&ff);
		const char *val = ff_get_value(&ff);

		if (!done_freq && subghz_strcasecmp(key, "Frequency") == 0)
		{
			out->frequency = (uint32_t)strtoul(val, NULL, 10);
			done_freq = true;
		}
		else if (!done_mod && (subghz_strcasecmp(key, "Preset") == 0 ||
		                       subghz_strcasecmp(key, "Modulation") == 0))
		{
			out->modulation = flipper_subghz_preset_to_modulation(val);
			if (out->modulation == MODULATION_UNKNOWN)
				out->modulation = MODULATION_OOK;
			done_mod = true;
		}
		else if (subghz_strcasecmp(key, "Key") == 0 ||
		         subghz_strcasecmp(key, "Data") == 0 ||
		         subghz_strcasecmp(key, "RAW_Data") == 0)
		{
			break; /* entered body — stop scanning */
		}
	}

	ff_close(&ff);
	return (out->frequency > 0);
}


bool flipper_subghz_save(const char *path, const flipper_subghz_signal_t *sig)
{
	flipper_file_t ff;
	bool result = true;

	if (path == NULL || sig == NULL)
		return false;

	if (!ff_open_write(&ff, path))
		return false;

	/* Write header */
	if (sig->type == FLIPPER_SUBGHZ_TYPE_RAW)
	{
		result = ff_write_kv_str(&ff, "Filetype", FLIPPER_SUBGHZ_RAW_FILETYPE);
	}
	else
	{
		result = ff_write_kv_str(&ff, "Filetype", FLIPPER_SUBGHZ_KEY_FILETYPE);
	}

	if (result)
		result = ff_write_kv_uint32(&ff, "Version", 1);

	if (result)
		result = ff_write_kv_uint32(&ff, "Frequency", sig->frequency);

	if (result && sig->preset[0] != '\0')
		result = ff_write_kv_str(&ff, "Preset", sig->preset);

	if (result && sig->protocol[0] != '\0')
		result = ff_write_kv_str(&ff, "Protocol", sig->protocol);

	if (result && sig->type == FLIPPER_SUBGHZ_TYPE_RAW)
	{
		/* Write RAW_Data lines, splitting at reasonable line lengths */
		uint16_t i = 0;
		char buf[FF_VALUE_MAX_LEN];

		while (i < sig->raw_count && result)
		{
			int pos = 0;
			uint16_t line_start = i;

			while (i < sig->raw_count)
			{
				int written = snprintf(&buf[pos], sizeof(buf) - (size_t)pos,
				                       "%s%d",
				                       (i > line_start) ? " " : "",
				                       (int)sig->raw_data[i]);

				if (written < 0 || (pos + written) >= (int)(sizeof(buf) - 8))
					break;

				pos += written;
				i++;
			}
			buf[pos] = '\0';

			result = ff_write_kv_str(&ff, "RAW_Data", buf);
		}
	}
	else if (result && sig->type == FLIPPER_SUBGHZ_TYPE_PARSED)
	{
		char key_str[32];

		if (result)
			result = ff_write_kv_uint32(&ff, "Bit", sig->bit_count);

		/* Write Key as 8-byte big-endian hex */
		if (result)
		{
			snprintf(key_str, sizeof(key_str),
			         "%02X %02X %02X %02X %02X %02X %02X %02X",
			         (unsigned int)((sig->key >> 56) & 0xFF),
			         (unsigned int)((sig->key >> 48) & 0xFF),
			         (unsigned int)((sig->key >> 40) & 0xFF),
			         (unsigned int)((sig->key >> 32) & 0xFF),
			         (unsigned int)((sig->key >> 24) & 0xFF),
			         (unsigned int)((sig->key >> 16) & 0xFF),
			         (unsigned int)((sig->key >> 8) & 0xFF),
			         (unsigned int)(sig->key & 0xFF));
			result = ff_write_kv_str(&ff, "Key", key_str);
		}

		if (result && sig->te > 0)
			result = ff_write_kv_uint32(&ff, "TE", sig->te);
	}

	ff_close(&ff);
	return result;
}

/*============================================================================*/
/**
 * @brief  Save an M1 native .sgh PACKET file for a parsed (key) signal.
 *
 * Writes a file that can be fully loaded by flipper_subghz_load() (which now
 * maps Payload→key and BT→te) and replayed via sub_ghz_replay_flipper_file().
 * The format uses M1-specific field names (Bits/Payload/BT) so that legacy
 * M1 tools can also read it.
 *
 * @param  path  file path on FatFs filesystem
 * @param  sig   signal structure to save (must be FLIPPER_SUBGHZ_TYPE_PARSED
 *               with bit_count > 0)
 * @return true on success
 */
bool flipper_subghz_save_m1native(const char *path, const flipper_subghz_signal_t *sig)
{
	flipper_file_t ff;
	bool result = true;
	char buf[32];

	if (path == NULL || sig == NULL)
		return false;

	/* PACKET saves are only valid for parsed/key signals with payload bits. */
	if (sig->type != FLIPPER_SUBGHZ_TYPE_PARSED || sig->bit_count == 0)
		return false;

	if (!ff_open_write(&ff, path))
		return false;

	/* Header */
	result = ff_write_kv_str(&ff, "Filetype", "M1 SubGHz PACKET");

	if (result)
		result = ff_write_kv_str(&ff, "Version", "0.9");

	if (result)
		result = ff_write_kv_uint32(&ff, "Frequency", sig->frequency);

	/* Derive modulation label from Flipper preset name */
	if (result)
	{
		const char *mod;
		if (stristr(sig->preset, "FSK"))
			mod = "FSK";
		else if (stristr(sig->preset, "ASK"))
			mod = "ASK";
		else
			mod = "OOK";   /* Default: OOK for most 433 MHz remotes */
		result = ff_write_kv_str(&ff, "Modulation", mod);
	}

	if (result && sig->protocol[0] != '\0')
		result = ff_write_kv_str(&ff, "Protocol", sig->protocol);

	if (result)
		result = ff_write_kv_uint32(&ff, "Bits", sig->bit_count);

	/* Key stored as 0x-prefixed hex (big-endian 8-byte) */
	if (result)
	{
		snprintf(buf, sizeof(buf), "0x%016llX",
		         (unsigned long long)sig->key);
		result = ff_write_kv_str(&ff, "Payload", buf);
	}

	if (result && sig->te > 0)
		result = ff_write_kv_uint32(&ff, "BT", sig->te);

	ff_close(&ff);
	return result;
}

/*============================================================================*/
/**
 * @brief  Lightweight .sub KEY save — writes from individual fields.
 *
 * Equivalent to flipper_subghz_save() for a PARSED signal but avoids the
 * need for a flipper_subghz_signal_t at the call site (and its ~16 KB
 * raw_data[] array).  All four save-path callers deal only with decoded
 * key signals and never need the raw buffer, so this is the preferred helper.
 *
 * @param  path      FatFs path for the output file
 * @param  frequency Carrier frequency in Hz
 * @param  preset    Flipper preset name (e.g. "FuriHalSubGhzPresetOok650Async")
 * @param  protocol  Protocol name string
 * @param  bit_count Number of data bits
 * @param  key       Decoded key value
 * @param  te        Timing element (0 to omit)
 * @return true on success
 */
bool flipper_subghz_save_key(const char *path, uint32_t frequency,
                              const char *preset, const char *protocol,
                              uint32_t bit_count, uint64_t key, uint32_t te)
{
	flipper_file_t ff;
	bool result = true;
	char key_str[32];

	if (path == NULL)
		return false;

	if (!ff_open_write(&ff, path))
		return false;

	result = ff_write_kv_str(&ff, "Filetype", FLIPPER_SUBGHZ_KEY_FILETYPE);

	if (result)
		result = ff_write_kv_uint32(&ff, "Version", 1);

	if (result)
		result = ff_write_kv_uint32(&ff, "Frequency", frequency);

	if (result && preset && preset[0] != '\0')
		result = ff_write_kv_str(&ff, "Preset", preset);

	if (result && protocol && protocol[0] != '\0')
		result = ff_write_kv_str(&ff, "Protocol", protocol);

	if (result)
		result = ff_write_kv_uint32(&ff, "Bit", bit_count);

	if (result)
	{
		snprintf(key_str, sizeof(key_str),
		         "%02X %02X %02X %02X %02X %02X %02X %02X",
		         (unsigned int)((key >> 56) & 0xFF),
		         (unsigned int)((key >> 48) & 0xFF),
		         (unsigned int)((key >> 40) & 0xFF),
		         (unsigned int)((key >> 32) & 0xFF),
		         (unsigned int)((key >> 24) & 0xFF),
		         (unsigned int)((key >> 16) & 0xFF),
		         (unsigned int)((key >>  8) & 0xFF),
		         (unsigned int)( key        & 0xFF));
		result = ff_write_kv_str(&ff, "Key", key_str);
	}

	if (result && te > 0)
		result = ff_write_kv_uint32(&ff, "TE", te);

	ff_close(&ff);
	return result;
}

/*============================================================================*/
/**
 * @brief  Variant of flipper_subghz_save_key() that also writes the
 *         `Manufacture:` field so the saved key can be replayed via the
 *         KeeLoq counter-mode encoder.
 *
 * Used by the Phase 8c-3 Create-from-scratch KeeLoq-family flow.  When
 * @p manufacture is NULL or empty the function behaves identically to
 * flipper_subghz_save_key() (no Manufacture line written).
 */
bool flipper_subghz_save_key_with_manufacture(const char *path,
                                               uint32_t    frequency,
                                               const char *preset,
                                               const char *protocol,
                                               uint32_t    bit_count,
                                               uint64_t    key,
                                               uint32_t    te,
                                               const char *manufacture)
{
	/* Thin wrapper kept for backwards-compatibility with Phase 9b/9c
	 * callers — defaults CounterMode to INCREMENT so no new line is
	 * written to existing files unless they go through the Phase 9d-3
	 * SignalSettings save path. */
	return flipper_subghz_save_key_full(path, frequency, preset, protocol,
	                                    bit_count, key, te, manufacture,
	                                    FLIPPER_SUBGHZ_COUNTER_MODE_INCREMENT);
}

/*============================================================================*/
/**
 * @brief  Phase 9d-2 — write a Flipper Key file with both optional
 *         `Manufacture:` and `CounterMode:` fields.
 *
 * The `CounterMode:` line is only emitted when @p counter_mode is non-default
 * (i.e. STATIC).  This keeps existing Phase 9b/9c saved files byte-identical
 * across re-saves that did not touch the counter-mode toggle.
 */
bool flipper_subghz_save_key_full(const char *path,
                                   uint32_t    frequency,
                                   const char *preset,
                                   const char *protocol,
                                   uint32_t    bit_count,
                                   uint64_t    key,
                                   uint32_t    te,
                                   const char *manufacture,
                                   flipper_subghz_counter_mode_t counter_mode)
{
	flipper_file_t ff;
	bool result = true;
	char key_str[32];

	if (path == NULL)
		return false;

	if (!ff_open_write(&ff, path))
		return false;

	result = ff_write_kv_str(&ff, "Filetype", FLIPPER_SUBGHZ_KEY_FILETYPE);

	if (result)
		result = ff_write_kv_uint32(&ff, "Version", 1);

	if (result)
		result = ff_write_kv_uint32(&ff, "Frequency", frequency);

	if (result && preset && preset[0] != '\0')
		result = ff_write_kv_str(&ff, "Preset", preset);

	if (result && protocol && protocol[0] != '\0')
		result = ff_write_kv_str(&ff, "Protocol", protocol);

	if (result)
		result = ff_write_kv_uint32(&ff, "Bit", bit_count);

	if (result)
	{
		snprintf(key_str, sizeof(key_str),
		         "%02X %02X %02X %02X %02X %02X %02X %02X",
		         (unsigned int)((key >> 56) & 0xFF),
		         (unsigned int)((key >> 48) & 0xFF),
		         (unsigned int)((key >> 40) & 0xFF),
		         (unsigned int)((key >> 32) & 0xFF),
		         (unsigned int)((key >> 24) & 0xFF),
		         (unsigned int)((key >> 16) & 0xFF),
		         (unsigned int)((key >>  8) & 0xFF),
		         (unsigned int)( key        & 0xFF));
		result = ff_write_kv_str(&ff, "Key", key_str);
	}

	if (result && te > 0)
		result = ff_write_kv_uint32(&ff, "TE", te);

	if (result && manufacture && manufacture[0] != '\0')
		result = ff_write_kv_str(&ff, "Manufacture", manufacture);

	if (result && counter_mode == FLIPPER_SUBGHZ_COUNTER_MODE_STATIC)
		result = ff_write_kv_str(&ff, "CounterMode", "Static");

	ff_close(&ff);
	return result;
}

/*============================================================================*/
/**
 * @brief  Lightweight M1 native .sgh PACKET save — writes from individual fields.
 *
 * Equivalent to flipper_subghz_save_m1native() but avoids the need for a
 * flipper_subghz_signal_t at the call site.
 *
 * @param  path      FatFs path for the output file
 * @param  frequency Carrier frequency in Hz
 * @param  preset    Flipper preset name (used to derive OOK/ASK/FSK label)
 * @param  protocol  Protocol name string
 * @param  bit_count Number of data bits
 * @param  key       Decoded key value
 * @param  te        Timing element (0 to omit)
 * @return true on success
 */
bool flipper_subghz_save_m1native_key(const char *path, uint32_t frequency,
                                       const char *preset, const char *protocol,
                                       uint32_t bit_count, uint64_t key, uint32_t te)
{
	flipper_file_t ff;
	bool result = true;
	char buf[32];

	if (path == NULL || bit_count == 0)
		return false;

	if (!ff_open_write(&ff, path))
		return false;

	result = ff_write_kv_str(&ff, "Filetype", "M1 SubGHz PACKET");

	if (result)
		result = ff_write_kv_str(&ff, "Version", "0.9");

	if (result)
		result = ff_write_kv_uint32(&ff, "Frequency", frequency);

	if (result)
	{
		const char *mod;
		if (preset && stristr(preset, "FSK"))
			mod = "FSK";
		else if (preset && stristr(preset, "ASK"))
			mod = "ASK";
		else
			mod = "OOK";
		result = ff_write_kv_str(&ff, "Modulation", mod);
	}

	if (result && protocol && protocol[0] != '\0')
		result = ff_write_kv_str(&ff, "Protocol", protocol);

	if (result)
		result = ff_write_kv_uint32(&ff, "Bits", bit_count);

	if (result)
	{
		snprintf(buf, sizeof(buf), "0x%016llX", (unsigned long long)key);
		result = ff_write_kv_str(&ff, "Payload", buf);
	}

	if (result && te > 0)
		result = ff_write_kv_uint32(&ff, "BT", te);

	ff_close(&ff);
	return result;
}

/*============================================================================*/
/** @brief  Case-insensitive substring search (like POSIX strcasestr). */
static const char *stristr(const char *haystack, const char *needle)
{
	if (!needle[0]) return haystack;
	for (; *haystack; haystack++)
	{
		const char *h = haystack, *n = needle;
		while (*h && *n && (tolower((unsigned char)*h) == tolower((unsigned char)*n)))
		{
			h++;
			n++;
		}
		if (!*n) return haystack;
	}
	return NULL;
}

/*============================================================================*/
/**
 * @brief  Map Flipper preset name to M1 modulation type
 * @param  preset  Flipper preset string (e.g., "FuriHalSubGhzPresetOok650Async")
 * @return M1 modulation type (S_M1_SubGHz_Modulation enum value)
 */
uint8_t flipper_subghz_preset_to_modulation(const char *preset)
{
	if (preset == NULL)
		return (uint8_t)MODULATION_UNKNOWN;

	/* OOK presets — e.g. FuriHalSubGhzPresetOok650Async */
	if (stristr(preset, "OOK"))
		return (uint8_t)MODULATION_OOK;

	/* ASK presets */
	if (stristr(preset, "ASK"))
		return (uint8_t)MODULATION_ASK;

	/* FSK presets — matches 2FSK, GFSK, 4FSK, etc.
	 * e.g. FuriHalSubGhzPreset2FSKDev238Async */
	if (stristr(preset, "FSK"))
		return (uint8_t)MODULATION_FSK;

	return (uint8_t)MODULATION_UNKNOWN;
}

/*============================================================================*/
/**
 * @brief  Convert frequency in Hz to closest M1 SI4463 band enum
 * @param  freq_hz  frequency in Hz (e.g., 433920000)
 * @return M1 Sub-GHz band enum value (S_M1_SubGHz_Band)
 */
uint8_t flipper_subghz_freq_to_band(uint32_t freq_hz)
{
	/* Convert to MHz for comparison */
	uint32_t freq_mhz = freq_hz / 1000000;

	if (freq_mhz <= 305)
		return (uint8_t)SUB_GHZ_BAND_300;
	else if (freq_mhz <= 312)
		return (uint8_t)SUB_GHZ_BAND_310;
	else if (freq_mhz <= 320)
		return (uint8_t)SUB_GHZ_BAND_315;
	else if (freq_mhz <= 358)
		return (uint8_t)SUB_GHZ_BAND_345;
	else if (freq_mhz <= 381)
		return (uint8_t)SUB_GHZ_BAND_372;
	else if (freq_mhz <= 412)
		return (uint8_t)SUB_GHZ_BAND_390;
	else if (freq_mhz <= 433)
		return (uint8_t)SUB_GHZ_BAND_433;
	else if (freq_mhz <= 500)
		return (uint8_t)SUB_GHZ_BAND_433_92;
	else
		return (uint8_t)SUB_GHZ_BAND_915;
}
