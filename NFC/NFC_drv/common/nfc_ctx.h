/* See COPYING.txt for license details. */

/*
 * nfc_ctx.h
 *
 *      Author: watlogic22
 */

#ifndef NFC_DRV_NFC_CTX_H_
#define NFC_DRV_NFC_CTX_H_

#include <stdint.h>
#include <stdbool.h>
#include "rfal_nfc.h"
#include "nfc_fileio.h"
#include "main.h"


/* ------- Size/parser line buffer defaults (redefine in project settings if needed) ------- */
#ifndef NFC_PATH_MAX
#define NFC_PATH_MAX        128
#endif

#ifndef NFC_LINE_MAX
#define NFC_LINE_MAX        256
#endif

#ifndef NFC_TITLE_MAX
#define NFC_TITLE_MAX       24
#endif

#ifndef NFC_UID_TEXT_MAX
#define NFC_UID_TEXT_MAX    32  /* "AA BB CC DD ..." */
#endif

// nfc_ctx.file.source_kind Info
#define LIVE_CARD   0   // Read card
#define LOAD_FILE   1   // Load file


#define M1NFC_FAM_CLASSIC        0
#define M1NFC_FAM_ULTRALIGHT     1
#define M1NFC_FAM_DESFIRE        2

#define M1NFC_TECH_A    0
#define M1NFC_TECH_B    1
#define M1NFC_TECH_F    2
#define M1NFC_TECH_V    3

#define NFC_T2T_NDEF_MAX_LEN   240   // Adjust later if needed


/* NFC dump workspace (size can be adjusted as needed) */
#define NFC_DUMP_BUF_SIZE    4096U                             /* Maximum dump data capacity */
#define NFC_VALID_BITS_SIZE  (NFC_DUMP_BUF_SIZE / 4U / 8U)     /* Based on minimum unit 4B (Type2 page) */
#define NFC_DUMP_MAX_UNITS      256U                             /* Maximum unit count (dump buffer / unit size) */

extern uint8_t g_nfc_dump_buf[NFC_DUMP_BUF_SIZE];
extern uint8_t g_nfc_valid_bits[NFC_VALID_BITS_SIZE];
extern uint16_t g_nfc_ntag_page_count;

/* ======================= 1) File/State ======================= */
/* State used in storage_browse() selection, extension check (.nfc), full path generation, etc. */
typedef struct {
    char     path[NFC_PATH_MAX]; /* Selected full path */
    uint8_t  uret;               /* Temporary return value (general purpose) */
    uint8_t  sys_error;          /* 0=OK, !0=error */
    uint8_t  ext_len;            /* Can be used for extension length check */
    uint8_t  ret_code;           /* For storing upper API return codes, etc. */
    bool     source_kind;
} nfc_file_state_t;

/* ======================= 2) Header (Card Identity) ======================= */
/* Core information needed for file header and UI summary: Tech/Family/UID/ATQA/SAK/ATS, etc. */
typedef struct {
    uint8_t       tech;                    /* M1NFC_TECH_A/B/F/V */
    uint8_t       family;                  /* M1NFC_FAM_CLASSIC/UL/… */

    uint8_t       uid[10];
    uint8_t       uid_len;

    /* Tech A additional information */
    struct {
        uint8_t   atqa[2];  bool has_atqa;
        uint8_t   sak;      bool has_sak;
        uint8_t   ats[32];  // default
        uint8_t   ats_len;  /* Can be 0 */
    } a;
} nfc_header_t;

/* ======================= 3) Dump Metadata ======================= */
/* Format definition and workspace pointer for body data (pages/blocks/units) */
typedef struct {
    uint16_t  unit_size;        /* Type2=4, Classic=16, others situation-dependent */
    uint32_t  unit_count;       /* Number of units allocated (maximum) */
    uint32_t  origin;           /* Starting index (usually 0) */
    uint8_t  *data;             /* Global workspace bound by nfc_alloc_dump() */
    uint8_t  *valid_bits;       /* NULL if all valid, otherwise byte-wise valid bitmap */
    uint32_t  max_seen_unit;    /* Maximum index actually observed during parsing */
    bool      has_dump;         /* Whether dump exists (for UI display) */
} nfc_dump_meta_t;

/* ======================= 4) Parser State ======================= */
/* Line-based I/O and current scan pointer — same concept as SubGhz block refill/line scan */
typedef struct {
    nfcfio_t  io;                      /* Line I/O context */
    char      line[NFC_LINE_MAX];      /* Single line buffer */
    char     *token;                   /* Token parsing pointer (temporary) */
    const char *p;                     /* Scan pointer (temporary) */
    uint8_t   parse_error;             /* 0=OK, otherwise=error code */
} nfc_parser_t;

/* ======================= 5) UI/Behavior State ======================= */
/* Runtime flags: immediate display text, emulation mode, preview count, etc. */
typedef struct {
    char    title_text[NFC_TITLE_MAX];   /* "Ultralight/NTAG", "Classic", etc. */
    char    uid_text[NFC_UID_TEXT_MAX];  /* UID HEX string "AA BB ..." */
    bool    emulate_uid_only;            /* true=UID-only emulation, false=RAW if possible */
    uint8_t preview_units;               /* Dump preview unit count (log/UI) */
} nfc_ui_state_t;

/* ======================= 6) NDEF Storage ======================= */
typedef struct {
    uint8_t  ndef[NFC_T2T_NDEF_MAX_LEN];
    uint16_t ndef_len;
    bool     valid;                  // Flag indicating if NDEF is actually filled
} nfc_t2t_info_t;

/* ======================= Top-level Runtime Context ======================= */
/* Gathers the above 5 blocks in one place for clean passing between functions */
typedef struct {
    nfc_file_state_t  file;
    nfc_header_t      head;
    nfc_dump_meta_t   dump;
    nfc_parser_t      parser;
    nfc_ui_state_t    ui;
    nfc_t2t_info_t    t2t;   // Type 2 Tag (NTAG, etc.) related information
} nfc_run_ctx_t;


/**
 * @brief nfc_ctx_module_init - Initialize module: prepare global context/mutex
 * 
 * @retval None
 */
void nfc_ctx_module_init(void);

/**
 * @brief nfc_ctx_get - Return pointer for modules that need write access
 * 
 * @retval Pointer to global NFC context
 */
nfc_run_ctx_t* nfc_ctx_get(void);

/**
 * @brief nfc_run_ctx_init - Reset context to default values
 * 
 * @param[in,out] c Pointer to NFC context structure
 * @retval None
 */
void nfc_run_ctx_init(nfc_run_ctx_t* c);

/**
 * @brief nfc_ctx_begin_live - Source switch (Live): set basic fields
 * 
 * @retval None
 */
void nfc_ctx_begin_live(void);

/**
 * @brief nfc_ctx_begin_file - Source switch (File): set basic fields
 * 
 * @param[in] fullpath Full path to NFC file
 * @retval None
 */
void nfc_ctx_begin_file(const char* fullpath);

/**
 * @brief nfc_ctx_refresh_ui - Update UI string with family/UID
 * 
 * @retval None
 */
void nfc_ctx_refresh_ui(void);

/**
 * @brief nfc_ctx_clear_dump - Clear dump metadata/pointer binding (set)/release (clear)
 * 
 * @retval None
 */
void nfc_ctx_clear_dump(void);

/**
 * @brief nfc_ctx_set_dump - Set dump metadata/pointer binding
 * 
 * @param[in] unit_size Size of each unit
 * @param[in] unit_count Total number of units
 * @param[in] origin Starting index
 * @param[in] data Pointer to dump data buffer
 * @param[in] valid_bits Pointer to valid bits bitmap
 * @param[in] max_seen_unit Maximum unit index seen
 * @param[in] has_dump Whether dump exists
 * @retval None
 */
void nfc_ctx_set_dump(uint16_t unit_size, uint32_t unit_count, uint32_t origin,
                      uint8_t* data, uint8_t* valid_bits, uint32_t max_seen_unit, bool has_dump);

/**
 * @brief FillNfcContextFromDevice - Fill nfc_ctx from device (header/UI)
 * 
 * @param[in] dev Pointer to RFAL NFC device
 * @retval 0 Success
 * @retval 1 Invalid device
 * @retval 2 Unsupported device type
 */
uint8_t FillNfcContextFromDevice(const rfalNfcDevice* dev);

/**
 * @brief nfc_classify_family_from_nfca - Weak default rule: classify Family by SAK/ATQA (redefine in project if desired)
 * 
 * @param[in] sak SAK value
 * @param[in] atqa ATQA array (2 bytes)
 * @retval Family code (M1NFC_FAM_*), 0 if unknown
 */
uint8_t nfc_classify_family_from_nfca(uint8_t sak, const uint8_t atqa[2]);

/**
 * @brief nfc_ctx_sync_emu - Reflect current nfc_ctx.head content to emulator EmuNfcA(g_emuA)
 * 
 * @retval None
 */
void nfc_ctx_sync_emu(void);

/* Type 2 Tag NDEF helpers */
/**
 * @brief nfc_ctx_clear_t2t_ndef - Clear T2T NDEF data
 * 
 * @retval None
 */
void nfc_ctx_clear_t2t_ndef(void);

/**
 * @brief nfc_ctx_set_t2t_ndef - Set T2T NDEF data
 * 
 * @param[in] buf Pointer to NDEF data buffer
 * @param[in] len Length of NDEF data
 * @retval None
 */
void nfc_ctx_set_t2t_ndef(const uint8_t *buf, uint16_t len);

/**
 * @brief nfc_ctx_get_t2t_info - Get T2T info structure pointer
 * 
 * @retval Pointer to T2T info structure
 */
const nfc_t2t_info_t * nfc_ctx_get_t2t_info(void);

/*-------------util function-----------*/
/**
 * @brief nfc_ctx_dump_t2t_ndef - Dump T2T NDEF data in hex/ASCII format
 * 
 * @retval None
 */
void nfc_ctx_dump_t2t_ndef(void);

/**
 * @brief nfc_ctx_dump_t2t_pages - Dump T2T pages in hex/ASCII format
 * 
 * @retval None
 */
void nfc_ctx_dump_t2t_pages(void);

/**
 * @brief nfc_ctx_get_t2t_page_count - Return Type 2 / NTAG page count
 * 
 * @retval Page count, or 0 if not T2T or dump invalid
 */
uint16_t nfc_ctx_get_t2t_page_count(void);

/**
 * @brief nfc_ctx_get_t2t_page - Get T2T page data
 * 
 * @param[in] pageIndex Page index to retrieve
 * @param[out] out Output buffer (4 bytes)
 * @retval true Success
 * @retval false Failed
 */
bool nfc_ctx_get_t2t_page(uint16_t pageIndex, uint8_t out[4]);

/**
 * @brief nfc_ctx_format_t2t_page_line - Format T2T page as single line string
 * 
 * @param[in] pageIndex Page index to format
 * @param[out] buf Output buffer for formatted string
 * @param[in] bufSize Size of output buffer
 * @retval true Success
 * @retval false Failed
 */
bool nfc_ctx_format_t2t_page_line(uint16_t pageIndex, char *buf, size_t bufSize);

/**
 * @brief nfc_ctx_set_t2t_version - Set T2T version information
 * 
 * @param[in] ver Pointer to version data
 * @param[in] len Length of version data
 * @retval None
 */
void nfc_ctx_set_t2t_version(const uint8_t *ver, uint8_t len);

/**
 * @brief nfc_ctx_get_t2t_version - Get T2T version information
 * 
 * @param[out] out Output buffer (8 bytes)
 * @retval Length of version data (0 if not available)
 */
uint8_t nfc_ctx_get_t2t_version(uint8_t out[8]);

/**
 * @brief nfc_ctx_set_t2t_page - Set T2T page data
 * 
 * @param[in] page Page number to write
 * @param[in] data Page data (4 bytes)
 * @retval None
 */
void nfc_ctx_set_t2t_page(uint16_t page, const uint8_t data[4]);


/* --------- Password Auth helpers (Unlock with Password) ---------- */

/** Password source */
typedef enum {
    NFC_PWD_SRC_NONE = 0,
    NFC_PWD_SRC_MANUAL,     /**< User-entered via keyboard */
    NFC_PWD_SRC_CAPTURED    /**< Captured from reader during emulation */
} nfc_pwd_source_t;

/**
 * @brief Set manual password for next tag read
 * @param[in] pwd 4-byte password
 */
void nfc_ctx_set_manual_pwd(const uint8_t pwd[4]);

/**
 * @brief Get manual password if set
 * @param[out] pwd_out 4-byte output buffer
 * @retval true Password available
 */
bool nfc_ctx_get_manual_pwd(uint8_t pwd_out[4]);

/**
 * @brief Clear manual password
 */
void nfc_ctx_clear_manual_pwd(void);

/**
 * @brief Set captured password (from reader during emulation)
 * @param[in] pwd 4-byte password
 */
void nfc_ctx_set_captured_pwd(const uint8_t pwd[4]);

/**
 * @brief Get captured password if available
 * @param[out] pwd_out 4-byte output buffer
 * @retval true Password available
 */
bool nfc_ctx_get_captured_pwd(uint8_t pwd_out[4]);

/**
 * @brief Clear captured password
 */
void nfc_ctx_clear_captured_pwd(void);

/**
 * @brief Check if any password is available (manual or captured)
 * @param[out] pwd_out 4-byte output buffer (manual takes priority)
 * @param[out] source  Source of the password (optional, NULL ok)
 * @retval true Password available
 */
bool nfc_ctx_get_best_pwd(uint8_t pwd_out[4], nfc_pwd_source_t *source);


#endif /* NFC_DRV_NFC_CTX_H_ */
