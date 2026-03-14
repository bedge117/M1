/* See COPYING.txt for license details. */

/*
*
*  m1_elf_loader.c
*
*  ELF32 loader for M1 external apps (.m1app files)
*
*  Loads relocatable ELF32 ARM Thumb executables from SD card into RAM,
*  patches relocations, resolves external symbols against firmware API
*  table, and prepares the entry point for execution.
*
* M1 Project
*
*/

/*************************** I N C L U D E S **********************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "m1_elf_loader.h"
#include "m1_log_debug.h"
#include "ff.h"

/*************************** D E F I N E S ************************************/

#define TAG "ELF"

/* ELF magic bytes */
#define ELFMAG0 0x7F
#define ELFMAG1 'E'
#define ELFMAG2 'L'
#define ELFMAG3 'F'

/* ELF ident indices */
#define EI_CLASS   4
#define EI_DATA    5
#define EI_VERSION 6

#define ELFCLASS32 1
#define ELFDATA2LSB 1

/* Section name matching buffer */
#define SHSTRTAB_MAX_SIZE  512

/* Temporary read buffer for file I/O */
#define FILE_READ_BUF_SIZE 256

/********************* F U N C T I O N   P R O T O T Y P E S ******************/

static bool elf_validate_header(const Elf32_Ehdr *ehdr);
static elf_load_status_t elf_read_sections(FIL *fp, elf_app_t *app);
static elf_load_status_t elf_load_section_data(FIL *fp, elf_app_t *app);
static elf_load_status_t elf_find_manifest(elf_app_t *app);
static elf_load_status_t elf_process_relocations(FIL *fp, elf_app_t *app,
                                                  const m1_api_interface_t *api);
static elf_load_status_t elf_relocate_section(FIL *fp, elf_app_t *app,
                                               uint16_t rel_sec_idx,
                                               uint16_t target_sec_idx,
                                               const m1_api_interface_t *api);
static void *elf_resolve_symbol(const char *name, const m1_api_interface_t *api);
static bool elf_apply_relocation(uint8_t *target_base, uint32_t offset,
                                  uint8_t rel_type, uint32_t sym_addr,
                                  uint32_t patch_addr);

/*************** F U N C T I O N   I M P L E M E N T A T I O N ****************/


/*============================================================================*/
/*
 * @brief  Compute GNU hash of a symbol name
 * @param  name  Null-terminated symbol name string
 * @retval uint32_t  Hash value
 */
/*============================================================================*/
uint32_t elf_gnu_hash(const char *name)
{
    uint32_t h = 5381;
    const uint8_t *p = (const uint8_t *)name;

    while (*p)
    {
        h = (h << 5) + h + *p;
        p++;
    }
    return h;
}


/*============================================================================*/
/*
 * @brief  Validate ELF header for ARM 32-bit little-endian relocatable
 * @param  ehdr  Pointer to ELF header
 * @retval bool  true if valid
 */
/*============================================================================*/
static bool elf_validate_header(const Elf32_Ehdr *ehdr)
{
    /* Check magic */
    if (ehdr->e_ident[0] != ELFMAG0 ||
        ehdr->e_ident[1] != ELFMAG1 ||
        ehdr->e_ident[2] != ELFMAG2 ||
        ehdr->e_ident[3] != ELFMAG3)
    {
        M1_LOG_E(TAG, "Bad ELF magic");
        return false;
    }

    /* Check class (32-bit) */
    if (ehdr->e_ident[EI_CLASS] != ELFCLASS32)
    {
        M1_LOG_E(TAG, "Not ELF32");
        return false;
    }

    /* Check endianness (little-endian) */
    if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
    {
        M1_LOG_E(TAG, "Not little-endian");
        return false;
    }

    /* Check machine type (ARM) */
    if (ehdr->e_machine != EM_ARM)
    {
        M1_LOG_E(TAG, "Not ARM (machine=%u)", ehdr->e_machine);
        return false;
    }

    /* Must be relocatable */
    if (ehdr->e_type != ET_REL)
    {
        M1_LOG_E(TAG, "Not relocatable (type=%u)", ehdr->e_type);
        return false;
    }

    /* Must have section headers */
    if (ehdr->e_shoff == 0 || ehdr->e_shnum == 0)
    {
        M1_LOG_E(TAG, "No section headers");
        return false;
    }

    return true;
}


/*============================================================================*/
/*
 * @brief  Read and categorize section headers from the ELF file
 * @param  fp   Open FatFS file handle
 * @param  app  App context to populate
 * @retval elf_load_status_t
 */
/*============================================================================*/
static elf_load_status_t elf_read_sections(FIL *fp, elf_app_t *app)
{
    Elf32_Shdr shdr;
    UINT br;
    FRESULT res;
    uint16_t i;
    uint8_t shstrtab[SHSTRTAB_MAX_SIZE];
    uint32_t shstrtab_size = 0;

    /* Limit sections we track */
    uint16_t num_sh = app->ehdr.e_shnum;
    if (num_sh > ELF_MAX_SECTIONS)
    {
        M1_LOG_W(TAG, "Too many sections (%u), clamping to %u", num_sh, ELF_MAX_SECTIONS);
        num_sh = ELF_MAX_SECTIONS;
    }

    app->num_sections = num_sh;
    app->symtab_idx = 0;
    app->strtab_idx = 0;
    app->meta_idx = 0;

    /* First pass: read all section headers and identify shstrtab */
    if (app->ehdr.e_shstrndx < num_sh)
    {
        /* Read the shstrtab section header to get its offset and size */
        uint32_t shstr_hdr_off = app->ehdr.e_shoff +
                                  (uint32_t)app->ehdr.e_shstrndx * app->ehdr.e_shentsize;
        res = f_lseek(fp, shstr_hdr_off);
        if (res != FR_OK)
            return ELF_ERR_FILE;

        res = f_read(fp, &shdr, sizeof(Elf32_Shdr), &br);
        if (res != FR_OK || br != sizeof(Elf32_Shdr))
            return ELF_ERR_FILE;

        shstrtab_size = shdr.sh_size;
        if (shstrtab_size > SHSTRTAB_MAX_SIZE)
            shstrtab_size = SHSTRTAB_MAX_SIZE;

        /* Read the shstrtab data */
        res = f_lseek(fp, shdr.sh_offset);
        if (res != FR_OK)
            return ELF_ERR_FILE;

        res = f_read(fp, shstrtab, shstrtab_size, &br);
        if (res != FR_OK || br != shstrtab_size)
            return ELF_ERR_FILE;
    }

    /* Second pass: read each section header and categorize */
    for (i = 0; i < num_sh; i++)
    {
        uint32_t hdr_off = app->ehdr.e_shoff + (uint32_t)i * app->ehdr.e_shentsize;

        res = f_lseek(fp, hdr_off);
        if (res != FR_OK)
            return ELF_ERR_FILE;

        res = f_read(fp, &shdr, sizeof(Elf32_Shdr), &br);
        if (res != FR_OK || br != sizeof(Elf32_Shdr))
            return ELF_ERR_FILE;

        app->sections[i].file_offset = shdr.sh_offset;
        app->sections[i].size = shdr.sh_size;
        app->sections[i].type = shdr.sh_type;
        app->sections[i].flags = shdr.sh_flags;
        app->sections[i].align = shdr.sh_addralign;
        app->sections[i].rel_sec_idx = 0;
        app->sections[i].data = NULL;

        /* Identify special sections by type */
        if (shdr.sh_type == SHT_SYMTAB && app->symtab_idx == 0)
        {
            app->symtab_idx = i;
            /* The linked strtab is in sh_link */
            if (shdr.sh_link < num_sh)
            {
                app->strtab_idx = (uint16_t)shdr.sh_link;
            }
        }

        /* Identify .m1meta by name */
        if (shstrtab_size > 0 && shdr.sh_name < shstrtab_size)
        {
            const char *sec_name = (const char *)&shstrtab[shdr.sh_name];
            if (strcmp(sec_name, ".m1meta") == 0)
            {
                app->meta_idx = i;
            }
        }

        /* Link .rel sections to their target sections */
        if (shdr.sh_type == SHT_REL)
        {
            uint16_t target = (uint16_t)shdr.sh_info;
            if (target < num_sh)
            {
                app->sections[target].rel_sec_idx = i;
            }
        }
    }

    return ELF_OK;
}


/*============================================================================*/
/*
 * @brief  Allocate RAM and load data for ALLOC sections
 * @param  fp   Open FatFS file handle
 * @param  app  App context with section info populated
 * @retval elf_load_status_t
 */
/*============================================================================*/
static elf_load_status_t elf_load_section_data(FIL *fp, elf_app_t *app)
{
    uint16_t i;
    UINT br;
    FRESULT res;

    for (i = 0; i < app->num_sections; i++)
    {
        elf_section_t *sec = &app->sections[i];

        /* Only allocate for sections that need memory (ALLOC flag set) */
        if (!(sec->flags & SHF_ALLOC))
            continue;

        if (sec->size == 0)
            continue;

        /* Allocate RAM for this section */
        sec->data = (uint8_t *)pvPortMalloc(sec->size);
        if (sec->data == NULL)
        {
            M1_LOG_E(TAG, "OOM: section %u needs %lu bytes (free=%lu)",
                     i, (unsigned long)sec->size,
                     (unsigned long)xPortGetFreeHeapSize());
            return ELF_ERR_NO_MEMORY;
        }

        if (sec->type == SHT_NOBITS)
        {
            /* BSS — zero-fill */
            memset(sec->data, 0, sec->size);
        }
        else if (sec->type == SHT_PROGBITS || sec->type == SHT_REL)
        {
            /* Read section data from file */
            res = f_lseek(fp, sec->file_offset);
            if (res != FR_OK)
            {
                M1_LOG_E(TAG, "Seek fail sec %u off %lu", i, (unsigned long)sec->file_offset);
                return ELF_ERR_FILE;
            }

            res = f_read(fp, sec->data, sec->size, &br);
            if (res != FR_OK || br != sec->size)
            {
                M1_LOG_E(TAG, "Read fail sec %u (%lu/%lu)",
                         i, (unsigned long)br, (unsigned long)sec->size);
                return ELF_ERR_FILE;
            }
        }
        else
        {
            /* Other types with ALLOC — read from file */
            res = f_lseek(fp, sec->file_offset);
            if (res != FR_OK)
                return ELF_ERR_FILE;

            res = f_read(fp, sec->data, sec->size, &br);
            if (res != FR_OK || br != sec->size)
                return ELF_ERR_FILE;
        }
    }

    /* Also allocate and load symtab and strtab (not ALLOC, but needed for relocation) */
    if (app->symtab_idx != 0)
    {
        elf_section_t *sym_sec = &app->sections[app->symtab_idx];
        if (sym_sec->data == NULL && sym_sec->size > 0)
        {
            sym_sec->data = (uint8_t *)pvPortMalloc(sym_sec->size);
            if (sym_sec->data == NULL)
                return ELF_ERR_NO_MEMORY;

            res = f_lseek(fp, sym_sec->file_offset);
            if (res != FR_OK)
                return ELF_ERR_FILE;

            res = f_read(fp, sym_sec->data, sym_sec->size, &br);
            if (res != FR_OK || br != sym_sec->size)
                return ELF_ERR_FILE;
        }
    }

    if (app->strtab_idx != 0)
    {
        elf_section_t *str_sec = &app->sections[app->strtab_idx];
        if (str_sec->data == NULL && str_sec->size > 0)
        {
            str_sec->data = (uint8_t *)pvPortMalloc(str_sec->size);
            if (str_sec->data == NULL)
                return ELF_ERR_NO_MEMORY;

            res = f_lseek(fp, str_sec->file_offset);
            if (res != FR_OK)
                return ELF_ERR_FILE;

            res = f_read(fp, str_sec->data, str_sec->size, &br);
            if (res != FR_OK || br != str_sec->size)
                return ELF_ERR_FILE;
        }
    }

    return ELF_OK;
}


/*============================================================================*/
/*
 * @brief  Find and validate the .m1meta manifest section
 * @param  app  App context
 * @retval elf_load_status_t
 */
/*============================================================================*/
static elf_load_status_t elf_find_manifest(elf_app_t *app)
{
    elf_section_t *meta;

    if (app->meta_idx == 0)
    {
        M1_LOG_W(TAG, "No .m1meta section — using defaults");
        /* Fill in default manifest */
        memset(&app->manifest, 0, sizeof(m1_app_manifest_t));
        app->manifest.magic = M1_APP_MANIFEST_MAGIC;
        app->manifest.api_version = M1_APP_API_VERSION;
        app->manifest.stack_size = 2048; /* 2048 words = 8KB default */
        strncpy(app->manifest.name, "Unknown", sizeof(app->manifest.name) - 1);
        return ELF_OK;
    }

    meta = &app->sections[app->meta_idx];

    if (meta->data == NULL || meta->size < sizeof(m1_app_manifest_t))
    {
        M1_LOG_E(TAG, ".m1meta too small (%lu)", (unsigned long)meta->size);
        return ELF_ERR_MANIFEST;
    }

    memcpy(&app->manifest, meta->data, sizeof(m1_app_manifest_t));

    if (app->manifest.magic != M1_APP_MANIFEST_MAGIC)
    {
        M1_LOG_E(TAG, "Bad manifest magic 0x%08lX", (unsigned long)app->manifest.magic);
        return ELF_ERR_MANIFEST;
    }

    if (app->manifest.api_version > M1_APP_API_VERSION)
    {
        M1_LOG_E(TAG, "API version %u > %u", app->manifest.api_version, M1_APP_API_VERSION);
        return ELF_ERR_API_VERSION;
    }

    /* Ensure name is null-terminated */
    app->manifest.name[sizeof(app->manifest.name) - 1] = '\0';

    /* Validate stack size — enforce sane bounds */
    if (app->manifest.stack_size < 256)
        app->manifest.stack_size = 256;
    if (app->manifest.stack_size > 16384)
        app->manifest.stack_size = 16384;

    M1_LOG_I(TAG, "Manifest: name=\"%s\" api=%u stack=%u words",
             app->manifest.name, app->manifest.api_version, app->manifest.stack_size);

    return ELF_OK;
}


/*============================================================================*/
/*
 * @brief  Resolve an external symbol by name against the firmware API table
 * @param  name  Symbol name string
 * @param  api   API interface with sorted hash table
 * @retval void* Address of the symbol, or NULL if not found
 */
/*============================================================================*/
static void *elf_resolve_symbol(const char *name, const m1_api_interface_t *api)
{
    uint32_t hash = elf_gnu_hash(name);
    int lo = 0;
    int hi = (int)api->count - 1;

    /* Binary search on sorted hash table */
    while (lo <= hi)
    {
        int mid = lo + (hi - lo) / 2;
        if (api->table[mid].hash == hash)
        {
            return api->table[mid].address;
        }
        else if (api->table[mid].hash < hash)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid - 1;
        }
    }

    return NULL;
}


/*============================================================================*/
/*
 * @brief  Apply a single ARM relocation
 * @param  target_base  Base address of the section being relocated
 * @param  offset       Offset within the section of the relocation site
 * @param  rel_type     ARM relocation type
 * @param  sym_addr     Resolved symbol address
 * @param  patch_addr   Absolute address of the relocation site in RAM
 * @retval bool         true on success
 */
/*============================================================================*/
static bool elf_apply_relocation(uint8_t *target_base, uint32_t offset,
                                  uint8_t rel_type, uint32_t sym_addr,
                                  uint32_t patch_addr)
{
    uint32_t *loc32 = (uint32_t *)(target_base + offset);

    switch (rel_type)
    {
        case R_ARM_ABS32:
        case R_ARM_TARGET1:
        {
            /* S + A */
            *loc32 += sym_addr;
            break;
        }

        case R_ARM_REL32:
        {
            /* S + A - P */
            *loc32 += sym_addr - patch_addr;
            break;
        }

        case R_ARM_THM_CALL:
        case R_ARM_THM_JUMP24:
        {
            /*
             * Thumb BL/B.W encoding:
             *   First halfword:  [15:11] = 11110, [10] = S, [9:0] = imm10
             *   Second halfword: [15:14] = 11, [13] = J1, [12] = J2, [11] = 0/1 (BL vs B.W),
             *                    [10:0] = imm11
             *
             * offset = SignExtend(S:I1:I2:imm10:imm11:0, 25)
             * where I1 = NOT(J1 XOR S), I2 = NOT(J2 XOR S)
             */
            uint16_t *hw = (uint16_t *)(target_base + offset);
            uint16_t upper = hw[0];
            uint16_t lower = hw[1];

            /* Decode existing offset */
            uint32_t s_bit  = (upper >> 10) & 1;
            uint32_t imm10  = upper & 0x3FF;
            uint32_t j1     = (lower >> 13) & 1;
            uint32_t j2     = (lower >> 11) & 1;
            uint32_t imm11  = lower & 0x7FF;

            uint32_t i1 = ~(j1 ^ s_bit) & 1;
            uint32_t i2 = ~(j2 ^ s_bit) & 1;

            int32_t existing_offset = (int32_t)(
                (s_bit << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1)
            );
            /* Sign extend from bit 24 */
            if (existing_offset & (1 << 24))
                existing_offset |= (int32_t)0xFE000000;

            /* Compute new target: S + A - P */
            int32_t new_offset = (int32_t)sym_addr + existing_offset - (int32_t)patch_addr;

            /* Check range: +/- 16MB for Thumb BL/B.W */
            if (new_offset > 0x00FFFFFF || new_offset < (int32_t)0xFF000000)
            {
                M1_LOG_W(TAG, "THM_CALL out of range: offset=%ld", (long)new_offset);
                /* Out of range — cannot patch. Skip without failing. */
                break;
            }

            /* Re-encode */
            uint32_t new_s  = (new_offset >> 24) & 1;
            uint32_t new_i1 = (new_offset >> 23) & 1;
            uint32_t new_i2 = (new_offset >> 22) & 1;
            uint32_t new_imm10 = (new_offset >> 12) & 0x3FF;
            uint32_t new_imm11 = (new_offset >> 1) & 0x7FF;

            uint32_t new_j1 = (~(new_i1 ^ new_s)) & 1;
            uint32_t new_j2 = (~(new_i2 ^ new_s)) & 1;

            hw[0] = (upper & 0xF800) | (uint16_t)((new_s << 10) | new_imm10);
            hw[1] = (lower & 0xD000) | (uint16_t)((new_j1 << 13) | (new_j2 << 11) | new_imm11);
            break;
        }

        case R_ARM_THM_MOVW_ABS_NC:
        {
            /*
             * Thumb MOVW encoding:
             *   First halfword:  [15:12] = 1111_0, [11] = i, [10] = 10, [9:8] = 01/00,
             *                    [7:4] = imm4, [3:0] = ...
             *   Second halfword: [15] = 0, [14:12] = imm3, [11:8] = Rd, [7:0] = imm8
             *
             * imm16 = imm4:i:imm3:imm8
             */
            uint16_t *hw = (uint16_t *)(target_base + offset);
            uint16_t upper = hw[0];
            uint16_t lower = hw[1];

            /* Extract existing imm16 */
            uint32_t imm4 = upper & 0x000F;
            uint32_t i_bit = (upper >> 10) & 1;
            uint32_t imm3 = (lower >> 12) & 0x07;
            uint32_t imm8 = lower & 0x00FF;
            uint32_t existing_imm16 = (imm4 << 12) | (i_bit << 11) | (imm3 << 8) | imm8;

            /* Add symbol address (lower 16 bits) */
            uint32_t new_val = (existing_imm16 + sym_addr) & 0xFFFF;

            /* Re-encode */
            uint32_t new_imm4 = (new_val >> 12) & 0x0F;
            uint32_t new_i    = (new_val >> 11) & 1;
            uint32_t new_imm3 = (new_val >> 8) & 0x07;
            uint32_t new_imm8 = new_val & 0xFF;

            hw[0] = (upper & 0xFBF0) | (uint16_t)((new_i << 10) | new_imm4);
            hw[1] = (lower & 0x8F00) | (uint16_t)((new_imm3 << 12) | new_imm8);
            break;
        }

        case R_ARM_THM_MOVT_ABS:
        {
            /* Same encoding as MOVW but uses upper 16 bits of symbol */
            uint16_t *hw = (uint16_t *)(target_base + offset);
            uint16_t upper = hw[0];
            uint16_t lower = hw[1];

            uint32_t imm4 = upper & 0x000F;
            uint32_t i_bit = (upper >> 10) & 1;
            uint32_t imm3 = (lower >> 12) & 0x07;
            uint32_t imm8 = lower & 0x00FF;
            uint32_t existing_imm16 = (imm4 << 12) | (i_bit << 11) | (imm3 << 8) | imm8;

            /* Add symbol address upper 16 bits */
            uint32_t new_val = (existing_imm16 + (sym_addr >> 16)) & 0xFFFF;

            uint32_t new_imm4 = (new_val >> 12) & 0x0F;
            uint32_t new_i    = (new_val >> 11) & 1;
            uint32_t new_imm3 = (new_val >> 8) & 0x07;
            uint32_t new_imm8 = new_val & 0xFF;

            hw[0] = (upper & 0xFBF0) | (uint16_t)((new_i << 10) | new_imm4);
            hw[1] = (lower & 0x8F00) | (uint16_t)((new_imm3 << 12) | new_imm8);
            break;
        }

        default:
            M1_LOG_W(TAG, "Unsupported relocation type %u at offset 0x%lX",
                     rel_type, (unsigned long)offset);
            return false;
    }

    return true;
}


/*============================================================================*/
/*
 * @brief  Process all relocation sections in the ELF
 * @param  fp   Open FatFS file handle
 * @param  app  App context
 * @param  api  Firmware API interface
 * @retval elf_load_status_t
 */
/*============================================================================*/
static elf_load_status_t elf_process_relocations(FIL *fp, elf_app_t *app,
                                                  const m1_api_interface_t *api)
{
    uint16_t i;
    elf_load_status_t status;

    for (i = 0; i < app->num_sections; i++)
    {
        if (app->sections[i].type == SHT_REL && app->sections[i].size > 0)
        {
            /* Find the target section (sh_info from the original header) */
            uint16_t target_idx = 0;
            uint16_t j;

            /* The target is whichever section lists this as its rel_sec_idx */
            for (j = 0; j < app->num_sections; j++)
            {
                if (app->sections[j].rel_sec_idx == i)
                {
                    target_idx = j;
                    break;
                }
            }

            if (target_idx == 0 || app->sections[target_idx].data == NULL)
            {
                /* Skip relocations for sections we didn't load */
                continue;
            }

            status = elf_relocate_section(fp, app, i, target_idx, api);
            if (status != ELF_OK)
                return status;
        }
    }

    return ELF_OK;
}


/*============================================================================*/
/*
 * @brief  Process relocations for a single section
 * @param  fp              Open FatFS file handle
 * @param  app             App context
 * @param  rel_sec_idx     Index of the .rel section
 * @param  target_sec_idx  Index of the section being relocated
 * @param  api             Firmware API interface
 * @retval elf_load_status_t
 */
/*============================================================================*/
static elf_load_status_t elf_relocate_section(FIL *fp, elf_app_t *app,
                                               uint16_t rel_sec_idx,
                                               uint16_t target_sec_idx,
                                               const m1_api_interface_t *api)
{
    elf_section_t *rel_sec = &app->sections[rel_sec_idx];
    elf_section_t *target_sec = &app->sections[target_sec_idx];
    elf_section_t *sym_sec = (app->symtab_idx != 0) ? &app->sections[app->symtab_idx] : NULL;
    elf_section_t *str_sec = (app->strtab_idx != 0) ? &app->sections[app->strtab_idx] : NULL;

    uint32_t num_rels;
    uint32_t r;
    Elf32_Rel rel_entry;
    UINT br;
    FRESULT res;

    if (sym_sec == NULL || sym_sec->data == NULL)
    {
        M1_LOG_E(TAG, "No symtab for relocations");
        return ELF_ERR_FORMAT;
    }

    num_rels = rel_sec->size / sizeof(Elf32_Rel);

    /* Read relocation entries from file (rel sections are not ALLOC, so not loaded into RAM) */
    res = f_lseek(fp, rel_sec->file_offset);
    if (res != FR_OK)
        return ELF_ERR_FILE;

    for (r = 0; r < num_rels; r++)
    {
        res = f_read(fp, &rel_entry, sizeof(Elf32_Rel), &br);
        if (res != FR_OK || br != sizeof(Elf32_Rel))
            return ELF_ERR_FILE;

        uint32_t sym_idx  = ELF32_R_SYM(rel_entry.r_info);
        uint8_t  rel_type = ELF32_R_TYPE(rel_entry.r_info);

        /* Look up the symbol */
        Elf32_Sym *sym_table = (Elf32_Sym *)sym_sec->data;
        uint32_t num_syms = sym_sec->size / sizeof(Elf32_Sym);

        if (sym_idx >= num_syms)
        {
            M1_LOG_E(TAG, "Symbol index %lu out of range (%lu)",
                     (unsigned long)sym_idx, (unsigned long)num_syms);
            return ELF_ERR_SYMBOL;
        }

        Elf32_Sym *sym = &sym_table[sym_idx];
        uint32_t sym_addr = 0;

        if (sym->st_shndx == SHN_UNDEF)
        {
            /* External symbol — resolve via API table */
            const char *sym_name = "";
            if (str_sec != NULL && str_sec->data != NULL && sym->st_name < str_sec->size)
            {
                sym_name = (const char *)&str_sec->data[sym->st_name];
            }

            void *resolved = elf_resolve_symbol(sym_name, api);
            if (resolved == NULL)
            {
                uint8_t bind = ELF32_ST_BIND(sym->st_info);
                if (bind == STB_WEAK)
                {
                    /* Weak symbol — resolve to 0 */
                    sym_addr = 0;
                }
                else
                {
                    M1_LOG_E(TAG, "Unresolved symbol: %s", sym_name);
                    return ELF_ERR_SYMBOL;
                }
            }
            else
            {
                sym_addr = (uint32_t)resolved;
            }
        }
        else if (sym->st_shndx == SHN_ABS)
        {
            /* Absolute symbol */
            sym_addr = sym->st_value;
        }
        else if (sym->st_shndx < app->num_sections)
        {
            /* Symbol defined in a loaded section */
            elf_section_t *def_sec = &app->sections[sym->st_shndx];
            if (def_sec->data != NULL)
            {
                sym_addr = (uint32_t)def_sec->data + sym->st_value;
            }
            else
            {
                M1_LOG_W(TAG, "Symbol in unloaded section %u", sym->st_shndx);
                sym_addr = sym->st_value;
            }
        }

        /* Compute the absolute address of the patch site */
        uint32_t patch_addr = (uint32_t)target_sec->data + rel_entry.r_offset;

        /* Apply the relocation */
        if (!elf_apply_relocation(target_sec->data, rel_entry.r_offset,
                                   rel_type, sym_addr, patch_addr))
        {
            /* Non-fatal — some relocation types may be unsupported but non-critical */
            M1_LOG_W(TAG, "Relocation warning at offset 0x%lX type %u",
                     (unsigned long)rel_entry.r_offset, rel_type);
        }
    }

    return ELF_OK;
}


/*============================================================================*/
/*
 * @brief  Load an M1 app from an ELF file on SD card
 * @param  path  FatFS path to the .m1app file (e.g. "0:/apps/game.m1app")
 * @param  app   App context to populate (caller must provide storage)
 * @param  api   Firmware API interface for symbol resolution
 * @retval elf_load_status_t
 */
/*============================================================================*/
elf_load_status_t elf_load_app(const char *path, elf_app_t *app,
                                const m1_api_interface_t *api)
{
    FIL fp;
    FRESULT res;
    UINT br;
    elf_load_status_t status;

    memset(app, 0, sizeof(elf_app_t));

    M1_LOG_I(TAG, "Loading: %s (heap=%lu)",
             path, (unsigned long)xPortGetFreeHeapSize());

    /* Open the ELF file */
    res = f_open(&fp, path, FA_READ);
    if (res != FR_OK)
    {
        M1_LOG_E(TAG, "Cannot open %s (err=%u)", path, res);
        return ELF_ERR_FILE;
    }

    /* Read ELF header */
    res = f_read(&fp, &app->ehdr, sizeof(Elf32_Ehdr), &br);
    if (res != FR_OK || br != sizeof(Elf32_Ehdr))
    {
        M1_LOG_E(TAG, "Read header fail");
        f_close(&fp);
        return ELF_ERR_FILE;
    }

    /* Validate header */
    if (!elf_validate_header(&app->ehdr))
    {
        f_close(&fp);
        return ELF_ERR_FORMAT;
    }

    if (app->ehdr.e_machine != EM_ARM)
    {
        f_close(&fp);
        return ELF_ERR_MACHINE;
    }

    /* Read section headers */
    status = elf_read_sections(&fp, app);
    if (status != ELF_OK)
    {
        f_close(&fp);
        elf_unload_app(app);
        return status;
    }

    /* Allocate and load section data */
    status = elf_load_section_data(&fp, app);
    if (status != ELF_OK)
    {
        f_close(&fp);
        elf_unload_app(app);
        return status;
    }

    /* Find and validate manifest */
    status = elf_find_manifest(app);
    if (status != ELF_OK)
    {
        f_close(&fp);
        elf_unload_app(app);
        return status;
    }

    /* Process relocations */
    status = elf_process_relocations(&fp, app, api);
    if (status != ELF_OK)
    {
        f_close(&fp);
        elf_unload_app(app);
        return status;
    }

    f_close(&fp);

    /* Determine entry point.
     * The ELF e_entry is an offset into a section. For relocatable files,
     * it's typically 0 and the real entry is the symbol named "app_main".
     * Try to find "app_main" in the symbol table first. */
    app->entry_point = 0;

    if (app->symtab_idx != 0 && app->strtab_idx != 0)
    {
        elf_section_t *sym_sec = &app->sections[app->symtab_idx];
        elf_section_t *str_sec = &app->sections[app->strtab_idx];

        if (sym_sec->data != NULL && str_sec->data != NULL)
        {
            Elf32_Sym *syms = (Elf32_Sym *)sym_sec->data;
            uint32_t num_syms = sym_sec->size / sizeof(Elf32_Sym);
            uint32_t s;

            for (s = 0; s < num_syms; s++)
            {
                if (syms[s].st_name < str_sec->size)
                {
                    const char *name = (const char *)&str_sec->data[syms[s].st_name];
                    if (strcmp(name, "app_main") == 0)
                    {
                        uint16_t sec_idx = syms[s].st_shndx;
                        if (sec_idx < app->num_sections &&
                            app->sections[sec_idx].data != NULL)
                        {
                            app->entry_point = (uint32_t)app->sections[sec_idx].data +
                                               syms[s].st_value;
                            /* Set Thumb bit for Cortex-M */
                            app->entry_point |= 1;
                            break;
                        }
                    }
                }
            }
        }
    }

    if (app->entry_point == 0)
    {
        M1_LOG_E(TAG, "No app_main entry point found");
        elf_unload_app(app);
        return ELF_ERR_FORMAT;
    }

    app->loaded = true;

    M1_LOG_I(TAG, "Loaded OK: entry=0x%08lX heap=%lu",
             (unsigned long)app->entry_point,
             (unsigned long)xPortGetFreeHeapSize());

    return ELF_OK;
}


/*============================================================================*/
/*
 * @brief  Unload an app and free all allocated memory
 * @param  app  App context to clean up
 */
/*============================================================================*/
void elf_unload_app(elf_app_t *app)
{
    uint16_t i;

    if (app == NULL)
        return;

    for (i = 0; i < app->num_sections; i++)
    {
        if (app->sections[i].data != NULL)
        {
            vPortFree(app->sections[i].data);
            app->sections[i].data = NULL;
        }
    }

    app->loaded = false;
    app->entry_point = 0;

    M1_LOG_I(TAG, "Unloaded (heap=%lu)", (unsigned long)xPortGetFreeHeapSize());
}
