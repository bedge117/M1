/* See COPYING.txt for license details. */

/*
*
*  m1_elf_loader.h
*
*  ELF32 structures and loader API for M1 external apps
*
* M1 Project
*
*/

#ifndef M1_ELF_LOADER_H_
#define M1_ELF_LOADER_H_

#include <stdint.h>
#include <stdbool.h>

/* ---- ELF32 structures (from ELF spec) ---- */

#define EI_NIDENT 16

typedef struct {
    uint8_t  e_ident[EI_NIDENT];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry;
    uint32_t e_phoff;
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;
} Elf32_Ehdr;

typedef struct {
    uint32_t sh_name;
    uint32_t sh_type;
    uint32_t sh_flags;
    uint32_t sh_addr;
    uint32_t sh_offset;
    uint32_t sh_size;
    uint32_t sh_link;
    uint32_t sh_info;
    uint32_t sh_addralign;
    uint32_t sh_entsize;
} Elf32_Shdr;

typedef struct {
    uint32_t st_name;
    uint32_t st_value;
    uint32_t st_size;
    uint8_t  st_info;
    uint8_t  st_other;
    uint16_t st_shndx;
} Elf32_Sym;

typedef struct {
    uint32_t r_offset;
    uint32_t r_info;
} Elf32_Rel;

/* ELF constants */
#define ET_REL          1
#define ET_EXEC         2
#define EM_ARM          40
#define SHT_PROGBITS    1
#define SHT_SYMTAB      2
#define SHT_STRTAB      3
#define SHT_REL         9
#define SHT_NOBITS      8
#define SHF_ALLOC       0x02
#define SHF_EXECINSTR   0x04
#define STB_GLOBAL      1
#define STB_WEAK        2
#define SHN_UNDEF       0
#define SHN_ABS         0xFFF1

#define ELF32_R_SYM(info)   ((info) >> 8)
#define ELF32_R_TYPE(info)  ((info) & 0xFF)
#define ELF32_ST_BIND(info) ((info) >> 4)
#define ELF32_ST_TYPE(info) ((info) & 0x0F)

/* ARM relocation types */
#define R_ARM_ABS32             2
#define R_ARM_THM_CALL          10
#define R_ARM_THM_JUMP24        30
#define R_ARM_THM_MOVW_ABS_NC   47
#define R_ARM_THM_MOVT_ABS      48
#define R_ARM_TARGET1           38  /* same as ABS32 on ARM */
#define R_ARM_REL32             3

/* M1 App Manifest (embedded in .m1meta section) */
#define M1_APP_MANIFEST_MAGIC   0x4D314150  /* "M1AP" */
#define M1_APP_API_VERSION      2

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t api_version;
    uint16_t stack_size;     /* in 32-bit words */
    char     name[32];
} m1_app_manifest_t;

/* Loaded section info */
#define ELF_MAX_SECTIONS  16

typedef struct {
    uint32_t file_offset;
    uint32_t size;
    uint32_t type;       /* SHT_PROGBITS, SHT_NOBITS, etc. */
    uint32_t flags;
    uint32_t align;
    uint32_t rel_sec_idx; /* index of matching .rel section, or 0 */
    uint8_t *data;        /* allocated RAM for this section */
} elf_section_t;

/* Loaded app context */
typedef struct {
    Elf32_Ehdr     ehdr;
    elf_section_t  sections[ELF_MAX_SECTIONS];
    uint16_t       num_sections;
    uint16_t       symtab_idx;    /* section index of .symtab */
    uint16_t       strtab_idx;    /* section index of .strtab */
    uint16_t       meta_idx;      /* section index of .m1meta */
    m1_app_manifest_t manifest;
    uint32_t       entry_point;   /* adjusted entry address in RAM */
    bool           loaded;
} elf_app_t;

/* API symbol entry (for resolution) */
typedef struct {
    uint32_t hash;
    void    *address;
} m1_api_sym_t;

/* API interface for symbol resolution */
typedef struct {
    m1_api_sym_t *table;
    uint16_t      count;
    uint16_t      api_version;
} m1_api_interface_t;

/* Loader results */
typedef enum {
    ELF_OK = 0,
    ELF_ERR_FILE,
    ELF_ERR_FORMAT,
    ELF_ERR_MACHINE,
    ELF_ERR_NO_MEMORY,
    ELF_ERR_RELOCATE,
    ELF_ERR_SYMBOL,
    ELF_ERR_MANIFEST,
    ELF_ERR_API_VERSION
} elf_load_status_t;

/* Public API */
elf_load_status_t elf_load_app(const char *path, elf_app_t *app, const m1_api_interface_t *api);
void elf_unload_app(elf_app_t *app);
uint32_t elf_gnu_hash(const char *name);

#endif /* M1_ELF_LOADER_H_ */
