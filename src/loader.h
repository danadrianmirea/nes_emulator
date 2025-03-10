#ifndef LOADER_H
#define LOADER_H

#include "nes.h"
#include "memory.h"

// iNES header structure
#define INES_HEADER_SIZE 16
#define INES_TRAINER_SIZE 512
#define PRG_ROM_BANK_SIZE 16384  // 16KB
#define CHR_ROM_BANK_SIZE 8192   // 8KB

typedef struct {
    char     signature[4];    // Should be "NES\x1A"
    u8       prg_rom_banks;  // Number of 16KB PRG-ROM banks
    u8       chr_rom_banks;  // Number of 8KB CHR-ROM banks
    u8       flags6;         // Mapper, mirroring, battery, trainer flags
    u8       flags7;         // VS/Playchoice, NES 2.0 format
    u8       flags8;         // PRG-RAM size
    u8       flags9;         // TV system
    u8       flags10;        // TV system, PRG-RAM presence
    u8       padding[5];     // Unused padding
} INESHeader;

// Function prototypes
bool load_ines_rom(const char* filename, Memory* mem);
void unload_rom(Memory* mem);

#endif // LOADER_H 