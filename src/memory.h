#ifndef MEMORY_H
#define MEMORY_H

#include "nes.h"

typedef struct Memory {
    u8 ram[RAM_SIZE];           // 2KB internal RAM
    u8 ppu_reg[PPU_REG_SIZE];   // PPU registers
    u8 apu_io[APU_IO_SIZE];     // APU and I/O registers
    u8* cart_rom;               // Pointer to loaded PRG-ROM
    u8* chr_rom;                // Pointer to loaded CHR-ROM (pattern tables)
    
    // Memory mapping flags and bank switching info
    bool has_trainer;
    u8 prg_rom_banks;
    u8 chr_rom_banks;
    u8 mapper_number;
} Memory;

// Memory management functions
bool memory_init(Memory* mem);
void memory_cleanup(Memory* mem);
void memory_reset(Memory* mem);

// Memory access functions
u8   memory_read(Memory* mem, u16 addr);
void memory_write(Memory* mem, u16 addr, u8 value);

#endif // MEMORY_H 