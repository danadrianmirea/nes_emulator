#ifndef MEMORY_H
#define MEMORY_H

#include "nes.h"

// Memory map sizes and offsets
#define RAM_SIZE            0x0800   // 2KB internal RAM
#define RAM_MIRROR_SIZE     0x1800   // 6KB of RAM mirrors
#define PPU_REG_SIZE       0x0008   // 8 PPU registers
#define PPU_MIRROR_SIZE    0x1FF8   // PPU register mirrors
#define APU_IO_SIZE        0x0020   // APU and I/O registers
#define SRAM_SIZE          0x2000   // 8KB of potential SRAM (cartridge save memory)
#define PRG_ROM_SIZE       0x8000   // 32KB PRG-ROM space

// Memory map addresses
#define RAM_START          0x0000
#define RAM_MIRROR_START   0x0800
#define PPU_REG_START      0x2000
#define PPU_MIRROR_START   0x2008
#define APU_IO_START       0x4000
#define SRAM_START         0x6000
#define PRG_ROM_START      0x8000

typedef struct Memory {
    // Internal RAM (2KB)
    u8 ram[RAM_SIZE];           
    
    // PPU and APU registers
    u8 ppu_reg[PPU_REG_SIZE];   
    u8 apu_io[APU_IO_SIZE];     
    
    // Cartridge memory
    u8* sram;                   // Battery-backed save RAM (if present)
    u8* prg_rom;               // Program ROM from cartridge
    u8* chr_rom;               // Pattern tables (graphics ROM/RAM)
    
    // Memory mapping flags and bank switching info
    bool has_trainer;
    bool has_battery;          // Battery-backed RAM present
    u8 prg_rom_banks;         // Number of 16KB PRG-ROM banks
    u8 chr_rom_banks;         // Number of 8KB CHR-ROM banks
    u8 mapper_number;         // Mapper type
    
    // Current bank switching state
    u8 prg_bank;             // Current PRG bank number
    u8 chr_bank;             // Current CHR bank number
} Memory;

// Memory management functions
bool memory_init(Memory* mem);
void memory_cleanup(Memory* mem);
void memory_reset(Memory* mem);

// Memory access functions
u8   memory_read(Memory* mem, u16 addr);
void memory_write(Memory* mem, u16 addr, u8 value);

#endif // MEMORY_H 