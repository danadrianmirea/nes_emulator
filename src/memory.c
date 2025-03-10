#include <stdlib.h>
#include <string.h>
#include "memory.h"

bool memory_init(Memory* mem) {
    if (!mem) return false;
    
    // Initialize all memory to 0
    memset(mem->ram, 0, RAM_SIZE);
    memset(mem->ppu_reg, 0, PPU_REG_SIZE);
    memset(mem->apu_io, 0, APU_IO_SIZE);
    
    mem->cart_rom = NULL;
    mem->has_trainer = false;
    mem->prg_rom_banks = 0;
    mem->chr_rom_banks = 0;
    mem->mapper_number = 0;
    
    return true;
}

void memory_cleanup(Memory* mem) {
    if (!mem) return;
    
    // Free cartridge ROM if loaded
    if (mem->cart_rom) {
        free(mem->cart_rom);
        mem->cart_rom = NULL;
    }
}

void memory_reset(Memory* mem) {
    if (!mem) return;
    
    // Reset RAM and registers to 0
    memset(mem->ram, 0, RAM_SIZE);
    memset(mem->ppu_reg, 0, PPU_REG_SIZE);
    memset(mem->apu_io, 0, APU_IO_SIZE);
}

u8 memory_read(Memory* mem, u16 addr) {
    if (!mem) return 0;
    
    // Memory map:
    // $0000-$07FF: 2KB internal RAM
    // $0800-$1FFF: Mirrors of $0000-$07FF
    // $2000-$2007: PPU registers
    // $2008-$3FFF: Mirrors of $2000-$2007
    // $4000-$4017: APU and I/O registers
    // $4018-$401F: APU and I/O functionality that is normally disabled
    // $4020-$FFFF: Cartridge space
    
    if (addr < 0x2000) {
        // Internal RAM and its mirrors
        return mem->ram[addr & 0x07FF];
    }
    else if (addr < 0x4000) {
        // PPU registers and their mirrors
        return mem->ppu_reg[(addr & 0x0007)];
    }
    else if (addr < 0x4018) {
        // APU and I/O registers
        return mem->apu_io[addr - 0x4000];
    }
    else if (addr >= 0x8000) {
        // Cartridge ROM
        if (mem->cart_rom) {
            return mem->cart_rom[addr - 0x8000];
        }
    }
    
    return 0;
}

void memory_write(Memory* mem, u16 addr, u8 value) {
    if (!mem) return;
    
    if (addr < 0x2000) {
        // Internal RAM and its mirrors
        mem->ram[addr & 0x07FF] = value;
    }
    else if (addr < 0x4000) {
        // PPU registers and their mirrors
        mem->ppu_reg[(addr & 0x0007)] = value;
    }
    else if (addr < 0x4018) {
        // APU and I/O registers
        mem->apu_io[addr - 0x4000] = value;
    }
    // Note: Writing to cartridge ROM space is handled by the mapper
} 