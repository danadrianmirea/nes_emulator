#include <stdlib.h>
#include <string.h>
#include "memory.h"

bool memory_init(Memory* mem) {
    if (!mem) return false;
    
    // Clear RAM
    memset(mem->ram, 0, RAM_SIZE);
    memset(mem->ppu_reg, 0, PPU_REG_SIZE);
    memset(mem->apu_io, 0, APU_IO_SIZE);
    
    // Initialize pointers
    mem->sram = NULL;
    mem->prg_rom = NULL;
    mem->chr_rom = NULL;
    
    // Initialize cartridge info
    mem->has_trainer = false;
    mem->has_battery = false;
    mem->prg_rom_banks = 0;
    mem->chr_rom_banks = 0;
    mem->mapper_number = 0;
    
    // Initialize bank switching state
    mem->prg_bank = 0;
    mem->chr_bank = 0;
    
    return true;
}

void memory_cleanup(Memory* mem) {
    if (!mem) return;
    
    // Free allocated memory
    if (mem->sram) {
        free(mem->sram);
        mem->sram = NULL;
    }
    if (mem->prg_rom) {
        free(mem->prg_rom);
        mem->prg_rom = NULL;
    }
    if (mem->chr_rom) {
        free(mem->chr_rom);
        mem->chr_rom = NULL;
    }
}

void memory_reset(Memory* mem) {
    if (!mem) return;
    
    // Clear RAM but preserve ROM contents
    memset(mem->ram, 0, RAM_SIZE);
    memset(mem->ppu_reg, 0, PPU_REG_SIZE);
    memset(mem->apu_io, 0, APU_IO_SIZE);
    
    // Reset bank switching state
    mem->prg_bank = 0;
    mem->chr_bank = 0;
}

u8 memory_read(Memory* mem, u16 addr) {
    if (!mem) return 0;
    
    // Handle memory map regions
    if (addr < 0x2000) {
        // Internal RAM and its mirrors
        return mem->ram[addr & 0x07FF];
    }
    else if (addr < 0x4000) {
        // PPU registers and their mirrors
        return mem->ppu_reg[addr & 0x0007];
    }
    else if (addr < 0x4020) {
        // APU and I/O registers
        return mem->apu_io[addr - 0x4000];
    }
    else if (addr >= 0x6000 && addr < 0x8000) {
        // SRAM (if present)
        if (mem->sram && mem->has_battery) {
            return mem->sram[addr - 0x6000];
        }
        return 0;
    }
    else if (addr >= 0x8000) {
        // PRG-ROM
        if (mem->prg_rom) {
            // For now, handle only NROM (mapper 0)
            // Will need to be updated for other mappers
            if (mem->prg_rom_banks == 1) {
                // Mirror 16KB ROM
                return mem->prg_rom[(addr & 0x3FFF)];
            } else {
                // 32KB ROM
                return mem->prg_rom[addr - 0x8000];
            }
        }
    }
    
    return 0;
}

void memory_write(Memory* mem, u16 addr, u8 value) {
    if (!mem) return;
    
    // Handle memory map regions
    if (addr < 0x2000) {
        // Internal RAM and its mirrors
        mem->ram[addr & 0x07FF] = value;
    }
    else if (addr < 0x4000) {
        // PPU registers and their mirrors
        mem->ppu_reg[addr & 0x0007] = value;
    }
    else if (addr < 0x4020) {
        // APU and I/O registers
        mem->apu_io[addr - 0x4000] = value;
    }
    else if (addr >= 0x6000 && addr < 0x8000) {
        // SRAM (if present and battery-backed)
        if (mem->sram && mem->has_battery) {
            mem->sram[addr - 0x6000] = value;
        }
    }
    else if (addr >= 0x8000) {
        // PRG-ROM space - might be used for mapper registers
        // For now (mapper 0), this is read-only
        return;
    }
} 