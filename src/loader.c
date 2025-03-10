#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "loader.h"

bool load_ines_rom(const char* filename, Memory* mem) {
    if (!filename || !mem) return false;
    
    FILE* file = fopen(filename, "rb");
    if (!file) return false;
    
    // Read header
    INESHeader header;
    if (fread(&header, 1, INES_HEADER_SIZE, file) != INES_HEADER_SIZE) {
        fclose(file);
        return false;
    }
    
    // Verify iNES format
    if (memcmp(header.signature, "NES\x1A", 4) != 0) {
        fclose(file);
        return false;
    }
    
    // Store cartridge info
    mem->prg_rom_banks = header.prg_rom_banks;
    mem->chr_rom_banks = header.chr_rom_banks;
    mem->mapper_number = (header.flags7 & 0xF0) | (header.flags6 >> 4);
    mem->has_trainer = header.flags6 & 0x04;
    
    // Skip trainer if present
    if (mem->has_trainer) {
        fseek(file, INES_TRAINER_SIZE, SEEK_CUR);
    }
    
    // Allocate and load PRG-ROM
    size_t prg_size = header.prg_rom_banks * PRG_ROM_BANK_SIZE;
    mem->cart_rom = (u8*)malloc(prg_size);
    if (!mem->cart_rom) {
        fclose(file);
        return false;
    }
    
    if (fread(mem->cart_rom, 1, prg_size, file) != prg_size) {
        free(mem->cart_rom);
        mem->cart_rom = NULL;
        fclose(file);
        return false;
    }
    
    // For now, we'll skip CHR-ROM loading as it's handled by the PPU
    // You'll want to add this when implementing the PPU
    
    fclose(file);
    return true;
}

void unload_rom(Memory* mem) {
    if (!mem) return;
    
    if (mem->cart_rom) {
        free(mem->cart_rom);
        mem->cart_rom = NULL;
    }
    
    mem->prg_rom_banks = 0;
    mem->chr_rom_banks = 0;
    mem->mapper_number = 0;
    mem->has_trainer = false;
} 