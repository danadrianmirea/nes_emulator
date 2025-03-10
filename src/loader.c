#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "loader.h"

// Private helper functions
static bool validate_header(const INESHeader* header) {
    // Check for iNES magic number
    if (memcmp(header->signature, "NES\x1A", 4) != 0) {
        printf("Error: Not a valid iNES ROM file\n");
        return false;
    }

    // Validate sizes
    if (header->prg_rom_banks == 0) {
        printf("Error: ROM contains no PRG-ROM banks\n");
        return false;
    }

    // Check for NES 2.0 format
    bool is_nes2 = ((header->flags7 & 0x0C) == 0x08);
    if (is_nes2) {
        printf("Warning: NES 2.0 format detected, some features may not be supported\n");
    }

    // Get mapper number
    u8 mapper_number = (header->flags7 & 0xF0) | (header->flags6 >> 4);
    
    // For now, we only support mappers 0-4
    if (mapper_number > 4) {
        printf("Error: Unsupported mapper type: %d\n", mapper_number);
        return false;
    }

    return true;
}

bool load_ines_rom(const char* filename, Memory* mem) {
    if (!filename || !mem) return false;
    
    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("Error: Could not open file: %s\n", filename);
        return false;
    }
    
    // Read header
    INESHeader header;
    if (fread(&header, 1, INES_HEADER_SIZE, file) != INES_HEADER_SIZE) {
        printf("Error: Could not read iNES header\n");
        fclose(file);
        return false;
    }
    
    // Validate header
    if (!validate_header(&header)) {
        fclose(file);
        return false;
    }
    
    // Store cartridge info
    mem->prg_rom_banks = header.prg_rom_banks;
    mem->chr_rom_banks = header.chr_rom_banks;
    mem->mapper_number = (header.flags7 & 0xF0) | (header.flags6 >> 4);
    mem->has_trainer = header.flags6 & 0x04;
    mem->has_battery = header.flags6 & 0x02;
    
    // Handle trainer if present
    if (mem->has_trainer) {
        printf("Warning: ROM contains trainer data\n");
        if (fseek(file, INES_TRAINER_SIZE, SEEK_CUR) != 0) {
            printf("Error: Could not skip trainer data\n");
            fclose(file);
            return false;
        }
    }
    
    // Allocate and load PRG-ROM
    size_t prg_size = header.prg_rom_banks * PRG_ROM_BANK_SIZE;
    mem->prg_rom = (u8*)malloc(prg_size);
    if (!mem->prg_rom) {
        printf("Error: Could not allocate memory for PRG-ROM\n");
        fclose(file);
        return false;
    }
    
    if (fread(mem->prg_rom, 1, prg_size, file) != prg_size) {
        printf("Error: Could not read PRG-ROM data\n");
        free(mem->prg_rom);
        mem->prg_rom = NULL;
        fclose(file);
        return false;
    }

    // Handle CHR-ROM if present
    if (header.chr_rom_banks > 0) {
        size_t chr_size = header.chr_rom_banks * CHR_ROM_BANK_SIZE;
        mem->chr_rom = (u8*)malloc(chr_size);
        if (!mem->chr_rom) {
            printf("Error: Could not allocate memory for CHR-ROM\n");
            free(mem->prg_rom);
            mem->prg_rom = NULL;
            fclose(file);
            return false;
        }

        if (fread(mem->chr_rom, 1, chr_size, file) != chr_size) {
            printf("Error: Could not read CHR-ROM data\n");
            free(mem->prg_rom);
            free(mem->chr_rom);
            mem->prg_rom = NULL;
            mem->chr_rom = NULL;
            fclose(file);
            return false;
        }
    } else {
        // No CHR-ROM, game uses CHR-RAM
        mem->chr_rom = NULL;
        printf("Info: ROM uses CHR-RAM\n");
    }

    // Allocate SRAM if battery-backed
    if (mem->has_battery) {
        mem->sram = (u8*)malloc(SRAM_SIZE);
        if (!mem->sram) {
            printf("Warning: Could not allocate SRAM, save functionality disabled\n");
            mem->has_battery = false;
        } else {
            memset(mem->sram, 0, SRAM_SIZE);
            printf("Info: Battery-backed SRAM enabled\n");
        }
    }
    
    fclose(file);
    
    // Print ROM info
    printf("ROM loaded successfully:\n");
    printf("PRG-ROM banks: %d (%dKB)\n", mem->prg_rom_banks, mem->prg_rom_banks * 16);
    printf("CHR-ROM banks: %d (%dKB)\n", mem->chr_rom_banks, mem->chr_rom_banks * 8);
    printf("Mapper: %d\n", mem->mapper_number);
    printf("Features: %s%s\n", 
           mem->has_battery ? "Battery " : "",
           mem->has_trainer ? "Trainer " : "");
    
    return true;
}

void unload_rom(Memory* mem) {
    if (!mem) return;
    
    if (mem->prg_rom) {
        free(mem->prg_rom);
        mem->prg_rom = NULL;
    }
    
    if (mem->chr_rom) {
        free(mem->chr_rom);
        mem->chr_rom = NULL;
    }

    if (mem->sram) {
        free(mem->sram);
        mem->sram = NULL;
    }
    
    mem->prg_rom_banks = 0;
    mem->chr_rom_banks = 0;
    mem->mapper_number = 0;
    mem->has_trainer = false;
    mem->has_battery = false;
} 