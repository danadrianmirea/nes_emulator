#include <string.h>
#include "nes.h"
#include "memory.h"

bool nes_init(NES* nes) {
    if (!nes) return false;
    
    // Initialize CPU registers
    nes->pc = 0;
    nes->sp = 0xFD;  // Initial stack pointer value
    nes->a = 0;
    nes->x = 0;
    nes->y = 0;
    nes->p = 0x34;   // Initial status register value
    
    // Clear screen buffer
    memset(nes->screen_buffer, 0, sizeof(nes->screen_buffer));
    
    return true;
}

void nes_cleanup(NES* nes) {
    if (!nes) return;
    // Additional cleanup if needed
}

void nes_reset(NES* nes) {
    if (!nes) return;
    
    // Reset CPU registers
    nes->sp = 0xFD;
    nes->p = 0x34;
    
    // Read reset vector from memory
    if (nes->memory) {
        u8 low = memory_read(nes->memory, 0xFFFC);
        u8 high = memory_read(nes->memory, 0xFFFD);
        nes->pc = (high << 8) | low;
    } else {
        nes->pc = 0;
    }
    
    // Clear registers
    nes->a = 0;
    nes->x = 0;
    nes->y = 0;
}

void nes_run_frame(NES* nes) {
    if (!nes) return;
    
    // This is a placeholder for the actual CPU emulation
    // You'll need to implement the actual CPU instructions and PPU synchronization
    
    // For now, just fill the screen with a test pattern
    static u32 frame_count = 0;
    frame_count++;
    
    for (int y = 0; y < NES_SCREEN_HEIGHT; y++) {
        for (int x = 0; x < NES_SCREEN_WIDTH; x++) {
            u32 color = ((x + frame_count) ^ (y + frame_count)) & 0xFF;
            nes->screen_buffer[y * NES_SCREEN_WIDTH + x] = 
                (0xFF << 24) |    // Alpha
                (color << 16) |   // Red
                (color << 8) |    // Green
                color;            // Blue
        }
    }
} 