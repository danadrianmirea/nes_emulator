#include <string.h>
#include "nes.h"

bool nes_init(NES* nes) {
    if (!nes) return false;
    
    // Initialize pointers to NULL
    nes->memory = NULL;
    nes->window = NULL;
    nes->renderer = NULL;
    nes->texture = NULL;
    
    // Clear screen buffer
    memset(nes->screen_buffer, 0, sizeof(nes->screen_buffer));
    nes->running = false;
    
    return true;
}

void nes_cleanup(NES* nes) {
    if (!nes) return;
    // Additional cleanup if needed
}

void nes_reset(NES* nes) {
    if (!nes) return;
    
    // Clear screen buffer
    memset(nes->screen_buffer, 0, sizeof(nes->screen_buffer));
}

void nes_run_frame(NES* nes) {
    if (!nes) return;
    
    // This is now handled in main.c with CPU stepping
} 