#ifndef NES_H
#define NES_H

#include <stdint.h>
#include <stdbool.h>
#include <SDL2/SDL.h>

// NES has a 256x240 resolution
#define NES_SCREEN_WIDTH    256
#define NES_SCREEN_HEIGHT   240
#define NES_SCALE          3

// Common types
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t   i8;
typedef int16_t  i16;
typedef int32_t  i32;

// Main NES structure
typedef struct {
    // Memory components
    struct Memory* memory;
    
    // SDL components
    SDL_Window*   window;
    SDL_Renderer* renderer;
    SDL_Texture*  texture;
    u32          screen_buffer[NES_SCREEN_WIDTH * NES_SCREEN_HEIGHT];
    
    bool running;
} NES;

// Function prototypes
bool nes_init(NES* nes);
void nes_cleanup(NES* nes);
void nes_reset(NES* nes);
void nes_run_frame(NES* nes);

#endif // NES_H 