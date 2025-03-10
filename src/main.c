#include <stdio.h>
#include <SDL2/SDL.h>
#include "nes.h"
#include "memory.h"
#include "loader.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("Usage: %s <rom_file>\n", argv[0]);
        return 1;
    }

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL initialization failed: %s\n", SDL_GetError());
        return 1;
    }

    // Create NES instance
    NES nes = {0};
    Memory memory = {0};
    nes.memory = &memory;

    // Initialize memory
    if (!memory_init(&memory)) {
        printf("Memory initialization failed\n");
        SDL_Quit();
        return 1;
    }

    // Create window
    nes.window = SDL_CreateWindow(
        "NES Emulator",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        NES_SCREEN_WIDTH * NES_SCALE,
        NES_SCREEN_HEIGHT * NES_SCALE,
        SDL_WINDOW_SHOWN
    );

    if (!nes.window) {
        printf("Window creation failed: %s\n", SDL_GetError());
        memory_cleanup(&memory);
        SDL_Quit();
        return 1;
    }

    // Create renderer
    nes.renderer = SDL_CreateRenderer(nes.window, -1, SDL_RENDERER_ACCELERATED);
    if (!nes.renderer) {
        printf("Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(nes.window);
        memory_cleanup(&memory);
        SDL_Quit();
        return 1;
    }

    // Create texture for the screen buffer
    nes.texture = SDL_CreateTexture(
        nes.renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        NES_SCREEN_WIDTH,
        NES_SCREEN_HEIGHT
    );

    if (!nes.texture) {
        printf("Texture creation failed: %s\n", SDL_GetError());
        SDL_DestroyRenderer(nes.renderer);
        SDL_DestroyWindow(nes.window);
        memory_cleanup(&memory);
        SDL_Quit();
        return 1;
    }

    // Load ROM
    if (!load_ines_rom(argv[1], &memory)) {
        printf("Failed to load ROM: %s\n", argv[1]);
        SDL_DestroyTexture(nes.texture);
        SDL_DestroyRenderer(nes.renderer);
        SDL_DestroyWindow(nes.window);
        memory_cleanup(&memory);
        SDL_Quit();
        return 1;
    }

    // Main emulation loop
    nes.running = true;
    SDL_Event event;

    while (nes.running) {
        // Handle SDL events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                nes.running = false;
            }
        }

        // Run one frame of emulation
        nes_run_frame(&nes);

        // Update screen
        SDL_UpdateTexture(nes.texture, NULL, nes.screen_buffer, NES_SCREEN_WIDTH * sizeof(u32));
        SDL_RenderClear(nes.renderer);
        SDL_RenderCopy(nes.renderer, nes.texture, NULL, NULL);
        SDL_RenderPresent(nes.renderer);
    }

    // Cleanup
    SDL_DestroyTexture(nes.texture);
    SDL_DestroyRenderer(nes.renderer);
    SDL_DestroyWindow(nes.window);
    memory_cleanup(&memory);
    SDL_Quit();

    return 0;
} 