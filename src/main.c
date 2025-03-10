#include <stdio.h>
#include <SDL2/SDL.h>
#include "nes.h"
#include "memory.h"
#include "loader.h"
#include "cpu.h"

// Target ~60 FPS
#define TARGET_FPS 60
#define FRAME_TIME (1000 / TARGET_FPS)

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
    CPU cpu = {0};
    
    nes.memory = &memory;
    
    // Initialize memory and CPU
    if (!memory_init(&memory)) {
        printf("Memory initialization failed\n");
        SDL_Quit();
        return 1;
    }
    
    cpu_init(&cpu, &memory);

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

    // Reset CPU after ROM is loaded
    cpu_reset(&cpu);

    // Main emulation loop
    nes.running = true;
    SDL_Event event;
    Uint32 frame_start, frame_time;
    
    // CPU cycles per frame (~29780 cycles at 60 FPS)
    const int cycles_per_frame = (1789773 / 60);  // NES CPU clock rate / target FPS
    int cycles_remaining = 0;

    while (nes.running) {
        frame_start = SDL_GetTicks();

        // Handle SDL events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                nes.running = false;
            }
            // TODO: Handle controller input events
        }

        // Execute CPU cycles for this frame
        cycles_remaining = cycles_per_frame;
        while (cycles_remaining > 0) {
            cycles_remaining -= cpu_step(&cpu);
            
            // Check for interrupts
            // TODO: Implement proper PPU timing and NMI generation
        }

        // Update screen
        SDL_UpdateTexture(nes.texture, NULL, nes.screen_buffer, NES_SCREEN_WIDTH * sizeof(u32));
        SDL_RenderClear(nes.renderer);
        SDL_RenderCopy(nes.renderer, nes.texture, NULL, NULL);
        SDL_RenderPresent(nes.renderer);

        // Frame timing
        frame_time = SDL_GetTicks() - frame_start;
        if (frame_time < FRAME_TIME) {
            SDL_Delay(FRAME_TIME - frame_time);
        }
    }

    // Cleanup
    SDL_DestroyTexture(nes.texture);
    SDL_DestroyRenderer(nes.renderer);
    SDL_DestroyWindow(nes.window);
    memory_cleanup(&memory);
    SDL_Quit();

    return 0;
} 