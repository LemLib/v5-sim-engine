#include <cstdio>
#include <unistd.h>
#include "pros/rtos.hpp"
#include "SDL2/SDL.h"

extern "C" void pros_init();
extern "C" void system_daemon_initialize();
__attribute__((constructor(101))) void init() {
    pros_init();
}

struct {
    SDL_Renderer* renderer;
    SDL_Window* window;
} display;

bool init_sdl() {
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        printf("Couldn't initialize SDL: %s\n", SDL_GetError());
        return false;
    }

    int rendererFlags, windowFlags;

    rendererFlags = SDL_RENDERER_ACCELERATED;

    windowFlags = 0;

    display.window = SDL_CreateWindow("Graphical Simulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 720,
                                      720, windowFlags);

    if (!display.window) {
        printf("Failed to open %d x %d window: %s\n", 720, 720, SDL_GetError());
        return false;
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

    display.renderer = SDL_CreateRenderer(display.window, -1, rendererFlags);

    if (!display.renderer) {
        printf("Failed to create renderer: %s\n", SDL_GetError());
        SDL_DestroyWindow(display.window);
        return false;
    }
    return true;
}

bool update() {
    SDL_RenderPresent(display.renderer);
    return true;
}

int main() {

    if (!init_sdl()) {
        return -1;
    }
    system_daemon_initialize();
    while(true) {
        pros::Task::delay(20);
        if(!update()) exit(0);
    }
}

