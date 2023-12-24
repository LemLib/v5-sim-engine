#include <cstdio>
#include <unistd.h>
#include "pros/rtos.hpp"
#include "SDL2/SDL.h"
#include "bot.h"
#include "SDL2/SDL2_gfxPrimitives.h"

extern "C" void pros_init();
extern "C" void system_daemon_initialize();

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

__attribute__((constructor(101))) void init() {
    for(int8_t i = 0; i < V5_MAX_DEVICE_PORTS; i++) {
        emu_smart_ports[i].port = (int8_t)(i+1);
    }
    for(int8_t i = 0; i < 2; i++) {
        emu_smart_ports[i].exists = true;
        emu_smart_ports[i].type = kDeviceTypeMotorSensor;
        emu_smart_ports[i].motor.gearset = kMotorGearSet_18;

    }
        emu_smart_ports[0].motor.voltage = 12000;
    emu_smart_ports[1].motor.voltage = -6000;
    pros_init();
}

using namespace sim;

constexpr Length scr_constant = 0.2_in;

bool update(Bot& bot) {
    static int a = 0;
    bot.update();
    static V2Position pos_prev;
    V2Position pos = bot.getPos();
    a++;

    SDL_SetRenderDrawColor(display.renderer, 0, 0, 0,0);
//    SDL_RenderClear(display.renderer);
    lineRGBA(display.renderer, (int16_t) (pos.x.convert(scr_constant)), (int16_t) (720-pos.y.convert(scr_constant)), (int16_t) (pos_prev.x.convert(scr_constant)), (int16_t) (720-pos_prev.y.convert(scr_constant)), 255, 0, 0, 128);
    SDL_RenderPresent(display.renderer);
    pos_prev = pos;
    a++;
    if(!(a % 5))
        std::cout << "X: " << pos.x << ", Y: " << pos.y << ", Theta: " << bot.getTheta().convert(deg) << "_deg" << std::endl;
    return true;
}

int main() {
    if (!init_sdl()) {
        return -1;
    }
    system_daemon_initialize();
    pros::Task::delay(2);
    sim::Bot bot({2}, {1});
    while(true) {
        pros::Task::delay(4);
        if(!update(bot)) exit(0);
    }
}

