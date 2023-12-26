#include <cstdio>
#include <unistd.h>
#include "pros/rtos.hpp"
#include "SDL2/SDL.h"
#include "bot.h"
#include "SDL2/SDL2_gfxPrimitives.h"
#include "SDL2/SDL_image.h"
#include <list>

extern "C" void pros_init();
extern "C" void system_daemon_initialize(pros::task_fn_t init, pros::task_fn_t comp_init, pros::task_fn_t disable,
                                         pros::task_fn_t auton, pros::task_fn_t op);

struct {
    SDL_Renderer *renderer;
    SDL_Window *window;
} display;

bool init_sdl() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        printf("Couldn't initialize SDL: %s\n", SDL_GetError());
        return false;
    }

    int rendererFlags, windowFlags;

    rendererFlags = SDL_RENDERER_ACCELERATED;

    windowFlags = 0;

    display.window = SDL_CreateWindow("Graphical Simulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 720, 720,
                                      windowFlags);

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

using namespace sim;

constexpr Length scr_constant = 0.2_in;

std::list<V2Position> pos, pos_l, pos_r;

void print_pos(std::shared_ptr<Bot> &bot) {
    static auto field_img = IMG_LoadTexture(display.renderer, "field.png");
    static auto bot_img = IMG_LoadTexture(display.renderer, "bot.png");
    std::pair<V2Position, V2Position> wheel_pos = bot->getWheelPos();
    pos.emplace_back(bot->getPos());
    if (pos.size() > 20) pos.erase(pos.begin());
    pos_l.emplace_back(wheel_pos.first);
    if (pos_l.size() > 20) pos_l.erase(pos_l.begin());
    pos_r.emplace_back(wheel_pos.second);
    if (pos_r.size() > 20) pos_r.erase(pos_r.begin());
    //std::cout << pos.x << ", " << pos.y << std::endl;
    auto x = (int16_t) (pos.back().x.convert(scr_constant));
    auto y = (int16_t) (720 - pos.back().y.convert(scr_constant));
    SDL_Rect rect2 = {x - 30, y - 33, 60, 66};
    SDL_SetRenderDrawColor(display.renderer, 0, 0, 0, 0);
    SDL_RenderClear(display.renderer);
    SDL_RenderCopy(display.renderer, field_img, nullptr, nullptr);
    SDL_RenderCopyEx(display.renderer, bot_img, nullptr, &rect2, 90 - bot->getTheta().convert(deg), nullptr,
                     SDL_FLIP_VERTICAL);
    V2Position prev = pos.front();
    if (pos.size() > 1)
        for (V2Position a: pos) {
            lineRGBA(display.renderer, (int16_t) (a.x.convert(scr_constant)),
                     (int16_t) (720 - a.y.convert(scr_constant)), (int16_t) (prev.x.convert(scr_constant)),
                     (int16_t) (720 - prev.y.convert(scr_constant)), 255, 0, 0, 128);
            prev = a;
        }
    prev = pos_l.front();
    if (pos_l.size() > 1)
        for (V2Position a: pos_l) {
            lineRGBA(display.renderer, (int16_t) (a.x.convert(scr_constant)),
                     (int16_t) (720 - a.y.convert(scr_constant)), (int16_t) (prev.x.convert(scr_constant)),
                     (int16_t) (720 - prev.y.convert(scr_constant)), 0, 255, 0, 128);
            prev = a;
        }
    if (pos_l.size() > 1)
        prev = pos_r.front();
    for (V2Position a: pos_r) {
        lineRGBA(display.renderer, (int16_t) (a.x.convert(scr_constant)), (int16_t) (720 - a.y.convert(scr_constant)),
                 (int16_t) (prev.x.convert(scr_constant)), (int16_t) (720 - prev.y.convert(scr_constant)), 0, 0, 255,
                 128);
        prev = a;
    }
    SDL_RenderPresent(display.renderer);
}

extern "C" bool sim_SDL_init();
extern "C" void display_background_processing();

std::shared_ptr<sim::Bot> bot;

extern "C" void engine_update_func() {
    bot->update(false);
}

extern "C" void(*engine_update)();

__attribute__((constructor(101))) void init() {
    for (int8_t i = 0; i < V5_MAX_DEVICE_PORTS; i++) {
        emu_smart_ports[i].port = (int8_t) (i + 1);
        emu_smart_ports[i].exists = true;
        emu_smart_ports[i].type = kDeviceTypeMotorSensor;
        emu_smart_ports[i].motor.gearset = kMotorGearSet_06;
    }
    engine_update = engine_update_func;
    pros_init();
}

#ifdef WINVER

#include <windows.h>

extern "C" void initialize();
extern "C" void competition_initialize();
extern "C" void disabled();
extern "C" void autonomous();
extern "C" void opcontrol();

int CALLBACK WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
#else
    int main() {
#endif
    if (!init_sdl())
        return 1;
    if (!sim_SDL_init())
        return 1;
    system_daemon_initialize([](void *) {
        initialize();
    }, [](void *) {
        competition_initialize();
    }, [](void *) {
        disabled();
    }, [](void *) {
        autonomous();
    }, [](void *) {
        opcontrol();
    });
    static Bot bota({2}, {1}, {36_in, 36_in}, 0_deg, 2_in, 6_in, 600_rpm, 2, 12_lb, 0.25_kgm2);
    bot = std::shared_ptr<Bot>(&bota); // 300 rpm 4"
    int a = 0;
    while (true) {
        pros::Task::delay(5);

        a++;
        if (!(a % 4))

            display_background_processing();

        if (!(a % 7))
            print_pos(bot);
    }
}

