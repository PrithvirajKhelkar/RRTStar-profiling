#include "Engine.h"

Engine::Engine() {
    window = nullptr;
    renderer == nullptr;
}

bool Engine::initialize() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        return false;
    }

    window = SDL_CreateWindow("Modular Game", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

    if (window == nullptr) {
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (renderer == nullptr) {
        return false;
    }

    world.initialize();

    return true;
}

void Engine::close() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void Engine::handleEvents(SDL_Event& e, bool& quit) {
    while (SDL_PollEvent(&e) != 0) {
        if (e.type == SDL_QUIT) {
            quit = true;
        }
    }
}


void Engine::gameLoop() {
    SDL_Event e;
    bool quit = false;

    while (!quit) {
        handleEvents(e, quit);
        world.update();

        

        world.render(renderer);
        SDL_Delay(DELTA_TIME);
    }
}

void Engine::startEngine() {
    if (!initialize()) {
        std::cerr << "Failed to initialize!\n";
        close();
        return;
    }

    setupEngine();

    gameLoop();
    close();
}