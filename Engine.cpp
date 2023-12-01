#include "Engine.h"

SDL_Window* window = nullptr;
SDL_Renderer* renderer = nullptr;
World world;

bool initialize() {
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

void close() {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void handleEvents(SDL_Event& e, bool& quit) {
    while (SDL_PollEvent(&e) != 0) {
        if (e.type == SDL_QUIT) {
            quit = true;
        }
    }
}


void gameLoop() {
    SDL_Event e;
    bool quit = false;

    while (!quit) {
        handleEvents(e, quit);
        world.update();
        world.render(renderer);

        SDL_Delay(100);
    }
}

void startEngine() {
    if (!initialize()) {
        std::cerr << "Failed to initialize!\n";
        close();
        return;
    }

    world.addWorldObject(std::make_shared<Obstacle>(200, 100, 0, -1, OBSTACLE_WIDTH, OBSTACLE_HEIGHT, 255, 0, 0));
    world.addWorldObject(std::make_shared<Obstacle>(400, 100, 0, 0, OBSTACLE_WIDTH, OBSTACLE_HEIGHT, 255, 0, 0));
    
    world.addWorldObject(std::make_shared<Robot>(10, 10, 0, 0, 10, 10, 0, 255, 0));

    gameLoop();
    close();
}