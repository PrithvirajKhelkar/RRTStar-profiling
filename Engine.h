#pragma once

#ifndef ENGINE_H
#define ENGINE_H

#include <iostream>
#include <vector>
#include <memory>

#include "globals.h"
#include "World.h"
#include "WorldObject.h"
#include "Obstacle.h"
#include "Robot.h"


class Engine {
public:

    Engine();

    bool initialize();
    void close();
    void handleEvents(SDL_Event& e, bool& quit);
    void gameLoop();
    void startEngine();

    virtual void setupEngine() = 0;

private:
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;

protected:
    World world;
};


#endif ENGINE_H
