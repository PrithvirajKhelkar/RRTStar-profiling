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

bool initialize();
void close();
void addWorldObject(std::shared_ptr<WorldObject> object);
void handleEvents(SDL_Event& e, bool& quit);
void update();
void render();
void gameLoop();
void startEngine();


#endif ENGINE_H
