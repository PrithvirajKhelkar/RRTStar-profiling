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
void handleEvents(SDL_Event& e, bool& quit);
void gameLoop();
void startEngine();


#endif ENGINE_H
