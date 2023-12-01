#pragma once
#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "globals.h"
#include "WorldObject.h"

class World {
public:
    void initialize();
    void addWorldObject(std::shared_ptr<WorldObject> object);
    void update();
    void render(SDL_Renderer* renderer);

private:
    std::vector<std::shared_ptr<WorldObject>> sceneObjects;
};

#endif // WORLD_H
