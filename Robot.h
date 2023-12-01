#pragma once
#ifndef ROBOT_H
#define ROBOT_H

#include <thread>
#include <chrono>
#include <iostream>

#include "WorldObject.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

class Robot : public WorldObject {
public:
    Robot(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b);

    ~Robot();

    void update(std::vector<std::shared_ptr<WorldObject>> allObjects) override;

    void solver();

    void startThread();

    void stopThread();

    void render(SDL_Renderer* renderer) const override;

private:
    std::thread myThread;
    bool isRunning;
    ompl::geometric::PathGeometric* path;
};

#endif // ROBOT_H
