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

    bool RobotCollision(std::vector<std::shared_ptr<WorldObject>> allObjects, int dx, int dy);

    std::function<bool(const ompl::base::State*)> getStateValidityCheckerFunction(std::vector<std::shared_ptr<WorldObject>> allObjects);

    void setAllObjects(std::vector<std::shared_ptr<WorldObject>> _allObjects);

private:
    std::thread myThread;
    bool isRunning;
    std::vector<std::vector<int>> solution;
    std::vector<std::shared_ptr<WorldObject>> allObjects;
};

#endif // ROBOT_H
