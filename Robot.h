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

    void initialize(std::vector<std::shared_ptr<WorldObject>> _allObjects);

    void update(std::vector<std::shared_ptr<WorldObject>> allObjects) override;

    void solver();

    void startThread();

    void stopThread();

    void render(SDL_Renderer* renderer) override;

    virtual void setStartState() = 0;
    virtual void setGoalState() = 0;

    virtual void onSolutionFound() = 0;

    virtual void updateRobot() = 0;
    virtual void renderRobot(SDL_Renderer* renderer) = 0;

private:
    std::thread myThread;
    bool isRunning;
    int maxVelocity;

protected:
    std::shared_ptr<ompl::geometric::SimpleSetup> simpleSetup;
};

#endif // ROBOT_H
