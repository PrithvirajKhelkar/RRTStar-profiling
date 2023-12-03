#pragma once

#ifndef PATH_ROBOT_H
#define PATH_ROBOT_H

#include "Robot.h"
#include <mutex>


class PathRobot : public Robot {
public:
    PathRobot(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b);

    void setStartState();
    void setGoalState();

    void onSolutionFound();

    void updateRobot();
    void renderRobot(SDL_Renderer* renderer);

private:
    std::vector<std::vector<int>> solution;
    std::mutex solutionMutex;
};


#endif PATH_ROBOT_H
