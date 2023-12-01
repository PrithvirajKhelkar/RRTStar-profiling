#pragma once
#ifndef ROBOT_H
#define ROBOT_H

#include "WorldObject.h"

class Robot : public WorldObject {
public:
    Robot(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b);

    void move() override;
};

#endif // ROBOT_H
