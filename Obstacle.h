#pragma once
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "WorldObject.h"

class Obstacle : public WorldObject {
public:
    Obstacle(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b);

    void update(std::vector<std::shared_ptr<WorldObject>> allObjects) override;
};

#endif // OBSTACLE_H
