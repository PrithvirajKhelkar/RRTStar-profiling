#pragma once
#ifndef WORLD_OBJECT_H
#define WORLD_OBJECT_H

#include "globals.h"

class WorldObject {
public:
    WorldObject(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b);
    virtual ~WorldObject() = default;

    int getVx() const;
    int getVy() const;
    void setVx(int newVx);
    void setVy(int newVy);

    virtual void move() = 0;
    virtual void render(SDL_Renderer* renderer) const;

    bool checkBoundaryCollision() const;

protected:
    int x, y;
    int vx, vy;
    int width, height;
    SDL_Color color;
};

#endif // WORLD_OBJECT_H
