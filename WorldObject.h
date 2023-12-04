#pragma once
#ifndef WORLD_OBJECT_H
#define WORLD_OBJECT_H

#include <vector>
#include <memory>

#include "globals.h"

enum WorldObjectType {
    ROBOT,
    OBSTACLE
};

class WorldObject {
public:
    WorldObject(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b, WorldObjectType objectType);
    virtual ~WorldObject() = default;



    int getX() const;
    int getY() const;
    int getVx() const;
    int getVy() const;
    void setVx(int newVx);
    void setVy(int newVy);
    int getWidth() const;
    int getHeight() const;
    WorldObjectType getObjectType() const;

    virtual void update(std::vector<std::shared_ptr<WorldObject>> allObjects) = 0;
    virtual void render(SDL_Renderer* renderer);

    bool checkBoundaryCollision() const;

    bool collidesWith(std::shared_ptr<WorldObject> otherObject, int xPos = 0, int yPos = 0, int dx = 0, int dy = 0);

    bool isMovable();

protected:
    WorldObjectType objectType;
    int x, y;
    int vx, vy;
    int width, height;
    SDL_Color color;
    bool movable;
};

#endif // WORLD_OBJECT_H
