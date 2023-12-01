#include "Obstacle.h"

Obstacle::Obstacle(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b)
    : WorldObject(x, y, vx, vy, width, height, r, g, b) {}

void Obstacle::update(std::vector<std::shared_ptr<WorldObject>> allObjects) {
    x += getVx();
    y += getVy();

    if (checkBoundaryCollision()) {
        // This is incorrect. But works for objects that only move in 
        // either horizontal or vertical axes.
        setVx(-getVx());
        setVy(-getVy());
    }
}