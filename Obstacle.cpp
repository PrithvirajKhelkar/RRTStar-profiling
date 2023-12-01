#include "Obstacle.h"

Obstacle::Obstacle(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b)
    : WorldObject(x, y, vx, vy, width, height, r, g, b) {}

void Obstacle::move() {
    x += getVx();
    y += getVy();

    if (checkBoundaryCollision()) {
        setVx(-getVx());
        setVy(-getVy());
    }
}