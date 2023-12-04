#include "WorldObject.h"

WorldObject::WorldObject(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b, WorldObjectType objectType)
    : x(x), y(y), vx(vx), vy(vy), width(width), height(height), color({ r, g, b }), objectType(objectType), movable(vx != 0 && vy != 0) {}

int WorldObject::getX() const {
    return x;
}

int WorldObject::getY() const {
    return y;
}

int WorldObject::getVx() const {
    return vx;
}

int WorldObject::getVy() const {
    return vy;
}

void WorldObject::setVx(int newVx) {
    vx = newVx;
}

void WorldObject::setVy(int newVy) {
    vy = newVy;
}

int WorldObject::getWidth() const {
    return width;
};
int WorldObject::getHeight() const {
    return height;
};

void WorldObject::render(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
    SDL_Rect rect = { x, y, width, height };
    SDL_RenderFillRect(renderer, &rect);
}

bool WorldObject::checkBoundaryCollision() const {
    return x < 0 || x + width > SCREEN_WIDTH || y < 0 || y + height > SCREEN_HEIGHT;
}

WorldObjectType WorldObject::getObjectType() const {
    return objectType;
}

bool WorldObject::collidesWith(std::shared_ptr<WorldObject> otherObject, int xPos, int yPos, int dx, int dy) {
    return xPos < x + dx + width && xPos > x + dx && yPos < y + dy + height && yPos > y + dy;
}

bool WorldObject::isMovable() {
    return movable;
}