#include "Robot.h"

Robot::Robot(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b)
    : WorldObject(x, y, vx, vy, width, height, r, g, b) {}

void Robot::move() {
    // Implement robot-specific movement logic if needed
    // For now, the function is empty as an example
}
