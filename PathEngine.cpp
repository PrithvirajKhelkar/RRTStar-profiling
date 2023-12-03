#include "PathEngine.h"

void PathEngine::setupEngine() {
    world.addWorldObject(std::make_shared<Obstacle>(200, 100, 0, -1, OBSTACLE_WIDTH, OBSTACLE_HEIGHT, 255, 0, 0));
    world.addWorldObject(std::make_shared<Obstacle>(400, 100, 0, 1, OBSTACLE_WIDTH, OBSTACLE_HEIGHT, 255, 0, 0));
    world.addWorldObject(std::make_shared<PathRobot>(10, 10, 0, 0, 10, 10, 0, 255, 0));
}