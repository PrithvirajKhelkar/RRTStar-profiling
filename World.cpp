#include "World.h"

void World::initialize() {}

void World::addWorldObject(std::shared_ptr<WorldObject> object) {
    sceneObjects.push_back(object);
}

void World::update() {
    for (auto& object : sceneObjects) {
        object->update(sceneObjects);
    }
}

void World::render(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 76, 86, 105, 255);
    SDL_RenderClear(renderer);

    for (const auto& object : sceneObjects) {
        object->render(renderer);
    }

    SDL_RenderPresent(renderer);
}
