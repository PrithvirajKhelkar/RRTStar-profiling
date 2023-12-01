#include "World.h"

auto space = std::make_shared<ompl::base::RealVectorStateSpace>();
std::shared_ptr<ompl::geometric::SimpleSetup> simpleSetup;

void World::initialize() {
    space->addDimension(0.0, SCREEN_WIDTH);
    space->addDimension(0.0, SCREEN_HEIGHT);

    simpleSetup = std::make_shared<ompl::geometric::SimpleSetup>(space);

    // Set start and goal states
    ompl::base::ScopedState<> start(simpleSetup->getStateSpace());
    start[0] = 10;
    start[1] = 10;
    ompl::base::ScopedState<> goal(simpleSetup->getStateSpace());
    goal[0] = SCREEN_WIDTH - 10;
    goal[1] = SCREEN_HEIGHT - 10;
    simpleSetup->setStartAndGoalStates(start, goal);


    // Set state validity checker
    simpleSetup->setStateValidityChecker([](const ompl::base::State* state) {
        return true;
        });
    space->setup();
    simpleSetup->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

    // Set up optimization objective
    ompl::base::OptimizationObjectivePtr objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(simpleSetup->getSpaceInformation());
    simpleSetup->setOptimizationObjective(objective);

    // Set the planner (RRT*)
    auto planner = std::make_shared<ompl::geometric::RRTstar>(simpleSetup->getSpaceInformation());
    simpleSetup->setPlanner(planner);

}

void World::addWorldObject(std::shared_ptr<WorldObject> object) {
    sceneObjects.push_back(object);
}

void World::update() {
    for (auto& object : sceneObjects) {
        object->move();
    }
    //simpleSetup->solve();
}

void World::render(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    for (const auto& object : sceneObjects) {
        object->render(renderer);
    }

    SDL_RenderPresent(renderer);
}
