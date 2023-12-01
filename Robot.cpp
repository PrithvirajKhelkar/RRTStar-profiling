#include "Robot.h"

auto space = std::make_shared<ompl::base::RealVectorStateSpace>();
std::shared_ptr<ompl::geometric::SimpleSetup> simpleSetup;

Robot::Robot(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b)
    : WorldObject(x, y, vx, vy, width, height, r, g, b), path(nullptr) {

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

    isRunning = true;

    startThread();

}

Robot::~Robot() {
    stopThread();
}

void Robot::solver() {
    while (isRunning) {
        simpleSetup->solve();

        path = &simpleSetup->getSolutionPath();
        path->interpolate();
    }
}

void Robot::startThread() {
    myThread = std::thread(&Robot::solver, this);
}

void Robot::stopThread() {
    isRunning = false;

    if (myThread.joinable()) {
        myThread.join();
    }
}

void Robot::update(std::vector<std::shared_ptr<WorldObject>> allObjects) {
    // Implement robot-specific movement logic if needed
    // For now, the function is empty as an example
    


}

void Robot::render(SDL_Renderer* renderer) const {
    WorldObject::render(renderer);

    if (path) {
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
            SDL_Rect rect = { static_cast<int>(path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]),
                              static_cast<int>(path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]),
                              1, 1 };
            SDL_RenderFillRect(renderer, &rect);
        }
    }
}
