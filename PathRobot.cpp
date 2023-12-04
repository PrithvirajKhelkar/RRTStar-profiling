#include "PathRobot.h"


PathRobot::PathRobot(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b)
    : Robot(x, y, vx, vy, width, height, r, g, b) {}

void PathRobot::setStartState() {
    ompl::base::ScopedState<> start(simpleSetup->getStateSpace());
    start[0] = getX();
    start[1] = getY();
    simpleSetup->setStartState(start);
};

void PathRobot::setGoalState() {
    ompl::base::ScopedState<> goal(simpleSetup->getStateSpace());
    goal[0] = SCREEN_WIDTH - 10;
    goal[1] = SCREEN_HEIGHT - 10;
    simpleSetup->setGoalState(goal);
};

void PathRobot::onSolutionFound() {
    ompl::geometric::PathGeometric& path = simpleSetup->getSolutionPath();

    path.interpolate();

    // Lock the mutex before modifying the shared resource (solution vector)
    std::lock_guard<std::mutex> lock(solutionMutex);

    solution.clear();

    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        if (!simpleSetup->getStateValidityChecker()->isValid(path.getState(i))) {
            simpleSetup->clear();
            break;
        }

        solution.push_back({ static_cast<int>(path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]),
                            static_cast<int>(path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]) });
    }
};

void PathRobot::updateRobot() {

};

void PathRobot::renderRobot(SDL_Renderer* renderer) {
    std::vector<std::vector<int>> temp(solution);

    // Lock the mutex before accessing the shared resource (solution vector)
    {
        std::lock_guard<std::mutex> lock(solutionMutex);
        temp = solution;
    }

    if (temp.size()) {
        for (std::vector<int> coord : temp) {
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 120);
            SDL_Rect rect = { coord[0],
                              coord[1],
                              4, 4 };
            SDL_RenderFillRect(renderer, &rect);
        }
    }
};