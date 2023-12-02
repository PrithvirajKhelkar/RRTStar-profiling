#include "Robot.h"
#include <mutex>

std::mutex solutionMutex;

Robot::Robot(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b)
    : WorldObject(x, y, vx, vy, width, height, r, g, b), isRunning(false) {

    isRunning = true;

    startThread();

}

Robot::~Robot() {
    stopThread();
}

void Robot::initialize(std::vector<std::shared_ptr<WorldObject>> _allObjects) {
    isRunning = true;

    std::cout << "came here" << std::endl;
}

void Robot::solver() {
    while (isRunning) {
        try {
            auto space = std::make_shared<ompl::base::RealVectorStateSpace>();
            std::shared_ptr<ompl::geometric::SimpleSetup> simpleSetup;

            space->addDimension(0.0, SCREEN_WIDTH);
            space->addDimension(0.0, SCREEN_HEIGHT);

            simpleSetup = std::make_shared<ompl::geometric::SimpleSetup>(space);

            // Set start and goal states
            ompl::base::ScopedState<> start(simpleSetup->getStateSpace());
            start[0] = getX();
            start[1] = getY();
            ompl::base::ScopedState<> goal(simpleSetup->getStateSpace());
            goal[0] = SCREEN_WIDTH - 10;
            goal[1] = SCREEN_HEIGHT - 10;
            simpleSetup->setStartAndGoalStates(start, goal);


            // Set state validity checker
            /* simpleSetup->setStateValidityChecker([](const ompl::base::State* state) {
                return true;
                });
            */
            simpleSetup->setStateValidityChecker(getStateValidityCheckerFunction(allObjects));

            space->setup();
            simpleSetup->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

            // Set up optimization objective
            ompl::base::OptimizationObjectivePtr objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(simpleSetup->getSpaceInformation());
            simpleSetup->setOptimizationObjective(objective);

            // Set the planner (RRT*)
            auto planner = std::make_shared<ompl::geometric::RRTstar>(simpleSetup->getSpaceInformation());
            simpleSetup->setPlanner(planner);

            simpleSetup->solve();

            ompl::geometric::PathGeometric& path = simpleSetup->getSolutionPath();
            path.interpolate();

            // Lock the mutex before modifying the shared resource (solution vector)
            std::lock_guard<std::mutex> lock(solutionMutex);

            solution.clear();

            for (std::size_t i = 0; i < path.getStateCount(); ++i) {
                solution.push_back({static_cast<int>(path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]),
                                    static_cast<int>(path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1])});
            }

        }
        catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
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
    std::cout << "came here first" << std::endl;
    if (!isRunning) {
        std::cout << "came here second" << std::endl;
        initialize(allObjects);
    }
    
    setAllObjects(allObjects);


}

void Robot::render(SDL_Renderer* renderer) const {
    WorldObject::render(renderer);
    
    std::vector<std::vector<int>> temp(solution);

    // Lock the mutex before accessing the shared resource (solution vector)
    {
        std::lock_guard<std::mutex> lock(solutionMutex);
        temp = solution;
    }

    if (temp.size()) {
        for (std::vector<int> coord : temp) {
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255);
            SDL_Rect rect = { coord[0],
                              coord[1],
                              1, 1 };
            SDL_RenderFillRect(renderer, &rect);
        }
    }
    
}

bool Robot::RobotCollision(std::vector<std::shared_ptr<WorldObject>> allObjects, int dx = 0, int dy = 0) {
    int r_x = x + dx;
    int r_y =  y + dy;

    for (const auto& obj : allObjects) {
        if (this != obj.get()) {
            if (r_x < obj->getX() + obj->getWidth() &&
                r_x + width > obj->getX() &&
                r_y < obj->getY() + obj->getHeight() &&
                r_y + height > obj->getY()) {
                return false;
            }

        }
    }

    return true;
}

std::function<bool(const ompl::base::State*)> Robot::getStateValidityCheckerFunction(std::vector<std::shared_ptr<WorldObject>> allObjects) {
    return [this, allObjects](const ompl::base::State* state) -> bool {
        const int dx = std::min(static_cast<int>(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]), SCREEN_WIDTH);
        const int dy = std::min(static_cast<int>(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]), SCREEN_HEIGHT);

        return RobotCollision(allObjects, dx, dy);
        };
}

void Robot::setAllObjects(std::vector<std::shared_ptr<WorldObject>> _allObjects) {
    allObjects.assign(_allObjects.begin(), _allObjects.end());
}