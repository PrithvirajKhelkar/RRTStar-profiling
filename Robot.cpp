#include "Robot.h"
#include <mutex>

std::mutex solutionMutex;

class RobotValidityChecker : public ompl::base::StateValidityChecker {
public:
    RobotValidityChecker(const ompl::base::SpaceInformationPtr& si, std::vector<std::shared_ptr<WorldObject>> obstacles)
        : ompl::base::StateValidityChecker(si), obstacles(obstacles) {}

    bool isValid(const ompl::base::State* state) const override {
        // Check validity with respect to all obstacles
        const int dx = std::min(static_cast<int>(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]), SCREEN_WIDTH);
        const int dy = std::min(static_cast<int>(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]), SCREEN_HEIGHT);

        for (const auto& obstacle : obstacles) {
            if (obstacle->getObjectType() == OBSTACLE && obstacle->collidesWith(dx, dy)) {
                return false; // Collision with obstacle
            }
        }
        return true; // State is valid
    }

private:
    std::vector<std::shared_ptr<WorldObject>> obstacles;
};

Robot::Robot(int x, int y, int vx, int vy, int width, int height, Uint8 r, Uint8 g, Uint8 b)
    : WorldObject(x, y, vx, vy, width, height, r, g, b, ROBOT), isRunning(false) {}

Robot::~Robot() {
    stopThread();
}

void Robot::initialize(std::vector<std::shared_ptr<WorldObject>> _allObjects) {
    isRunning = true;

    auto space = std::make_shared<ompl::base::RealVectorStateSpace>();
    space->addDimension(0.0, SCREEN_WIDTH);
    space->addDimension(0.0, SCREEN_HEIGHT);

    simpleSetup = std::make_shared<ompl::geometric::SimpleSetup>(space);

    // Create the validity checker with obstacles
    auto validityChecker = std::make_shared<RobotValidityChecker>(simpleSetup->getSpaceInformation(), _allObjects);
    simpleSetup->setStateValidityChecker(validityChecker);

    // Set start and goal states
    setStartState();
    setGoalState();

    // Set up optimization objective
    ompl::base::OptimizationObjectivePtr objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(simpleSetup->getSpaceInformation());
    simpleSetup->setOptimizationObjective(objective);

    // Set the planner (RRT*)
    auto planner = std::make_shared<ompl::geometric::RRTstar>(simpleSetup->getSpaceInformation());
    simpleSetup->setPlanner(planner);

    startThread();
    
}

void Robot::solver() {
    while (isRunning) {
        try {
            ompl::base::PlannerStatus solved = simpleSetup->solve();
            if (solved) {
                onSolutionFound();
            }
            else {
                std::cout << "No solution found" << std::endl;
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
    if (!isRunning) {
        initialize(allObjects);
    }
    updateRobot();
}

void Robot::render(SDL_Renderer* renderer) {
    WorldObject::render(renderer);
    
    renderRobot(renderer);

    
}
