#include "Robot.h"
#include <mutex>

std::mutex solutionMutex;
std::shared_ptr<ompl::geometric::SimpleSetup> simpleSetup;

// Define the state space for 2D position
class RobotStateSpace : public ompl::base::RealVectorStateSpace {
public:
    RobotStateSpace() : ompl::base::RealVectorStateSpace(2) {
        setName("RobotStateSpace");
    }
};

// Define the state validity checker with a vector of obstacles


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
    ompl::base::ScopedState<> start(simpleSetup->getStateSpace());
    start[0] = getX();
    start[1] = getY();
    ompl::base::ScopedState<> goal(simpleSetup->getStateSpace());
    goal[0] = SCREEN_WIDTH - 10;
    goal[1] = SCREEN_HEIGHT - 10;
    simpleSetup->setStartAndGoalStates(start, goal);


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
    if (!isRunning) {
        initialize(allObjects);
    }
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
