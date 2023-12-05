#include "Robot.h"

double solverTime;


class RobotValidityChecker : public ompl::base::StateValidityChecker {
public:
    RobotValidityChecker(const ompl::base::SpaceInformationPtr& si, std::vector<std::shared_ptr<WorldObject>> obstacles)
        : ompl::base::StateValidityChecker(si), obstacles(obstacles) {
        for (auto& obstacle : obstacles) {
            if (obstacle->getObjectType() == OBSTACLE) {
                obstacleMap[obstacle.get()]["x"] = obstacle->getX();
                obstacleMap[obstacle.get()]["y"] = obstacle->getY();
                obstacleMap[obstacle.get()]["isColliding"] = false;
            }
            else if (obstacle->getObjectType() == ROBOT) {
                robot = obstacle;
            }
        }
    }

    bool isValid(const ompl::base::State* state) const override {
        // Check validity with respect to all obstacles
        const int stateX = std::min(static_cast<int>(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]), SCREEN_WIDTH);
        const int stateY = std::min(static_cast<int>(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]), SCREEN_HEIGHT);

        for (const auto& obstacle : obstacles) {
            if (obstacle->getObjectType() == OBSTACLE) {
                int prevOx = obstacleMap[obstacle.get()]["x"];
                int prevOy = obstacleMap[obstacle.get()]["y"];

                int oX = obstacle->getX();
                int oY = obstacle->getY();

                if (!obstacle->isMovable() || prevOx != oX || prevOy != oY) {

                    //std::cout << solverTime << std::endl;

                    // observed velocities
                    int dVx = oX - prevOx;
                    int dVy = oY - prevOy;

                    if (robot) {
                        bool collides = obstacle->collidesWith(robot, stateX, stateY);
                        obstacleMap[obstacle.get()]["isColliding"] = collides;

                        if (collides) return false;
                    }
                  
                }
                else {
                    if (obstacleMap[obstacle.get()]["isColliding"]) return false;
                }

            }   
        }
        return true; // State is valid
    }

private:
    std::shared_ptr<WorldObject> robot;
    std::vector<std::shared_ptr<WorldObject>> obstacles;
    mutable std::unordered_map<const WorldObject*, std::unordered_map<std::string, int>> obstacleMap;
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
                solverTime = simpleSetup->getLastPlanComputationTime();
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
    robotThread = std::thread(&Robot::solver, this);
}

void Robot::stopThread() {
    isRunning = false;

    if (robotThread.joinable()) {
        robotThread.join();
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
