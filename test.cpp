#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <iostream>
#include <vector>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Define the state space for 2D position
class DroneStateSpace : public ob::RealVectorStateSpace {
public:
    DroneStateSpace() : ob::RealVectorStateSpace(2) {
        setName("DroneStateSpace");
        setBounds(0.0, 10.0); // Set bounds for x and y coordinates
    }
};

// Define a class for obstacles
class Obstacle {
public:
    Obstacle(double centerX, double centerY, double radius) : centerX(centerX), centerY(centerY), radius(radius) {}

    // Function to check if a given state collides with the obstacle
    bool collidesWith(const ob::State* state) const {
        double x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        double y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        return (std::hypot(x - centerX, y - centerY) <= radius);
    }

    // Function to update the obstacle's position (for moving obstacles)
    void updatePosition(double time) {
        // Implement your update logic here
        // For simplicity, assuming a stationary obstacle
    }

private:
    double centerX;
    double centerY;
    double radius;
};

// Define the state validity checker with a vector of obstacles
class ValidityChecker : public ob::StateValidityChecker {
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, const std::vector<Obstacle>& obstacles)
        : ob::StateValidityChecker(si), obstacles(obstacles) {}

    bool isValid(const ob::State* state) const override {
        // Check validity with respect to all obstacles
        for (const auto& obstacle : obstacles) {
            if (obstacle.collidesWith(state)) {
                return false; // Collision with obstacle
            }
        }
        return true; // State is valid
    }

private:
    const std::vector<Obstacle>& obstacles;
};

void  runASimpleTest() {
    // Create the state space
    auto space(std::make_shared<DroneStateSpace>());

    // Set the bounds for the state space
    space->setBounds(0.0, 10.0);

    // Create a simple setup object
    og::SimpleSetup ss(space);

    // Create a vector of obstacles
    std::vector<Obstacle> obstacles;
    obstacles.emplace_back(5.0, 5.0, 1.5); // Example obstacle

    // Create the validity checker with obstacles
    auto validityChecker = std::make_shared<ValidityChecker>(ss.getSpaceInformation(), obstacles);
    ss.setStateValidityChecker(validityChecker);

    // Set the start and goal states
    ob::ScopedState<> start(space);
    start[0] = 1.0;
    start[1] = 1.0;

    ob::ScopedState<> goal(space);
    goal[0] = 9.0;
    goal[1] = 9.0;

    ss.setStartAndGoalStates(start, goal);

    // Choose a planner (e.g., RRT)
    auto planner(std::make_shared<og::RRT>(ss.getSpaceInformation()));

    // Set the planner for the setup
    ss.setPlanner(planner);

    // Planning loop
    while (true) {
        // Attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = ss.solve(1.0);

        if (solved) {
            // Print the path
            ss.simplifySolution();
            std::cout << "Found solution:" << std::endl;
            ss.getSolutionPath().print(std::cout);
        }
        else {
            std::cout << "No solution found" << std::endl;
        }

        // Update obstacle positions for the next iteration
        for (auto& obstacle : obstacles) {
            obstacle.updatePosition(1.0); // Assuming a fixed time step for updating positions
        }

        // Sleep for a short duration before the next iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

}
