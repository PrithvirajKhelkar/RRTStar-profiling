#include "armRRTStar.h"
#include <cstdlib>

# define M_PI           3.14159265358979323846 

double lower_bound_jointX = -M_PI;
double upper_bound_jointX = M_PI;

int arm_length = 10;

int size = 6;

void planAndVisualizeArm(const std::string& imagePath, std::vector<double> link_lengths, const std::vector<double>& start_config, const std::vector<double>& goal_config, const std::string& outputPath)
{
    Mat img = imread(imagePath);

    if (img.empty()) {
        std::cerr << "Error: Unable to load the image." << std::endl;
    }

    auto space = std::make_shared<ob::RealVectorStateSpace>();
    for (size_t i = 0; i < size; ++i) {
        space->addDimension(lower_bound_jointX, upper_bound_jointX);
    }

    auto ss = std::make_shared<og::SimpleSetup>(space);

    ob::ScopedState<> start(ss->getStateSpace());
    for (size_t i = 0; i < size; ++i) {
        start[i] = start_config[i];
    }

    ob::ScopedState<> goal(ss->getStateSpace());
    for (size_t i = 0; i < size; ++i) {
        goal[i] = goal_config[i];
    }

    ss->setStartAndGoalStates(start, goal);

    ss->setStateValidityChecker([&img](const ob::State* state) {
        // Implement your state validity checker for a 6-DOF robot
        // Check if the robot configuration is valid in the given state
        // You may need to perform collision checking or other validity checks

        double angle = 0;
        int x = 0;
        int y = img.rows / 2;

        for (int i = 0; i < size; i++) {
            angle += state->as<ob::RealVectorStateSpace::StateType>()->values[i];

            int newX = x + static_cast<int>(arm_length * cos(angle));
            int newY = y + static_cast<int>(arm_length * sin(angle));

            for (int dx = x; dx < newX; dx++) {
                for (int dy = y; dy < newY; dy++) {
                    if (dx > img.cols - 1 || dx < 0) return false;
                    if (dy > img.rows - 1 || dy < 0) return false;

                    return img.at<Vec3b>(dy, dx)[2] >= 200 && img.at<Vec3b>(dy, dx)[1] >= 200 && img.at<Vec3b>(dy, dx)[0] >= 200;
                }
            }
            x = newX;
            y = newY;

        }

        return false; // Placeholder, replace with your implementation
        });

    space->setup();
    //ss->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

    ob::OptimizationObjectivePtr objective = std::make_shared<ob::PathLengthOptimizationObjective>(ss->getSpaceInformation());
    ss->setOptimizationObjective(objective);

    auto planner = std::make_shared<og::RRTstar>(ss->getSpaceInformation());
    ss->setPlanner(planner);

    double totalDistance = 0.0;

    try {
        if (ss->solve()) {
            og::PathGeometric& path = ss->getSolutionPath();
            std::cout << "Solution coordinates" << std::endl;
            for (int i = 0; i < size; i++) {
                std::cout << path.getState(path.getStateCount() - 1)->as<ob::RealVectorStateSpace::StateType>()->values[i] << std::endl;
            }

            
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

}

void DrawArmRRTStarPath()
{
    std::string imagePath = "inputs/img_10.png";

    std::srand(static_cast<unsigned>(std::time(nullptr)));

    std::vector<double> start_config = {-M_PI/4, -M_PI/4, 0, 0, 0, 0};
    //start_config.reserve(size);

    std::vector<double> link_lengths;
    link_lengths.reserve(size);

    std::vector<double> goal_coords = {M_PI/3, -M_PI/3, M_PI/4, 0, 0, 0};
    //goal_coords.reserve(size);

    for (size_t i = 0; i < size; ++i) {
        // Generate a random angle and add it to the vector
        //double random_angle = lower_bound_jointX + (upper_bound_jointX - lower_bound_jointX) * (std::rand() / (RAND_MAX + 1.0));
        //goal_coords.push_back(random_angle);

        //start_config.push_back(0);
        link_lengths.push_back(arm_length);
    }

    
    std::string outputPath = "outputs/test.jpg";

    std::cout << "Goal coordinates" << std::endl;
    for (int i = 0; i < size; i++) {
        std::cout << goal_coords[i] << std::endl;
    }

    planAndVisualizeArm(imagePath, link_lengths, start_config, goal_coords, outputPath);

  
}
