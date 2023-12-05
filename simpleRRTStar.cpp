#include "simpleRRTStar.h"


double planAndVisualizePath(const std::string& imagePath, int start_x, int start_y, int goal_x, int goal_y, const std::string& outputPath)
{
    // Load image
    Mat img = imread(imagePath);

    // Check if the image is loaded successfully
    if (img.empty()) {
        std::cerr << "Error: Unable to load the image." << std::endl;
        return -1.0;  // Return a negative value to indicate failure
    }

    // State space setup
    auto space = std::make_shared<ob::RealVectorStateSpace>();
    space->addDimension(0.0, img.cols);
    space->addDimension(0.0, img.rows);

    // SimpleSetup initialization
    auto ss = std::make_shared<og::SimpleSetup>(space);

    // Set start and goal states
    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = start_x;
    start[1] = start_y;
    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = goal_x;
    goal[1] = goal_y;
    ss->setStartAndGoalStates(start, goal);

    // Set state validity checker
    ss->setStateValidityChecker([&img](const ob::State* state) {
        const int w = std::min(static_cast<int>(state->as<ob::RealVectorStateSpace::StateType>()->values[0]), img.cols - 1);
        const int h = std::min(static_cast<int>(state->as<ob::RealVectorStateSpace::StateType>()->values[1]), img.rows - 1);
        return img.at<Vec3b>(h, w)[2] >= 200 && img.at<Vec3b>(h, w)[1] >= 200 && img.at<Vec3b>(h, w)[0] >= 200;
        });
    space->setup();
    ss->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

    // Set up optimization objective
    ob::OptimizationObjectivePtr objective = std::make_shared<ob::PathLengthOptimizationObjective>(ss->getSpaceInformation());
    ss->setOptimizationObjective(objective);

    // Set the planner (RRT*)
    auto planner = std::make_shared<og::RRTstar>(ss->getSpaceInformation());
    ss->setPlanner(planner);

    double totalDistance = 0.0;

    try {
        // Attempt to solve the problem
        if (ss->solve())
        {
            // Retrieve and interpolate the solution path
            og::PathGeometric& path = ss->getSolutionPath();
            path.interpolate();

            // Visualize the path on the image and calculate the distance
            for (std::size_t i = 0; i < path.getStateCount(); ++i)
            {
                const int w = std::min(img.cols - 1, static_cast<int>(path.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]));
                const int h = std::min(img.rows - 1, static_cast<int>(path.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]));
                img.at<Vec3b>(h, w)[2] = 255;
                img.at<Vec3b>(h, w)[1] = 0;
                img.at<Vec3b>(h, w)[0] = 0;

                // Calculate the distance between consecutive states
                if (i > 0) {
                    const auto& state1 = path.getState(i - 1);
                    const auto& state2 = path.getState(i);
                    totalDistance += ss->getSpaceInformation()->distance(state1, state2);
                }
            }

            // Save the result image
            imwrite(outputPath, img);

            return totalDistance;  // Return the distance covered by the path
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return -1.0;  // Return a negative value to indicate failure
}

void DrawRRTStarPath()
{
    // Example usage:
    for (int i = 1; i <= 9; i++) {

        std::string imagePath = "inputs/img_" + std::to_string(i) + ".png";
        int start_x = 20;
        int start_y = 20;

        int goal_x = 780;
        int goal_y = 560;
        std::string outputPath = "outputs/test.jpg";

        double distance = planAndVisualizePath(imagePath, start_x, start_y, goal_x, goal_y, outputPath);

        if (distance >= 0.0)
            std::cout << "Path planning successful. Result saved as " << outputPath << std::endl
            << "Total Distance Covered: " << distance << std::endl;
        else
            std::cerr << "Path planning failed." << std::endl;

    }
}
