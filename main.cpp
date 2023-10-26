#include <opencv2/opencv.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace cv;

Mat img;

bool isStateValid(const ob::State* state)
{
    const int w = std::min(static_cast<int>(state->as<ob::RealVectorStateSpace::StateType>()->values[0]), img.cols - 1);
    const int h = std::min(static_cast<int>(state->as<ob::RealVectorStateSpace::StateType>()->values[1]), img.rows - 1);
    return img.at<Vec3b>(h, w)[2] >= 200 && img.at<Vec3b>(h, w)[1] >= 200 && img.at<Vec3b>(h, w)[0] >= 200;
}

int main()
{
    img = imread("map.png");

    if (img.empty()) {
        std::cerr << "Error: Unable to load the image." << std::endl;
        return 1;
    }

    auto space = std::make_shared<ob::RealVectorStateSpace>();
    space->addDimension(0.0, img.cols);
    space->addDimension(0.0, img.rows);

    auto ss = std::make_shared<og::SimpleSetup>(space);

    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = 20;
    start[1] = 20;
    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = 780;
    goal[1] = 560;
    ss->setStartAndGoalStates(start, goal);

    ss->setStateValidityChecker(isStateValid);
    space->setup();
    ss->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

    auto planner = std::make_shared<og::RRTstar>(ss->getSpaceInformation());

    // Create an optimization objective to minimize path length
    ob::OptimizationObjectivePtr objective = std::make_shared<ob::PathLengthOptimizationObjective>(ss->getSpaceInformation());
    ss->setOptimizationObjective(objective);

    ss->setPlanner(planner);

    try {
        if (ss->solve())
        {
            og::PathGeometric& path = ss->getSolutionPath();
            path.interpolate();

            for (std::size_t i = 0; i < path.getStateCount(); ++i)
            {
                const int w = std::min(img.cols - 1, static_cast<int>(path.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]));
                const int h = std::min(img.rows - 1, static_cast<int>(path.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]));
                img.at<Vec3b>(h, w)[2] = 255;
                img.at<Vec3b>(h, w)[1] = 0;
                img.at<Vec3b>(h, w)[0] = 0;
            }

            imwrite("path_result.jpg", img);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
