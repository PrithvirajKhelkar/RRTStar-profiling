#pragma once
#ifndef SIMPLERRTSTAR_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define SIMPLERRTSTAR_H

#include <opencv2/opencv.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace cv;


double planAndVisualizePath(const std::string& imagePath, int start_x, int start_y, int goal_x, int goal_y, const std::string& outputPath);

void DrawRRTStarPath();

#endif