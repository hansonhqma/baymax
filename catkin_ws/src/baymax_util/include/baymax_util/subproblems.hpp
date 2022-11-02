#pragma once

#include <opencv2/opencv.hpp>
#include <math.h>

namespace Subproblem
{

enum Calc_Result
{
    NO_SOLUTION,
    ONE_SOLUTION,
    TWO_SOLUTIONS
};

Calc_Result Prob_1( cv::Point3d k, cv::Point3d u, cv::Point3d v, double& theta );
Calc_Result Prob_2( cv::Point3d k1, cv::Point3d k2, cv::Point3d u, cv::Point3d v, double& theta1, double& theta2 );

};
