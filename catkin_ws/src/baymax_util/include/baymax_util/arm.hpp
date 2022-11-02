#pragma once

#include <opencv2/opencv.hpp>

typedef cv::Matx33d RotationMat;
typedef cv::Matx31d VecMat;

struct joint
{
    VecMat axis;
    VecMat disp;
};

class dofbot_arm
{

};
