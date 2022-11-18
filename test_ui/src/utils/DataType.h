#pragma once

#include <opencv2/opencv.hpp>

struct ImgData
{
    size_t  time_stamp = 0;
    cv::Mat image;
};

struct ImuData
{
    size_t time_stamp = 0;
    float  w_x        = 0.0f;
    float  w_y        = 0.0f;
    float  w_z        = 0.0f;
    float  a_x        = 0.0f;
    float  a_y        = 0.0f;
    float  a_z        = 0.0f;
};