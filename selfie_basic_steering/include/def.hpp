#pragma once
#include <vector>

#define DET_NUM 3
#define LIDAR_POS_X 0
#define LIDAR_POS_Y 0

namespace cv
{
struct Point
{
    int x;
    int y;
};
}

struct LidarReading
{
    std::vector<cv::Point> pos;
    std::vector<float> angle;
};

struct SteerData
{
    float offset[DET_NUM];
    float slope[DET_NUM-1];
};
