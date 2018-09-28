#pragma once

#include <iostream>
#include <math.h>

#include <include/def.hpp>

class Process
{
public:
    float angle_min;
    std::vector<long> raw_data;

private:
    LidarReading all_points;
    LidarReading all_simplified;
    LidarReading left_points;
    LidarReading right_points;
    LidarReading rejected_points;
    LidarReading enemies_points;
    LidarReading trash_points;

public:
    int max_dist = 45;
    int thresh_simplify = 10;

    cv::Point left_det[DET_NUM];
    cv::Point right_det[DET_NUM];
    cv::Point mid_det[DET_NUM];

    int offset[DET_NUM];
    float slope[DET_NUM-1];

public:
    Process();
    void polar_to_cartesian();
    void simplify_data();
    void split_poins_equally();
    void search_points();
    void calc_mid();
    void calc_offsets();
    void calc_slopes();
    void filter_enemies();
    void pack_data(SteerData &out);
    void display_data();
};
