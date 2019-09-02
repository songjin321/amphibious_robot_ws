/**
 * @file main.cpp
 * @author SongJin (songjinxs@163.com)
 * @brief A interface with ROS
 * @version 0.1
 * @date 2019-06-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <ros/ros.h>
#include <glog/logging.h>
#include "arslam/Subscriber.hpp"
#include "arslam/Publisher.hpp"
//#include "arslam/FixlagFeatureVIO.hpp"
//#include "arslam/Parameters.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_slam_node");

    ros::NodeHandle nh("ar_slam_node");

    // initialise logging
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1;
    LOG(INFO) << "Hello";
    // publisher
    arslam::Publisher publisher(nh);

    // read configuration file


    // initialise estimator


    // subscriber
}

