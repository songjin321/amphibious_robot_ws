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
#include <string>
#include "arslam/Subscriber.hpp"
#include "arslam/Publisher.hpp"
#include "arslam/FixlagFeatureVIO.hpp"
#include "arslam/VioParametersReader.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_slam_node");

    ros::NodeHandle nh("ar_slam_node");

    // initialise logging
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1; // 设置带颜色输出
    // read configuration file
    std::string configFilename;
    if(!nh.getParam("config_filename",configFilename)) {
        LOG(ERROR) << "Please specify filename of configuration!";
        return -1;
    }
    DLOG(INFO) << "config file name = " << configFilename;
    arslam::VioParametersReader vio_parameters_reader(configFilename);
    arslam::VioParameters parameters;
    if(!vio_parameters_reader.getParameters(parameters))
      LOG(ERROR) << "read vio parameters error! ";
    DLOG(INFO) << "sigma_bg  = " << parameters.imu_parameters.sigma_bg;
    // initialise estimator
    arslam::FixlagFeatureVIO estimator(parameters);

    // set publisher
    arslam::Publisher publisher(nh);
    estimator.setStateCallback(std::bind(&arslam::Publisher::publishStateAsCallback, &publisher,std::placeholders::_1,std::placeholders::_2));

    // subscriber loop
    arslam::Subscriber subscriber(nh, estimator);
    while (ros::ok()) {
      ros::spinOnce();
    }
    return 0;
}

