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
//#include "arslam/FixlagFeatureVIO.hpp"
#include "arslam/VioParametersReader.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_slam_node");

    ros::NodeHandle nh("ar_slam_node");

    // initialise logging
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    // publisher
    arslam::Publisher publisher(nh);

    // read configuration file
    std::string configFilename;
    if(!nh.getParam("config_filename",configFilename)) {
        LOG(ERROR) << "Please specify filename of configuration!";
        return -1;
    }
    arslam::VioParametersReader vio_parameters_reader(configFilename);
    arslam::VioParameters parameters;
    vio_parameters_reader.getParameters(parameters);
    
    /*
    // initialise estimator
    okvis::ThreadedKFVio okvis_estimator(parameters);

    // subscriber loop
    okvis::Subscriber subscriber(nh, &okvis_estimator, vio_parameters_reader);
    while (ros::ok()) {
      ros::spinOnce();
	    okvis_estimator.display();
    }
    */
    return 0;
}

