/**
 * @file Subscriber.hpp
 * @author song jin (songjinxs@163.com)
 * @brief 
 * @version 0.1
 * @date 2019-09-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "arslam/FixlagFeatureVIO.hpp"
namespace arslam
{
class Subscriber
{
public:
    /**
     * @brief Construct a new Subscriber object
     * 
     * @param nh 
     * @param estimator 
     */
    Subscriber(ros::NodeHandle& nh, FixlagFeatureVIO& estimator);

private:
    /// @brief The image callback.
    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);

    /// @brief The IMU callback.
    void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

    FixlagFeatureVIO* p_estimator_; ///< The estimator pointer;
    ros::Subscriber subImu_;  ///< The IMU message subscriber.
    ros::Subscriber sunImage_; ///< The Image message subscirber.
};
}