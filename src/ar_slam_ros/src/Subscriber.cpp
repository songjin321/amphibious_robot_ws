/**
 * @file Subscriber.cpp
 * @author song jin (songjinxs@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-09-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "arslam/Subscriber.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "arslam/Time.hpp"
#include <glog/logging.h>
namespace arslam
{
    Subscriber::Subscriber(ros::NodeHandle& nh, FixlagFeatureVIO& estimator)
    :p_estimator_(&estimator)
    {
        // set image and imu callback
        subImu_ = nh.subscribe("/imu", 1000, &Subscriber::imuCallback, this, ros::TransportHints().tcpNoDelay());
        sunImage_ = nh.subscribe("/image", 100, &Subscriber::imageCallback, this);
    }

    void Subscriber::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
    {
        // DLOG(INFO) << "receive a imu, timestamp " << std::fixed << std::setprecision(15) <<  convertTime2double(imu_msg->header.stamp.sec, imu_msg->header.stamp.nsec);
        p_estimator_->addImuData(
            imu_msg->header.stamp.toSec(),
            Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z),
            Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
                            imu_msg->angular_velocity.z)
        );
    }

    void Subscriber::imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
    {
        // DLOG(INFO) << "receive a image, timestamp:" << std::fixed << std::setprecision(15) << convertTime2double(img_msg->header.stamp.sec, img_msg->header.stamp.nsec);
        // convert ros image to cv mat, color to gray
        cv_bridge::CvImageConstPtr ptr;
        try
        {
            ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image;
        if (ptr->encoding == "rgb8")
            cv::cvtColor(ptr->image, image, cv::COLOR_RGB2GRAY);
        else if (ptr->encoding == "bgr8")
            cv::cvtColor(ptr->image, image, cv::COLOR_BGR2GRAY);
        else if (ptr->encoding == "mono8")
            image = ptr->image;
        else 
            LOG(WARNING) << "unclear image encode type";

        p_estimator_->addImage(
            img_msg->header.stamp.toSec(),
            image
            );
        
    }
}