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
        p_estimator_->addImuData(
            convertTime2double(imu_msg->header.stamp.sec, imu_msg->header.stamp.nsec),
            Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z),
            Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
                            imu_msg->angular_velocity.z)
        );
    }

    void Subscriber::imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
    {
        // convert ros image to cv mat
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat image = ptr->image;

        p_estimator_->addImage(
            convertTime2double(img_msg->header.stamp.sec, img_msg->header.stamp.nsec), 
            image
            );
    }
}