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
#include "vio_estimator/System.h"
#include <sensor_msgs/Image.h>
#include <glog/logging.h>

class ImageGrabber
{
public:
    ImageGrabber(AR_SLAM::System& SLAM):mSLAM(SLAM){}
    void GrabImage(const sensor_msgs::ImageConstPtr& msg)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        slam.processNewImage(cv_ptr->image,cv_ptr->header.stamp.toSec());
        slam.processNewIMU(cv_ptr->image,cv_ptr->header.stamp.toSec());
    }
private:
    AR_SLAM::System& slam;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_slam_node");
    ros::start();

    // set up the node
    ros::NodeHandle nh("ar_slam_node");

    // initialise logging
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1;    

    // publisher
    Publisher publisher(nh)

    if(argc != 2)
    {
        std::cerr << std::endl << "Usage: rosrun AR_SLAM ar_slam path_to_settings" << std::endl;
        ros::shutdown();
        return 1;
    }

    AR_SLAM::System SLAM(argv[1]);

}