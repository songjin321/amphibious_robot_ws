#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include "information_filed/"

void information_callback()
{
    
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "active_slam");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Subscriber sub_information = n.subscribe(“/information_filed/maximum_current_view”, 1,
     information_callback);

    pub_best_view_show = n.advertise<feature_tracker::Feature>("feature", 1);

    ros::spin();
    return 0;
}