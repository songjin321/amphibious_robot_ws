#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "view_controller/ControlView.h"
#include <Eigen/Geometry>
ros::Publisher pub;
bool view_contoller_server_callback(view_controller::ControlView::Request &req,
                            view_controller::ControlView::Response &res)
{
    // TODO: 实现云台控制
    Eigen::Vector3d view;
    view.x() = req.best_view.x;
    view.y() = req.best_view.y;
    view.z() = req.best_view.z;
    ROS_INFO("set gimbal view to  x = %f, y = %f, z = %f", req.best_view.x, req.best_view.y, req.best_view.z);
    res.is_finished = true;
    return true;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "view_controller_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService<view_controller::ControlView::Request,
                                                    view_controller::ControlView::Response>
                                                    ("/view_controller_server", view_contoller_server_callback); 
    pub = nh.advertise<geometry_msgs::Vector3>("/view_robot_camera_curr", 1);
    // TODO::create a thread to get gimbal angle and publish it.
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        //
        geometry_msgs::Vector3 view_robot_camera_curr;
        view_robot_camera_curr.x = 1;
        view_robot_camera_curr.y = 0;
        view_robot_camera_curr.z = 0;
        pub.publish(view_robot_camera_curr);
        ros::spinOnce();
        loop_rate.sleep();
    }

}