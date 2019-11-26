#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <geometry_msgs/Vector3.h>
#include "act_map_msgs/ViewInformation.h"
#include "view_controller/ControlView.h"
#include <glog/logging.h>
#include <iostream>
using namespace std;
ros::Publisher best_view_show;
ros::ServiceClient view_controller_client;
Eigen::Vector3d view_robot_camera_curr(1,0,0); // 当前相机在机器人坐标系下的视角
Eigen::Matrix3d R_robot_camera_curr_Matrix;
Eigen::AngleAxisd R_robot_camera_curr;

// Eigen::Vector3d view_robot_camera_curr; // 当前相机在机器人坐标系下的视角

void view_info_callback(act_map_msgs::ViewInformation view_info)
{
    // 在机器人坐标系下，相机初始视角
    Eigen::Vector3d view_robot_camera_fixed(1,0,0);
    // ROS_INFO("curr_view   x = %f, y = %f, z = %f", view_info.curr_view.x, view_info.curr_view.y, view_info.curr_view.z);
    // ROS_INFO("max_info_view   x = %f, y = %f, z = %f", view_info.max_info_view.x, view_info.max_info_view.y, view_info.max_info_view.z);
    // 当前相机在世界坐标系下的视角
    Eigen::Vector3d view_world_camera_curr(
        view_info.curr_view.x,
        view_info.curr_view.y,
        view_info.curr_view.z
    );

    // 在世界坐标系下的最大信息视角
    Eigen::Vector3d view_world_camera_max_info(
        view_info.max_info_view.x,
        view_info.max_info_view.y,
        view_info.max_info_view.z
    );

    // 从世界坐标系转换到机器人坐标系
    const Eigen::Vector3d e3(0,1,0);

    Eigen::AngleAxis<double> R_world_camera_max_info(
        std::acos(view_world_camera_max_info.dot(e3)), 
        e3.cross(view_world_camera_max_info).normalized()
        ); 

    Eigen::AngleAxis<double> R_world_camera_curr(
        std::acos(view_world_camera_curr.dot(e3)), 
        e3.cross(view_world_camera_curr).normalized()
        );

//    Eigen::AngleAxis<double> R_robot_camera_curr(
//         std::acos(view_robot_camera_curr.dot(e3)),
//         e3.cross(view_robot_camera_curr).normalized()
//         );
//     ROS_INFO("view_robot_camera_curr   x = %f, y = %f, z = %f", view_robot_camera_curr.x(), view_robot_camera_curr.y(), view_robot_camera_curr.z());
//     std::cout<<"R_robot_camera_curr_Matrix = \n"<<R_robot_camera_curr.matrix() <<endl;
    auto R_robot_camera_max_info = R_robot_camera_curr * R_world_camera_curr.inverse() * R_world_camera_max_info;
    
    Eigen::Vector3d view_robot_camera_max_info = R_robot_camera_max_info * e3;
//    ROS_INFO("view_robot_camera_max_info   x = %f, y = %f, z = %f", view_robot_camera_max_info.x(), view_robot_camera_max_info.y(), view_robot_camera_max_info.z());

    // 计算得到best_view
    double max_info_view_value = view_info.max_info_view_value;
    double curr_view_value = view_info.curr_view_value;
    double alpha = curr_view_value/(curr_view_value + max_info_view_value);
    Eigen::Vector3d view_robot_camera_best;
//    view_robot_camera_best = (alpha*view_robot_camera_fixed + (1-alpha)*view_robot_camera_max_info).normalized();
    view_robot_camera_best = view_robot_camera_max_info;
    ROS_INFO("view_robot_camera_max_info   x = %f, y = %f, z = %f", view_robot_camera_best.x(), view_robot_camera_best.y(), view_robot_camera_best.z());

    // plot best_view
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.ns = "dir";
    m.id = 3;
    m.type = visualization_msgs::Marker::ARROW;
    m.color.g = 1.0;
    m.color.a = 1.0;
    m.action = visualization_msgs::Marker::MODIFY;

    const double shaft_dia = 0.2 * 0.22;
    const double arrow_len = 10 * shaft_dia;

    geometry_msgs::Point start;
    start.x = view_info.position.x;
    start.y = view_info.position.y;
    start.z = view_info.position.z;
    m.points.emplace_back(start);
    geometry_msgs::Point end;
    end.x = start.x + view_robot_camera_best.x() * arrow_len;
    end.y = start.y + view_robot_camera_best.y() * arrow_len;
    end.z = start.z + view_robot_camera_best.z() * arrow_len;
    m.points.emplace_back(end);
    m.scale.x = shaft_dia;        // shaft dia
    m.scale.y = 2.5 * shaft_dia;  // head dia
    m.scale.z = 0.6 * arrow_len;  // head len
    best_view_show.publish(m);

    // call service
    view_controller::ControlView srv;
    srv.request.best_view.x = view_robot_camera_best.x();
    srv.request.best_view.y = view_robot_camera_best.y();
    srv.request.best_view.z = view_robot_camera_best.z();
    // ROS_INFO("view_robot_camera_best   x = %f, y = %f, z = %f", view_robot_camera_best.x(), view_robot_camera_best.y(), view_robot_camera_best.z());
    if(view_controller_client.call(srv))
    {
        ROS_INFO("set gimbal view successfully!");
    }else
    {
        ROS_ERROR("set gimbal view failed!");
    }
}

void view_robot_camera_callback(geometry_msgs::Vector3 msg)
{
    // feedback from stm32 of T_camera_robot
    view_robot_camera_curr.x() = msg.x - 1.5708;
    view_robot_camera_curr.y() = msg.y;
    view_robot_camera_curr.z() = msg.z;
    ROS_INFO("view_robot_camera_curr   x = %f, y = %f, z = %f", view_robot_camera_curr.x(), view_robot_camera_curr.y(), view_robot_camera_curr.z());

    Eigen::Vector3d eulerAngle(view_robot_camera_curr.x(),view_robot_camera_curr.y(),view_robot_camera_curr.z());

    Eigen::AngleAxisd rollAngle(eulerAngle(2),Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(eulerAngle(1),Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(eulerAngle(0),Eigen::Vector3d::UnitZ());
    
    // 将传入的欧拉角直接转换成轴角形式使用
    R_robot_camera_curr_Matrix = yawAngle*pitchAngle*rollAngle;
    R_robot_camera_curr = R_robot_camera_curr_Matrix;

//    std::cout<<"R_robot_camera_curr_Matrix = \n"<<R_robot_camera_curr.matrix() <<endl;
    // view_robot_camera_curr = view_robot_camera_curr.normalized();
//    ROS_INFO("view_robot_camera_curr   x = %f, y = %f, z = %f", view_robot_camera_curr.x(), view_robot_camera_curr.y(), view_robot_camera_curr.z());
    
    // CHECK_DOUBLE_EQ(view_robot_camera_curr.norm(), 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "active_slam");
    ros::NodeHandle nh("~");
    google::InitGoogleLogging(argv[0]);
    ros::Subscriber sub_information = nh.subscribe("/act_map/view_info", 1, view_info_callback);
//    ros::Subscriber sub_view_robot_camera = nh.subscribe("/view_robot_camera_curr", 1, view_robot_camera_callback);
    ros::Subscriber sub_view_robot_camera = nh.subscribe("/chatter_euler1", 1, view_robot_camera_callback);
    view_controller_client = nh.serviceClient<view_controller::ControlView>("/view_controller_server");
    best_view_show = nh.advertise<visualization_msgs::Marker>("best_view", 1);

    ros::spin();
    return 0;
}
