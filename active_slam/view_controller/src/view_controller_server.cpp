#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "view_controller/ControlView.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <cmath>
#include <iostream>
ros::Publisher pub;
ros::Publisher best_view_pub;

/****************************
* 本程序演示了 Eigen 几何模块的使用方法
****************************/
void XYZ_euler(Eigen::Vector3d& b_view, Eigen::Vector3d& euler_angles){
    // cout.precision(3);
    // cout<<"Vector3d =\n"<<b_view <<endl;
    // Eigen/Geometry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    Eigen::Vector3d ini_vector(1,0,0);
    // Eigen::Vector3d cross_vec = ini_vector.cross(b_view);
    // cross_vec = cross_vec.normalized();
    // double angle = ini_vector.dot(b_view);
    // double angle_ = acos(angle);
    // // ROS_INFO("angle_ = %f", angle_);
    // // ROS_INFO("cross_vec x = %f, y = %f, z = %f", cross_vec.x(), cross_vec.y(), cross_vec.z());
    // Eigen::AngleAxisd rotation_vector ( angle_, cross_vec);     //沿 Z 轴旋转 45 度
    Eigen::AngleAxisd rotation_vector (
            std::acos(b_view.dot(ini_vector)), 
            ini_vector.cross(b_view).normalized()); 
    // cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;                //用matrix()转换成矩阵
    // 也可以直接赋值
    rotation_matrix = rotation_vector.toRotationMatrix();

    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
    euler_angles = rotation_matrix.eulerAngles ( 2,0,1 ); // 2 1 0 ZYX顺序，即roll pitch yaw顺序
    // ROS_INFO("euler_angles-- x = %f, y = %f, z = %f", euler_angles.x(), euler_angles.y(), euler_angles.z());
    
// [ INFO] [1570850715.845532104]: set view to   x = 0.963941, y = -0.089408, z = 0.250648
// [ INFO] [1570850715.845555993]: euler_angle x = 3.049105, y = -2.888243, z = -3.129805
// the result : -0.093 -0.254 0.012

    // cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;
}

bool view_contoller_server_callback(view_controller::ControlView::Request &req,
                            view_controller::ControlView::Response &res)
{
    // TODO: 实现云台控制
    Eigen::Vector3d view;
    Eigen::Vector3d euler_angle;
    // x = 0.555302, y = -0.009440, z = -0.831596
//    x = 0.208475, y = -0.740692, z = -0.638682
// 0.648250, y = 0.661435, z = -0.377062
//    req.best_view.x = 0.648250;
//    req.best_view.y = 0.661435;
//    req.best_view.z = 0.377062;
    //    add nan process
    if (std::isnan(req.best_view.x) || std::isnan(req.best_view.y) || std::isnan(req.best_view.z))
    {
        ROS_INFO("set view Failed!!!");
        res.is_finished = false;
        return false;
    }
//    ROS_INFO("set view to   x = %f, y = %f, z = %f ", req.best_view.x, req.best_view.y, req.best_view.z);

    if (std::abs(req.best_view.x) <= 0.0001 && std::abs(req.best_view.y) <= 0.0001 && std::abs(req.best_view.z) <= 0.0001)
    {
        ROS_INFO("Best view 0......");
        res.is_finished = false;
        return false;
    }

    view.x() = req.best_view.x;
    view.y() = req.best_view.y;
    view.z() = req.best_view.z;

    Eigen::MatrixXd trans_view_gimbal(3,3);
    trans_view_gimbal << 1,0,0,
            0,0,-1,
            0,-1,0;
    view = trans_view_gimbal * view;

    ROS_INFO("set view to   x = %f, y = %f, z = %f \n", view.x(), view.y(), view.z());

    XYZ_euler(view, euler_angle);
    ROS_INFO("euler_angle x = %f, y = %f, z = %f", euler_angle.x(), euler_angle.y(), euler_angle.z());

    geometry_msgs::Vector3 euler_angle_MCU;     // pitch roll yaw

    if(req.best_view.x >= 0){
        if(req.best_view.z >= 0){
            if(req.best_view.y < 0){
                if(euler_angle.x() > 0){
                    euler_angle.x() = 3.1415926 - euler_angle.x();
                }
                else if(euler_angle.x() < 0){
                    euler_angle.x() = 3.1415926 + euler_angle.x();
                }

                if(euler_angle.y() > 0){
                    euler_angle.y() = 3.1415926 - euler_angle.y();
                }
                else if(euler_angle.y() < 0){
                    euler_angle.y() = 3.1415926 + euler_angle.y();
                }

                if(euler_angle.z() > 0){
                    euler_angle.z() = 3.1415926 - euler_angle.z();
                }
                else if(euler_angle.z() < 0){
                    euler_angle.z() = 3.1415926 + euler_angle.z();
                }

                euler_angle_MCU.x = -euler_angle.x();
                euler_angle_MCU.y = euler_angle.y();
                euler_angle_MCU.z = -euler_angle.z();
            }
            else {
                if(euler_angle.x() > 0){
                    euler_angle.x() = 3.1415926 - euler_angle.x();
                }
                else if(euler_angle.x() < 0){
                    euler_angle.x() = 3.1415926 + euler_angle.x();
                }

                if(euler_angle.y() > 0){
                    euler_angle.y() = 3.1415926 - euler_angle.y();
                }
                else if(euler_angle.y() < 0){
                    euler_angle.y() = 3.1415926 + euler_angle.y();
                }

                if(euler_angle.z() > 0){
                    euler_angle.z() = 3.1415926 - euler_angle.z();
                }
                else if(euler_angle.z() < 0){
                    euler_angle.z() = 3.1415926 + euler_angle.z();
                }

                euler_angle_MCU.x = -euler_angle.x();
                euler_angle_MCU.y = euler_angle.y();
                euler_angle_MCU.z = euler_angle.z();
            }
        }
        else{
//            if(req.best_view.y < 0){
//                if(euler_angle.x() > 0){
//                    euler_angle.x() = 3.1415926 - euler_angle.x();
//                }
//                else if(euler_angle.x() < 0){
//                    euler_angle.x() = 3.1415926 + euler_angle.x();
//                }

//                if(euler_angle.y() > 0){
//                    euler_angle.y() = 3.1415926 - euler_angle.y();
//                }
//                else if(euler_angle.y() < 0){
//                    euler_angle.y() = 3.1415926 + euler_angle.y();
//                }

//                if(euler_angle.z() > 0){
//                    euler_angle.z() = 3.1415926 - euler_angle.z();
//                }
//                else if(euler_angle.z() < 0){
//                    euler_angle.z() = 3.1415926 + euler_angle.z();
//                }
//                euler_angle_MCU.x = -euler_angle.x();
//                euler_angle_MCU.y = euler_angle.y();
//                euler_angle_MCU.z = euler_angle.z();
//            }
//            else {

//                euler_angle_MCU.x = euler_angle.x();
//                euler_angle_MCU.y = euler_angle.y();
//                euler_angle_MCU.z = euler_angle.z();
//            }

            euler_angle_MCU.x = euler_angle.x();
            euler_angle_MCU.y = euler_angle.y();
            euler_angle_MCU.z = euler_angle.z();
        }
    }
    else{       // x<0
        ROS_INFO("X < 0......");
        res.is_finished = false;
        return false;

        if(req.best_view.z >= 0){       // z>0
            if(req.best_view.y < 0){
                if(euler_angle.x() > 0){
                    euler_angle.x() = 3.1415926 - euler_angle.x();
                }
                else if(euler_angle.x() < 0){
                    euler_angle.x() = 3.1415926 + euler_angle.x();
                }

                if(euler_angle.y() > 0){
                    euler_angle.y() = 3.1415926 - euler_angle.y();
                }
                else if(euler_angle.y() < 0){
                    euler_angle.y() = 3.1415926 + euler_angle.y();
                }

                if(euler_angle.z() > 0){
                    euler_angle.z() = 3.1415926 - euler_angle.z();
                }
                else if(euler_angle.z() < 0){
                    euler_angle.z() = 3.1415926 + euler_angle.z();
                }

                euler_angle_MCU.x = -euler_angle.x();
                euler_angle_MCU.y = euler_angle.y();
                euler_angle_MCU.z = -euler_angle.z();
            }
            else {
                euler_angle_MCU.x = euler_angle.x();
                euler_angle_MCU.y = euler_angle.y();
                euler_angle_MCU.z = euler_angle.z();
            }
        }
        else{       // z<0
            if(req.best_view.y < 0){       // y<0
//                if(euler_angle.x() > 0){
//                    euler_angle.x() = 3.1415926 - euler_angle.x();
//                }
//                else if(euler_angle.x() < 0){
//                    euler_angle.x() = 3.1415926 + euler_angle.x();
//                }

//                if(euler_angle.y() > 0){
//                    euler_angle.y() = 3.1415926 - euler_angle.y();
//                }
//                else if(euler_angle.y() < 0){
//                    euler_angle.y() = 3.1415926 + euler_angle.y();
//                }

//                if(euler_angle.z() > 0){
//                    euler_angle.z() = 3.1415926 - euler_angle.z();
//                }
//                else if(euler_angle.z() < 0){
//                    euler_angle.z() = 3.1415926 + euler_angle.z();
//                }
                euler_angle_MCU.x = euler_angle.x();
                euler_angle_MCU.y = euler_angle.y();
                euler_angle_MCU.z = euler_angle.z();
            }
            else {

                euler_angle_MCU.x = euler_angle.x();
                euler_angle_MCU.y = euler_angle.y();
                euler_angle_MCU.z = euler_angle.z();
            }
        }
    }

    //  0.767129, y = -0.402214, z = -0.935554
//    euler_angle_MCU.x = 0.767129;
//    euler_angle_MCU.y = 0.402214;
//    euler_angle_MCU.z = 0.935554;
//    euler_angle_MCU.x = euler_angle.x();
//    euler_angle_MCU.y = euler_angle.y();
//    euler_angle_MCU.z = euler_angle.z();
//    ROS_INFO("output x = %f, y = %f, z = %f", euler_angle_MCU.x, euler_angle_MCU.y, euler_angle_MCU.z);

    best_view_pub.publish(euler_angle_MCU);
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
    
    best_view_pub = nh.advertise<geometry_msgs::Vector3>("/best_view_robot", 1);
    // pub = nh.advertise<geometry_msgs::Vector3>("/view_robot_camera_curr", 1);

    // TODO::create a thread to get gimbal angle and publish it.
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        //
        // geometry_msgs::Vector3 view_robot_camera_curr;
        // view_robot_camera_curr.x = 1;
        // view_robot_camera_curr.y = 0;
        // view_robot_camera_curr.z = 0;
        // pub.publish(view_robot_camera_curr);
        ros::spinOnce();
        loop_rate.sleep();
    }

}
