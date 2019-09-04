/**
 * @file Publisher.hpp
 * @author Song Jin (songjinxs@163.com)
 * @brief 
 * @version 0.1
 * @date 2019-09-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <arslam/Time.hpp>
namespace arslam
{
class Publisher
{
public:
  /**
   * @brief Constructor. Calls setNodeHandle().
   * @param nh The ROS node handle for publishing.
   */
  Publisher(ros::NodeHandle& nh);

    /**
   * @brief Set and publish pose.
   * @remark This can be registered with the VioInterface.
   * @param t     Timestamp of pose.
   * @param T_WS  The pose.
   */
  void publishStateAsCallback(const arslam::Time& t,
                              const Eigen::Isometry3d& T_WB);
private:
    ros::NodeHandle nh_;
};


}