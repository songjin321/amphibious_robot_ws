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
private:
    ros::NodeHandle nh_;
};


}