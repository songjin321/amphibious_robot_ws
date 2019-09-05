/**
 * @file Publisher.cpp
 * @author song jin (songjinxs@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-09-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "arslam/Publisher.hpp"
namespace arslam{

Publisher::Publisher(ros::NodeHandle& nh)
:nh_(nh)
{
}

void Publisher::publishStateAsCallback(const arslam::Time& t,
                            const Eigen::Isometry3d& T_WB)
{
                                
}

}
