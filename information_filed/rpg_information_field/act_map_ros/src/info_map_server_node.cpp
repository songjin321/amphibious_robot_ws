//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map_ros/act_map_server.h"

#include <rpg_common/main.h>


RPG_COMMON_MAIN
{
  ros::init(argc, argv, "infomap_server_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO_STREAM("info_map_server_node.cpp has been called");

  act_map_ros::InfoMapServer actmap_server(nh, pnh);

  ros::spin();

  ROS_ERROR_STREAM("node has been killed!!!!!");
  return 0;
}
