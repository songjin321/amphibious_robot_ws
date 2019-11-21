//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <rpg_common/pose.h>

namespace ros {
class Time;
}  // namespace ros

namespace rpg_common_ros {
namespace tf {

void initListener();

// Hard variants.
rpg::Pose get_T_A_B(const std::string& A_name, const std::string& B_name);
rpg::Pose get_T_A_B(const std::string& A_name, const std::string& B_name,
                    const ros::Time& time);
// Soft variants.
bool get_T_A_B(
    const std::string& A_name, const std::string& B_name, rpg::Pose* T_A_B);
bool get_T_A_B(
    const std::string& A_name, const std::string& B_name,
    const ros::Time& time, rpg::Pose* T_A_B);

}  // namespace tf
}  // namespace rpg_common_ros
namespace rpg_ros = rpg_common_ros;
