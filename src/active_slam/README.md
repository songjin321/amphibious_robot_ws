# active_slam

## 简介
后期可以控制两栖机器人的目标位置，实现
active slam的效果：机器人主动对环境进行探索
这个包根据slam的结果来确定机械颈的姿态，当前机器人的位置和地图点以及图像

现阶段只能直接控制机器人的视角，最好可以规划一条轨迹，实现active slam的效果

## 接口

1. 订阅的信息

信息场计算的视角信息值 话题名称： /act_map/view_info 话题类型： act_map_msgs::ViewInformation

view_controller发布的当前相机在机器人坐标系下的视角 话题名称： /view_robot_camera_curr 话题类型： geometry_msgs::Vector3
2. 发布的信息

计算出的最优视角，用于可视化 话题名称：/active_slam_node/best_view 话题类型: visualization_msgs::Marker

call一个服务，控制相机视角，服务名称：/view_controller_server