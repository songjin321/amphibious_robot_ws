# active_slam

## 简介
实现active slam的效果：机器人主动对环境进行探索并保持视觉定位的有效性

现阶段只能实现active localization,只计算了一个最优视角,并控制相机视角看向最优视角

## 包含的模块

### 1.information_filed

基于rpg_information_filed, 使用ceres优化的方法计算最大信息视角

### 2.active_localization

通过平衡机器人场景探索和视觉定位两部分的要求计算一个最优视角,实现主动定位

    订阅的信息: 信息场计算的视角信息值 话题名称： /act_map/view_info 话题类型： act_map_msgs::ViewInformation

    view_controller发布的当前相机在机器人坐标系下的视角 话题名称： /view_robot_camera_curr 话题类型： geometry_msgs::Vector3

    发布计算出的最优视角，用于可视化 话题名称：/active_slam_node/best_view 话题类型: visualization_msgs::Marker 并call一个服务，控制相机视角，服务名称：/view_controller_server

### 3.view_controller
和云台通信，用来控制云台的运动

    提供一个服务用来控制相机姿态     服务名称：/view_controller_server   服务类型：view_controller::ControlView

    发布的当前相机在机器人坐标系下的视角 话题名称： /view_robot_camera_curr 话题类型： geometry_msgs::Vector3

## 使用
```
roslaunch active_localization active_localization.launch
```