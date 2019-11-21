# information_field

## 简介
code of fisher information feild that shared by the author 

信息场

使用rpg_information_filed+voxblox

目标是使其在tx2上能实时跑起来，整个系统。

后期可以控制两栖机器人的目标位置，实现
active slam的效果：机器人主动对环境进行探索
这个包根据slam的结果来确定机械颈的姿态，当前机器人的位置和地图点以及图像

## 接口

1. 订阅的信息
订阅vio估计出来的相机在世界坐标系的位姿 话题名称：/vins_estimator/camera_pose 话题类型：geometry_msgs::PoseStamped

订阅vio估计的当前帧的点云数据 话题名称：/vins_estimator/point_cloud 话题类型：sensor_msg:PointCloud2

2. 发布的信息
信息场计算的视角信息值 话题名称： /act_map/view_info  话题类型：act_map_msgs::ViewInformation

## 仿真实验

1. 基于优化的最大信息视角计算
rosrun act_map exp_optim_orient_nrsl.py two_walls --v=1 

2. 仿真测试最优视角
rosrun act_map exp_best_orient_nrsl.py four_walls --v=1

3. 画图
rosrun act_map exp_plot.py 

## 使用
实验室网盘上的包[active_view.bag](http://file.nrs-lab.com/s/SPkpkPfx4fDfG59)
基于采样的
```
roslaunch act_map_ros nrsl_d435i.launch
roslaunch vins_estimator nrsl_d435i.launch 
roslaunch navigation rviz.launch
rosbag play active_view.bag
```
基于优化的
```
roslaunch act_map_ros nrsl_optimization.launch
roslaunch vins_estimator nrsl_d435i.launch 
roslaunch navigation rviz.launch
rosbag play active_view.bag
```