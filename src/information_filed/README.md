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

搞清楚坐标的关系，使用eigen进行转换

1. 订阅的信息
订阅vio估计出来的相机在世界坐标系的位姿 话题名称：/vins_estimator/camera_pose 话题类型：geometry_msgs::PoseStamped

订阅vio估计的当前帧的点云数据 话题名称：/vins_estimator/point_cloud 话题类型：sensor_msg:PointCloud2

2. 发布的信息
订阅信息场计算的最优视角 话题名称： /information_filed/best_view  话题类型： information_filed::information_view