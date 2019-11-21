# exposure_controller

主动曝光控制

## 简介

根据相机采集的图片，实时对相机的曝光参数进行修改,与vins-mono中的图像处理方法进行对比

## 包含的包

1. dynamic_reconfigure

    用来动态控制相机参数

2. exposure_controller  

    订阅相机捕捉的图片 话题名称： /camera/color/image_raw 话题类型： sensor_msgs::Image

    发布要设置的相机的曝光时间和曝光增益 话题名称： /camera/set_exposure 话题类型： std_msgs::Int32MultiArray

3. feature_tracker

    一个简单的光流跟踪小程序,用来测试主动曝光的效果

## 使用
接上realsense d435,先启动相机驱动,然后启动主动曝光控制
```
roslaunch realsense2_camera rs_maplab.launch
roslaunch exposure_controller exposure_controller.launch
```