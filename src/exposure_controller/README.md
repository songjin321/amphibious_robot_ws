# exposure_controller

主动控制曝光

## 简介

根据相机采集的图片，实时对相机的曝光参数进行修改,与vins-mono中的图像处理方法进行对比

## 接口

1. 订阅的信息

相机捕捉的图片 话题名称： /camera/color/image_raw 话题类型： sensor_msgs::Image

2. 发布的信息

要设置的相机的曝光时间和曝光增益 话题名称： /camera/set_exposure 话题类型： std_msgs::Int32MultiArray
