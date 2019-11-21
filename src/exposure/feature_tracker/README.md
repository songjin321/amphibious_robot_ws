# 一个基于曝光补偿的光流跟踪
## 编译
mkdir build && cmake .. && make 
## 运行
```
./feature_tracker NodeName IMAGE_TOPIC
```
同时运行多个,则NodeName取不同名字即可

### 发布的话题
1. "NodeName/tracker_image", 光流追踪显示,绿色表示追踪上的,蓝色表示追踪丢失,红色表示新增加的
2. "NodeName/correct_image", 利用相机响应曲线矫正的图像
3. "NodeName/tracked_nmt", 当前帧追踪的特征点数量,可以用rqt_plot绘制出来

### 订阅的话题
1. IMAGE_TOPIC, 相机捕获的图像,其中header.frame_id中需要保存该图像的曝光时间.