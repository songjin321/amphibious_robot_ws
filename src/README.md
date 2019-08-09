# 两栖机器人ROS代码仓库
## 简介
1. slam(宋瑾+朱西)
ar_slam的ros接口，实现鲁棒的vio，并计算当前最优视角
https://github.com/zhangzichao

2. exposure_controller(王煜)
通过主动控制曝光时间来处理野外光线问题

3. manipulate_controller(王志濠)
接收slam计算的最优视角，并控制机器人视角转动

4. navigation
机器人的顶层导航栈，现阶段无路径规划和机器人底层控制部分,TODO::两栖机器人的轨迹规划，轨迹跟踪和底层的控制


## 编译安装
### 1.安装依赖
- glog      
sudo apt install libgoogle-glog-dev
- eigen3    
sudo apt install libeigen3-dev
- ceres     
https://github.com/ceres-solver/ceres-solver
- ros包依赖  
rosdep install --from-paths src --ignore-src -r -y（**每一个包都要在自己的package.xml中写好相关的依赖**）
### 2.下载代码
git clone --recursive 

### 3.编译
1. sudo apt-get install ros-kinetic-catkin python-catkin-tools（安装catkin）

2. catkin build

## 运行测试

### 1. 跑包

从实验室网盘上下载包
http://file.nrs-lab.com/f/158206

1. roslaunch navigation realsense.launch
2. rosbag play demo.bag
