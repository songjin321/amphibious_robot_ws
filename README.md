# 两栖机器人ROS代码仓库
## 简介
两栖机器人野外环境下视觉定位ROS主仓库。[项目主页](https://git.nrs-lab.com/amphirobot/projectmanagement)
## 任务分配
每个人负责的模块写成一个ros包，各人在各自包的分支下进行开发。
1. active_slam(宋瑾+朱西)
[ar_slam](https://git.nrs-lab.com/amphirobot/svae-slam)的ros接口，实现鲁棒的vio，并计算当前最优视角

2. exposure_controller(王煜)
通过主动控制曝光时间来处理野外光线变化问题，更利于特征的提取和VO的计算

3. view_controller(王志濠)
接收ar_slam_ros计算的最优视角，并控制机器人视角转动

4. navigation
TODO::两栖机器人的轨迹规划，轨迹跟踪和底层的控制


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
