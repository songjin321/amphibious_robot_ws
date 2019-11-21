# 两栖机器人ROS代码仓库
## 简介
两栖机器人野外环境下视觉定位ROS主仓库。[项目主页](https://git.nrs-lab.com/amphirobot/projectmanagement)

## 编译安装

### 1.安装Ros Kinetic+Ubuntu16.04

### 2.新建工作空间，下载源代码到src文件夹

git clone https://git.nrs-lab.com/amphirobot/amphibious_robot_ws-.git

### 3.安装依赖
正常x86的ubuntu
- glog      
sudo apt install libgoogle-glog-dev
- eigen3    
sudo apt install libeigen3-dev
- ceres     
https://github.com/ceres-solver/ceres-solver
- Sophus
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
make
sudo make install
```

如果在tx2下需要
- ceres 1.14 with eigen 3.3.7. [ceres-solver/ceres-solver#288](https://github.com/ceres-solver/ceres-solver/issues/288)

### 4.安装驱动
1. tx2+realsensed435i

2. 正常x86+realsensed435i

3. 只是想跑包,可以将drivers中的包加入黑名单
- catkin config --blacklist realsense2_camera 

### 5.编译
1. sudo apt-get install ros-kinetic-catkin python-catkin-tools（安装catkin）
2. catkin build

如果在tx2上，为了防止内存爆炸

1. build vins_estimator with -j1

没有相机驱动,不编译曝光控制部分
catkin config --blacklist realsense2_camera exposure_controller

默认不使用GPU编译,如果要使用GPU-SFIT
catkin build feature_tracker -DENABLE_GPU=ON
## 运行测试

### 1. 测试ar_slam

从实验室网盘上下载包
http://file.nrs-lab.com/f/158206

1. roslaunch navigation demo_bag.launch
2. rosbag play demo.bag

### 2. 测试XXX

## 开发流程
1. git clone https://git.nrs-lab.com/amphirobot/amphibious_robot_ws-.git
2. git checkout -b develop-view_controller origin/develop-view_controller(这个替换成自己需要开发的远程分支名)
3. 进行开发，疯狂commit，每日一次，神清气爽
4. git push 推送到服务器上
5. 开发得差不多了，在gitlab上使用create merge request合并到develop上
 
