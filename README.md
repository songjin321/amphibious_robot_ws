# 两栖机器人ROS代码仓库
## 简介
两栖机器人野外环境下视觉定位ROS主仓库。[项目主页](https://git.nrs-lab.com/amphirobot/projectmanagement)

## 编译安装

### 1.安装Ros Kinetic+Ubuntu16.04

### 2.新建工作空间，下载源代码到src文件夹
```
mkdir -p amphi_ws/src && cd amphi_ws/src
git clone https://git.nrs-lab.com/amphirobot/amphibious_robot_ws-.git AmphiActiveVIO
```
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
- Fast
```
git clone https://github.com/uzh-rpg/fast.git
cd fast
mkdir build
cd build
cmake ..
make
sudo make install
```
- Protobuf
```
sudo apt install protobuf-compiler
```

如果在tx2下需要特别版本的ceres和eigen
- ceres 1.14 with eigen 3.3.7. [ceres-solver/ceres-solver#288](https://github.com/ceres-solver/ceres-solver/issues/288)

### 4.安装相机驱动
1. tx2+realsensed435i

2. 正常x86+realsensed435i

### 5.编译
1. sudo apt-get install ros-kinetic-catkin python-catkin-tools（安装catkin）
2. catkin build

3. 如果在tx2上，为了防止内存爆炸
catkin build vins_estimator -j1

4. 如果没有安装相机驱动,只打算跑包,则可以不编译曝光控制部分
```
catkin config --blacklist realsense2_camera exposure_controller
```
5. 默认不使用GPU编译,如果要使用GPU-SFIT
```
catkin build feature_tracker -DENABLE_GPU=ON
```
## 运行测试

### 1. 测试AmhpiVIO

从实验室网盘上下载包[amphi_vio.bag]()

1. roslaunch vins_estimator nrsl_d435i.launch
2. rosbag play amphi_vio.bag 或者使用相机 roslaunch realsense2_camera rs_maplab.launch
3. roslaunch navigation rviz.launch

### 2. 测试ActiveLocalization

从实验室网盘上下载包[active_localization.bag]()

1. roslaunch vins_estimator nrsl_d435i.launch
2. roslaunch active_localization active_localization.launch
3. rosbag play active_localization.bag
4. roslaunch navigation rviz.launch

### 3. 测试ActiveExposure 

1. roslaunch exposure_controller exposure_controller.launch

## experiments
