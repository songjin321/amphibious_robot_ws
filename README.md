# 两栖机器人ROS代码仓库
## 简介
两栖机器人野外环境下视觉定位ROS主仓库。[项目主页](https://git.nrs-lab.com/amphirobot/projectmanagement)
## 任务分配
每个人负责的模块写成一个ros包，各人在各自包的分支下进行开发。
1. ar_slam_ros(宋瑾)
  
  [ar_slam](https://git.nrs-lab.com/amphirobot/svae-slam)的ros接口，订阅相机图像，发布相机位姿，当前帧的特征点数量和滑出窗口的估计完成的特征点三维坐标

2. exposure_controller(王煜)

  订阅当前的相机图像，修改相机的曝光时间

3. information_filed(朱西)

  订阅地图点，提供服务，查询信息最大视角

4. active_localization（王志濠）

  由相机位姿，机器人运动方向计算最优视角

5. view_controller(王志濠)

发布一个服务：call一下可以调整视角

6. navigation
  
  最上层的launch文件

7. move_base
  
  TODO::两栖机器人的轨迹规划，轨迹跟踪和底层的控制

## 编译安装
### 1.安装依赖
- glog      
sudo apt install libgoogle-glog-dev
- eigen3    
sudo apt install libeigen3-dev
- ceres     
https://github.com/ceres-solver/ceres-solver
- opencv
- ros包依赖  
rosdep install --from-paths src --ignore-src -r -y（**每一个包都要在自己的package.xml中写好相关的依赖**）
### 2.下载代码
```bash
git clone https://git.nrs-lab.com/amphirobot/amphibious_robot_ws-.git
```
### 3.编译
1. sudo apt-get install ros-kinetic-catkin python-catkin-tools（安装catkin）

2. catkin build

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
 