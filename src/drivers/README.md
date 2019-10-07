# How to install realsense d435i driver in TX2

1. use JetPack 3.3 to flash the OS of TX2
2. use [buildLibrealsense2TX](https://github.com/jetsonhacks/installLibrealsenseTX2.git) to patch the kernal and build librealsense 2.19.1
3. use ethz [vi_realsenses](https://github.com/ethz-asl/vi_realsense) to inteplolate acc and gyro raw message. the default interplolation method in realsense-ros will result in **imu message disorder** with vins-mono.
4. build ceres 1.14 with eigen 3.3.7. [ceres-solver/ceres-solver#288](https://github.com/ceres-solver/ceres-solver/issues/288)
5. build vins_estimator with -j1 to prevent memory explosion
