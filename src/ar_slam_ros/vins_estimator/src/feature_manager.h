#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "feature_tracker/Feature.h"
class Estimator;
class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
    int offset; // 特征点第一次观测对应的offset为0
};

class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;
    cv::Mat descriptor; // 描述子
    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

struct ImagePatchCorresponds
{
  int id;                     ///< 这个Patch在滑动窗中的id
  cv::Mat ref_patch;          ///< 在参考帧中的patch块
  cv::Mat ref_patch_warp;     ///< 在参考帧中经过warp affine后的patch块
  int ref_index;              ///< 参考帧在滑动窗中的id
  cv::Mat cur_patch_init;     ///< 在当前帧中直接投影的初始patch块
  cv::Mat cur_patch_final;    ///< 在当前帧中优化后的patch块
  size_t patch_size;          ///< 图像块的大小
  cv::Point2f ref_px;         ///< 在参考帧中的投影位置
  cv::Point2f cur_px_init;    ///< 在当前帧投影的初始位置
  cv::Point2f cur_px_final;   ///< 在当前帧投影的最终位置
  bool match_success;         ///< 光度匹配是否成功
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();

    /**
     * @brief 将frame和地图匹配，得到每个点的id，并将时差足够的点插入到地图中，对image进行赋值，判断是否为关键帧
     * 
     * @param frame_count 
     * @param image 
     * @param frame 
     * @param td 
     * @return true 
     * @return false 
     */
    bool addFeatureCheckParallax(int frame_count, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, feature_tracker::FeaturePtr, double td, Estimator* estimator_ptr);
    void debugShow();
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth(const VectorXd &x);
    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();
    list<FeaturePerId> feature;
    int last_track_num;
    vector<pair<int, FeaturePerId>> match_show; // old_id, 和其匹配的特征点
    vector<ImagePatchCorresponds> project_show; // 画图显示将3D点投影到当前帧进行光度Patch匹配的结果.
    
  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
    unsigned int id_factory = 0;
    cv::FlannBasedMatcher matcher_flann;
};

#endif