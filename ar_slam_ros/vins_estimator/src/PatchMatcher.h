/**
 * @file PatchMatcher.h
 * @author Song Jin (songjinxs@163.com)
 * @brief patch matcher
 * @version 0.1
 * @date 2019-11-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <algorithm>
#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>
#include <svo/matcher.h>
#include <svo/frame.h>
#include <svo/config.h>
#include <svo/feature.h>
#include "feature_manager.h"
class PatchMatcher
{
public:
    PatchMatcher();
    ~PatchMatcher();

    bool setCurFrame(std::vector<cv::Mat>& img_cur_pyr, Eigen::Vector3d& t_w_cur, Eigen::Quaterniond& q_w_cur, double timestamp);

    bool setRefFrameAndFeature(std::vector<cv::Mat>& img_ref_pyr, Eigen::Vector3d& t_w_ref, 
    Eigen::Quaterniond& q_w_ref, double timestamp, Eigen::Vector2d& ref_px);

    bool projectMapPointToCurFrameAndCheck(Eigen::Vector3d& point);

    bool directMatch(Eigen::Vector3d& point, Eigen::Vector2d& px_cur_final, ImagePatchCorresponds& image_patch_cors);

    double calDistance(cv::Point2f pt1, cv::Point2f pt2);

    void createPatchFromPatchWithBorder();

    // Objects declared here can be used by all tests
    vk::PinholeCamera *cam_;
    svo::Frame *frame_ref_;
    svo::Frame *frame_cur_;
    svo::Feature *ref_ftr_;
    std::vector<cv::Point2f> tracked_points_;
    std::vector<cv::Point2f> new_points_;
    static const int halfpatch_size_ = 4;
    static const int patch_size_ = 8;
    int check_threshold = 5; // 判断该点是否需要匹配的阈值
    int align_max_iter= 20;
    svo::Matcher matcher;
    uint8_t patch_[patch_size_*patch_size_] __attribute__ ((aligned (16)));
    uint8_t patch_with_border_[(patch_size_+2)*(patch_size_+2)] __attribute__ ((aligned (16)));
    Eigen::Vector2d px_cur_;
    Eigen::Vector2d px_ref_; 
};