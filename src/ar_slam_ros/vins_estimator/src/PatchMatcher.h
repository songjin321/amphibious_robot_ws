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
class PatchMatcher
{
public:
    PatchMatcher();
    ~PatchMatcher();

    void setCurFrame(cv::Mat& img_cur, Eigen::Vector3d& t_w_cur, Eigen::Quaterniond& q_w_cur, double timestamp);

    void setRefFrame(cv::Mat& img_cur, Eigen::Vector3d& t_w_cur, Eigen::Quaterniond& q_w_cur, double timestamp);

    bool projectMapPointToCurFrameAndCheck(Eigen::Vector3d& point);

    void setPoints(const std::vector<cv::Point2f>& tracked_points, const std::vector<cv::Point2f>& new_points)
    {
        tracked_points_ = tracked_points;
        new_points_ = new_points;
    };

    bool directMatch(Eigen::Vector3d point, Eigen::Vector2d& px_cur);
    double calDistance(cv::Point2f pt1, cv::Point2f pt2);
    Eigen::Vector3d currCameraToWorld(Eigen::Vector2d px_cam)
    {
        return frame_cur_->c2f(px_cam);
    }
    void createPatchFromPatchWithBorder();
private:
    // Objects declared here can be used by all tests
    vk::PinholeCamera *cam_;
    svo::Frame *frame_ref_;
    svo::Frame *frame_cur_;
    svo::Feature *ref_ftr_;
    std::vector<cv::Point2f> tracked_points_;
    std::vector<cv::Point2f> new_points_;
    static const int halfpatch_size_ = 4;
    static const int patch_size_ = 8;
    int check_threshold = 10; // 判断该点是否需要匹配的阈值
    int align_max_iter= 10;
    svo::Matcher matcher;
    uint8_t patch_[patch_size_*patch_size_] __attribute__ ((aligned (16)));
    uint8_t patch_with_border_[(patch_size_+2)*(patch_size_+2)] __attribute__ ((aligned (16)));
};