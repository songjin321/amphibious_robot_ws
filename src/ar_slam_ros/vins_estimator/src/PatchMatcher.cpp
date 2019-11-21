/**
 * @file PatchMatcher.cpp
 * @author Song Jin (songjinxs@163.com)
 * @brief patch matcher
 * @version 0.1
 * @date 2019-11-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "PatchMatcher.h"
#include <svo/feature_alignment.h>
#include "opencv2/opencv.hpp"
PatchMatcher::PatchMatcher()
{
  cam_ = new vk::PinholeCamera(640, 480, 606.8794, 605.7115, 318.1654, 242.6335, 0.1319, -0.1987);
}

bool PatchMatcher::setCurFrame(std::vector<cv::Mat>& img_cur_pyr, Eigen::Vector3d &t_w_cur, Eigen::Quaterniond &q_w_cur, double timestamp)
{
  if (img_cur_pyr.empty())
  {
    std::cerr << "empty current image for matching" << std::endl;
    return false;
  }

  frame_cur_ = new svo::Frame(cam_, img_cur_pyr, timestamp);
  frame_cur_->T_f_w_ = Sophus::SE3(q_w_cur, t_w_cur).inverse();
  return true;
}

bool PatchMatcher::setRefFrameAndFeature(std::vector<cv::Mat>& img_ref_pyr, Eigen::Vector3d& t_w_ref, 
Eigen::Quaterniond& q_w_ref, double timestamp, Eigen::Vector2d& ref_px)
{
  if (img_ref_pyr.empty())
  {
    std::cerr << "empty reference image for matching" << std::endl;
    return false;
  }
  frame_ref_ = new svo::Frame(cam_, img_ref_pyr, timestamp);
  frame_ref_->T_f_w_ = Sophus::SE3(q_w_ref, t_w_ref).inverse();
  px_ref_ = ref_px;
  ref_ftr_ = new svo::Feature(frame_ref_, px_ref_, 0);
  return true;
}

bool PatchMatcher::projectMapPointToCurFrameAndCheck(Eigen::Vector3d &point)
{
  px_cur_ = frame_cur_->w2c(point);
  if (frame_cur_->cam_->isInFrame(px_cur_.cast<int>(), patch_size_))
  {
    for (int i = 0; i < tracked_points_.size(); i++)
    {
      if (calDistance(tracked_points_[i], cv::Point2f(px_cur_(0), px_cur_(1))) < check_threshold)
        return false;
    }
    for (int i = 0; i < new_points_.size(); i++)
    {
      if (calDistance(new_points_[i], cv::Point2f(px_cur_(0), px_cur_(1))) < 3*check_threshold)
        return true;
    }
  }
  return false; ///< no texture region
}

bool PatchMatcher::directMatch(Eigen::Vector3d& point, Eigen::Vector2d& px_cur_final, ImagePatchCorresponds& image_patch_cors)
{
  // svo
  /*
  // 计算ref和cur的变化
  Sophus::SE3 T_cur_ref(frame_cur_->T_f_w_ * frame_ref_->T_f_w_.inverse());

  // compute reference patch
  Eigen::Matrix2d A_cur_ref;
  svo::warp::getWarpMatrixAffine(
      *ref_ftr_->frame->cam_, *frame_cur_->cam_, ref_ftr_->px, ref_ftr_->f,
      (ref_ftr_->frame->pos() - point).norm(), T_cur_ref, ref_ftr_->level, A_cur_ref);
  int search_level_ = svo::warp::getBestSearchLevel(A_cur_ref, svo::Config::nPyrLevels() - 1);
  svo::warp::warpAffine(
      A_cur_ref, ref_ftr_->frame->img_pyr_[ref_ftr_->level], ref_ftr_->px,
      ref_ftr_->level, search_level_, halfpatch_size_ + 1, patch_with_border_);
  createPatchFromPatchWithBorder();

  // px_scaled should be set
  Eigen::Vector2d px_scaled(px_cur_ / (1 << search_level_));

  bool success = false;
  success = svo::feature_alignment::align2D(
      frame_cur_->img_pyr_[search_level_], patch_with_border_, patch_,
      align_max_iter, px_scaled);
  px_cur_final = px_scaled * (1 << search_level_);
  */

  // opencv
  std::vector<uchar> status;
  std::vector<float> err;
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
  std::vector<cv::Point2f> p0,p1;
  p0.emplace_back(px_ref_.x(), px_ref_.y());
  p1.emplace_back(px_cur_.x(), px_cur_.y());
  cv::calcOpticalFlowPyrLK(
    frame_ref_->img_pyr_, 
    frame_cur_->img_pyr_, 
    p0, 
    p1, 
    status, err, cv::Size(21, 21), 3, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
  bool success = false;
  if (status[0] == 1)
  {
    px_cur_final(0) = p1[0].x;
    px_cur_final(1) = p1[0].y;
    success = true;
  }
  // 对image_patch_cors进行赋值
  image_patch_cors.cur_px_init = cv::Point2f(px_cur_.x(), px_cur_.y());
  image_patch_cors.ref_px = cv::Point2f(px_ref_.x(), px_ref_.y());
  image_patch_cors.cur_px_final = cv::Point2f(px_cur_final.x(), px_cur_final.y());
  image_patch_cors.ref_patch_warp = cv::Mat(halfpatch_size_, halfpatch_size_, CV_8U, patch_);
  image_patch_cors.match_success = success;
  // image_patch_cors.cur_patch_init = 
  return success;
}

void PatchMatcher::createPatchFromPatchWithBorder()
{
  uint8_t *ref_patch_ptr = patch_;
  for (int y = 1; y < patch_size_ + 1; ++y, ref_patch_ptr += patch_size_)
  {
    uint8_t *ref_patch_border_ptr = patch_with_border_ + y * (patch_size_ + 2) + 1;
    for (int x = 0; x < patch_size_; ++x)
      ref_patch_ptr[x] = ref_patch_border_ptr[x];
  }
}

double PatchMatcher::calDistance(cv::Point2f pt1, cv::Point2f pt2)
{
  return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

PatchMatcher::~PatchMatcher()
{
}