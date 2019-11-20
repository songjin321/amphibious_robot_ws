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

void PatchMatcher::setCurFrame(cv::Mat &img_cur, Eigen::Vector3d &t_w_cur, Eigen::Quaterniond &q_w_cur, double timestamp)
{
  if (img_cur.empty())
    std::cerr << "empty current image for matching" << std::endl;
  cv::Mat img_cur_gray;
  cv::cvtColor(img_cur, img_cur_gray, cv::COLOR_BGR2GRAY);
  frame_cur_ = new svo::Frame(cam_, img_cur_gray, timestamp);
  // frame_cur_->T_f_w_ = Sophus::SE3(q_w_cur, t_w_cur).inverse();
}

void PatchMatcher::setRefFrame(cv::Mat &img_ref, Eigen::Vector3d &t_w_ref, Eigen::Quaterniond &q_w_ref, double timestamp)
{
  if (img_ref.empty())
    std::cerr << "empty reference image for matching" << std::endl;
  cv::Mat img_ref_gray;
  cv::cvtColor(img_ref, img_ref_gray, cv::COLOR_BGR2GRAY);
  frame_ref_ = new svo::Frame(cam_, img_ref_gray, timestamp);
  frame_ref_->T_f_w_ = Sophus::SE3(q_w_ref, t_w_ref).inverse();
}

bool PatchMatcher::projectMapPointToCurFrameAndCheck(Eigen::Vector3d &point)
{
  Eigen::Vector2d px(frame_cur_->w2c(point));
  if (frame_cur_->cam_->isInFrame(px.cast<int>(), patch_size_)) // 8px is the patch size in the matcher
  {
    for (int i = 0; i < tracked_points_.size(); i++)
    {
      if (calDistance(tracked_points_[i], cv::Point2f(px(0), px(1))) < check_threshold)
        return false;
    }
    for (int i = 0; i < new_points_.size(); i++)
    {
      if (calDistance(new_points_[i], cv::Point2f(px(0), px(1))) < check_threshold)
        return true;
    }
    return false; ///< no texture region
  }
  return false;
}

bool PatchMatcher::directMatch(Eigen::Vector3d& point, Eigen::Vector2d &px_cur)
{
  // 分别投影到ref和cur帧上
  px_cur = frame_cur_->w2c(point);
  ref_ftr_ = new svo::Feature(frame_ref_, frame_ref_->w2c(point), 0);

  // 计算ref和cur的变化
  Sophus::SE3 T_cur_ref(frame_cur_->T_f_w_ * frame_ref_->T_f_w_.inverse());

  // compute reference patch
  Eigen::Matrix2d A_cur_ref;
  svo::warp::getWarpMatrixAffine(
      *ref_ftr_->frame->cam_, *frame_cur_->cam_, ref_ftr_->px, ref_ftr_->f,
      (ref_ftr_->frame->pos() - point).norm(), T_cur_ref, ref_ftr_->level, A_cur_ref);
  std::cout << "A_cur_ref = " << A_cur_ref << std::endl;
  int search_level_ = svo::warp::getBestSearchLevel(A_cur_ref, svo::Config::nPyrLevels() - 1);
  std::cout << "level_cur = " << search_level_ << std::endl;
  svo::warp::warpAffine(
      A_cur_ref, ref_ftr_->frame->img_pyr_[ref_ftr_->level], ref_ftr_->px,
      ref_ftr_->level, search_level_, halfpatch_size_ + 1, patch_with_border_);
  createPatchFromPatchWithBorder();

  // px_cur should be set
  Eigen::Vector2d px_scaled(px_cur / (1 << search_level_));

  bool success = false;
  success = svo::feature_alignment::align2D(
      frame_cur_->img_pyr_[search_level_], patch_with_border_, patch_,
      align_max_iter, px_scaled);
  px_cur = px_scaled * (1 << search_level_);
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