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
#include <algorithm>
#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>
#include <vikit/blender_utils.h>
#include <svo/matcher.h>
#include <svo/frame.h>
#include <svo/config.h>
#include <svo/feature.h>

namespace {

/// Matcher Test-Fixture
class MatcherTest {
 public:
  MatcherTest()
  {
    cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);

    // load images
    std::string dataset_dir("/sin2_tex2_h1_v8_d");
    std::string img_name(dataset_dir+"/img/frame_000002_0.png");
    printf("Loading image '%s'\n", img_name.c_str());
    cv::Mat img_ref(cv::imread(img_name, 0));
    img_name = std::string(dataset_dir+"/img/frame_000006_0.png");
    printf("Loading image '%s'\n", img_name.c_str());
    cv::Mat img_cur(cv::imread(img_name, 0));
    assert(!img_ref.empty() && !img_cur.empty());

    // create frame
    frame_ref_ = new svo::Frame(cam_, img_ref, 1.0);
    frame_cur_ = new svo::Frame(cam_, img_cur, 2.0);
    ref_ftr_ = new svo::Feature(frame_ref_, Eigen::Vector2d(300, 260), 0);

    // set poses
    Eigen::Vector3d t_w_ref(0.1131, 0.1131, 2.0000);
    Eigen::Vector3d t_w_cur(0.5673, 0.5641, 2.0000);
    Eigen::Quaterniond q_w_ref(0.0, 0.8227, 0.2149, 0.0);
    Eigen::Quaterniond q_w_cur(0.0, 0.8235, 0.2130, 0.0);
    frame_ref_->T_f_w_ = Sophus::SE3(q_w_ref, t_w_ref).inverse();
    frame_cur_->T_f_w_ = Sophus::SE3(q_w_cur, t_w_cur).inverse();

    // load ground-truth depth
    vk::blender_utils::loadBlenderDepthmap(dataset_dir+"/depth/frame_000002_0.depth", *cam_, depth_ref_);
  }

  virtual ~MatcherTest()
  {
    delete cam_;
    delete frame_ref_;
    delete frame_cur_;
    delete ref_ftr_;
  }

  // Individual tests
  void testEpipolarSearchFullImg();
  void testWarpAffine();

  // Objects declared here can be used by all tests
  vk::PinholeCamera* cam_;
  svo::Frame* frame_ref_;
  svo::Frame* frame_cur_;
  svo::Feature* ref_ftr_;
  cv::Mat depth_ref_;
};

void MatcherTest::testEpipolarSearchFullImg()
{
  svo::Matcher matcher;
  cv::Mat depthmap(frame_ref_->cam_->height(), frame_ref_->cam_->width(), CV_32FC1, cv::Scalar(0));
  cv::Mat error_map(depthmap.rows, depthmap.cols, CV_32FC1, cv::Scalar(0));
  cv::Mat mask(depthmap.rows, depthmap.cols, CV_8UC1, cv::Scalar(1));
  double depth_range = 0.8;
  size_t n_converged = 0;
  double sum_error = 0;
  std::vector<double> errors; errors.reserve(depthmap.cols*depthmap.rows);
  for(int y=4; y<depthmap.rows-4; ++y)
  {
    for(int x=4; x<depthmap.cols-4; ++x)
    {
      svo::Feature ftr(frame_ref_, Eigen::Vector2d(x,y), 0);
      double depth_gt = depth_ref_.at<float>(y, x);
      double depth_estimate;
      int res = matcher.findEpipolarMatchDirect(
          *frame_ref_, *frame_cur_, ftr, depth_gt, fmax(depth_gt-depth_range,0.0),
          depth_gt+depth_range, depth_estimate);
      if(res)
      {
        depthmap.at<float>(y,x) = depth_estimate;
        error_map.at<float>(y,x) = fabsf(depth_estimate-depth_gt);
        ++n_converged;
        sum_error += error_map.at<float>(y,x);
        errors.push_back(error_map.at<float>(y,x));
      }
      else
        mask.at<uint8_t>(y,x) = 0;
    }
  }

  // compute mean, median and variance of error in converged area
  printf("n converged:  \t %zu mm (ref: 216114)\n", n_converged);
  printf("mean error:   \t %f mm (ref: 0.410084)\n", sum_error*100/n_converged);
  std::vector<double>::iterator it = errors.begin()+0.5*errors.size();
  std::nth_element(errors.begin(), it, errors.end());
  printf("50-percentile: \t %f mm (ref: 0.083203)\n", *it*100);
  it = errors.begin()+0.8*errors.size();
  std::nth_element(errors.begin(), it, errors.end());
  printf("80-percentile: \t %f mm (ref: 0.161824)\n", *it*100);
  it = errors.begin()+0.95*errors.size();
  std::nth_element(errors.begin(), it, errors.end());
  printf("95-percentile: \t %f mm (ref: 0.263539)\n", *it*100);
}

void MatcherTest::testWarpAffine()
{
  const int halfpatch_size = 4;
  const int patch_size=5;
  svo::Matcher matcher;
  double depth = depth_ref_.at<float>(260, 300);;

  Sophus::SE3 T_cur_ref(frame_cur_->T_f_w_*frame_ref_->T_f_w_.inverse());
  Eigen::Vector2d px_cur(frame_cur_->cam_->world2cam(T_cur_ref*(ref_ftr_->f*depth)));

  // compute reference patch
  cv::Mat ref_patch(matcher.halfpatch_size_+1, matcher.halfpatch_size_+1,
                    CV_8U, matcher.patch_with_border_);

  Eigen::Matrix2d A_cur_ref;
  svo::warp::getWarpMatrixAffine(
      *cam_, *cam_, ref_ftr_->px, ref_ftr_->f, 1.0, T_cur_ref, ref_ftr_->level, A_cur_ref);
  std::cout << "A_cur_ref = " << A_cur_ref << std::endl;
  int level_cur = svo::warp::getBestSearchLevel(A_cur_ref, svo::Config::nPyrLevels()-1);
  std::cout << "level_cur = " << level_cur << std::endl;
  svo::warp::warpAffine(
      A_cur_ref, ref_ftr_->frame->img_pyr_[ref_ftr_->level], ref_ftr_->px,
      ref_ftr_->level, level_cur, matcher.halfpatch_size_+1, matcher.patch_with_border_);

  // copy current patch
  Eigen::Vector2i pxi(px_cur[0]/(1<<level_cur)+0.5, px_cur[1]/(1<<level_cur)+0.5);
  cv::Mat cur_patch(patch_size, patch_size, CV_8U);
  frame_cur_->img_pyr_.at(level_cur)(cv::Range(pxi[1]-halfpatch_size,
                                               pxi[1]+halfpatch_size),
                                     cv::Range(pxi[0]-halfpatch_size,
                                               pxi[0]+halfpatch_size)).copyTo(cur_patch);

  // display results
  cv::circle(frame_cur_->img_pyr_[level_cur],
             cv::Point2f(pxi[0], pxi[1]),
             patch_size, cv::Scalar(255), 2);
  cv::circle(frame_ref_->img_pyr_[ref_ftr_->level],
              cv::Point2f(ref_ftr_->px[0]/(1<<ref_ftr_->level), ref_ftr_->px[1]/(1<<ref_ftr_->level)),
              patch_size, cv::Scalar(255), 2);
  cv::imshow("cur_patch", cur_patch);
  cv::imshow("ref_patch", ref_patch);
  cv::imshow("ref_img", frame_ref_->img_pyr_[ref_ftr_->level]);
  cv::imshow("cur_img", frame_cur_->img_pyr_[level_cur]);
  cv::waitKey(0);
}

}  // namespace

int main(int argc, char** argv)
{
  MatcherTest test;
  // test.testEpipolarSearchFullImg();
  test.testWarpAffine();
  return 0;
}