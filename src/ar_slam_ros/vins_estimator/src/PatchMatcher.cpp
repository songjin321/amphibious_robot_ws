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
PatchMatcher::PatchMatcher()
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
}

PatchMatcher::~PatchMatcher()
{
  
}