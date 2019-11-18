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
    // void testEpipolarSearchFullImg();
    // void testWarpAffine();

private:
    // Objects declared here can be used by all tests
    vk::PinholeCamera *cam_;
    svo::Frame *frame_ref_;
    svo::Frame *frame_cur_;
    svo::Feature *ref_ftr_;
    cv::Mat depth_ref_;
};