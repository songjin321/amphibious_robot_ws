#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include <std_msgs/Header.h>
#pragma once
using namespace std;
class Frame
{
public:
    typedef shared_ptr<Frame>   Ptr;
    std::vector<cv::KeyPoint>   keypoints;     // keypoints in current frame
    std::vector<cv::Point2f>    keysUn;     // keypoints in current frame
    cv::Mat                     descriptors;   // descriptor in current frame 
    std_msgs::Header            header;
};