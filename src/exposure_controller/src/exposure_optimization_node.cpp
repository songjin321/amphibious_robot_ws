#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

struct row_col_gradient
{
    int row;
    int col;
    double gradient;
};

class ExposureController
{
public:
    ExposureController(int rows, int cols, int k, double p):
    g_derivative_table(1, 256, CV_32FC1)
    {
        // 初始化W
        int S = rows * cols;
        W.resize(S);
        for(int i = 0; i < S; i++)
        {
            if ( i <= p*S)
                W[i] = pow(sin(M_PI_2 * i / (p * S)), k);
            else
                W[i] = pow(sin(M_PI_2 - M_PI_2 * (i - p*S) / (S - p * S)), k);
        }
    }

    void set_exposure_time(double exposure_time_)
    {
        exposure_time = exposure_time_;
    }

    void set_exposure_gain(double exposure_gain_)
    {
        exposure_gain = exposure_gain_;
    }

    void set_g_derivative_table()
    {
        double* p = g_derivative_table.ptr();
        for( int i = 0; i < 256; ++i)
            p[i] = table[i];
    }
    // 做好时间优化
    void exposure_adjust(cv::Mat &gray_image)
    {
        // 计算梯度
        cv::Mat blurred_image, gradient_image;
        cv::GaussianBlur(gray_image, blurred_image, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
        cv::Mat grad_x, grad_y;
        cv::Sobel(blurred_image, grad_x, CV_16S, 1, 0, 3, 1.0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(blurred_image, grad_y, CV_16S, 0, 1, 3, 1.0, 0, cv::BORDER_DEFAULT);
        cv::convertScaleAbs(grad_x, grad_x);
        cv::convertScaleAbs(grad_y, grad_y);
        cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, gradient_image);

        // 对梯度进行排序
        double *p;
        for (int i = 0; i < gradient_image.rows; i++)
        {
            p = gradient_image.ptr<double>(i);
            for (int j = 0; j < gradient_image.cols; j++)
            {
                row_col_gradient temp;
                temp.row = i;
                temp.col = j;
                temp.gradient = p[j];
                indexs.push_back(temp);
            }
        }
        std::sort(indexs.begin(), indexs.end(), [](const row_col_gradient &first, const row_col_gradient &second) {
            first.gradient > second.gradient;
        });

        // 计算相机响应函数的导数
        cv::Mat g_derivative, grad_g_x, grad_g_y;
        cv::Mat temp;
        cv::LUT(gray_image, g_derivative_table, temp);
        cv::divide(1 / exposure_time, temp, g_derivative);
        cv::Sobel(g_derivative, grad_g_x, CV_16S, 1, 0, 3, 1.0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(g_derivative, grad_g_y, CV_16S, 0, 1, 3, 1.0, 0, cv::BORDER_DEFAULT);

        // 求和计算曝光修正值
        double sum_gradient = 0;
        for (int i = 0; i < indexs.size(); i++)
        {
            row_col_gradient index = indexs[i];
            sum_gradient += W[i] * 2 * (grad_x.at<double>(index.row, index.col) * grad_g_x.at<double>(index.row, index.col) 
            + grad_y.at<double>(index.row, index.col) * grad_g_y.at<double>(index.row, index.col));
        }
        exposure_time += step * sum_gradient;
    }


private:
    double exposure_time; // 相机的曝光时间
    double exposure_gain; // 相机的曝光增益
    int rows; // 输入图像的行数
    int cols; // 输入图像的列数
    double step; // 下降步长
    cv::Mat g_derivative_table; // 反响应函数的导数
    std::vector<double> W; // 梯度的权重
    std::vector<row_col_gradient> indexs; //用于排序
};

ros::Publisher exposure_pub;
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        colorImg = cv_ptr->image;
        exposure_adjust(colorImg);
        ROS_INFO("exposure time = %d, exposure gain = %d", exposure_time, exposure_gain);
        std_msgs::Int32MultiArray array;
        array.data.clear();
        array.data.push_back(exposure_time);
        array.data.push_back(exposure_gain);
        exposure_pub.publish(array);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exposure_control");
    ros::NodeHandle n;
    exposure_pub = n.advertise<std_msgs::Int32MultiArray>("/camera/set_exposure", 1);
    ros::Subscriber image_sub = n.subscribe("/camera/color/image_raw", 1, imageCallback);
    ros::spin();
    return 0;
}
