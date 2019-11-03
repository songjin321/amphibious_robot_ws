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
    ExposureController(int rows, int cols, int k, double p,  double step_):
    g_derivative_table(1, 256, CV_64FC1), step(step_)
    {
        // 初始化W
        int S = rows * cols;
        W.resize(S);
        double W_sum = 0;
        for(int i = 0; i < S; i++)
        {
            if ( i <= p*S)
                W[i] = pow(sin(M_PI_2 * i / (p * S)), k);
            else
                W[i] = pow(sin(M_PI_2 - M_PI_2 * (i - p*S) / (S - p * S)), k);
            W_sum += W[i];
        }
        for(int i = 0; i < S; i++)
        {
            W[i] = W[i]/W_sum;
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

    double get_exposure_time()
    {
        return exposure_time;
    }
     
    double get_exposure_gain()
    {
        return exposure_gain;
    }

    void set_g_derivative_table(const std::vector<double>& g)
    {
        // 
        if(g.size() != 256)
        {
            ROS_WARN("uncorrect g function size!");
            return;
        }
            
        
        std::vector<double> params(6, 0);
        // 拟合5次多项式，然后对g_derivative赋值

        params[0] = -4.824;
        params[1] = 0.2696;
        params[2] = -0.004746;
        params[3] = 0.000198;
        params[4] = -2.266e-6;
        params[5] = 1.517e-8;
    
        for( int i = 0; i < 256; ++i)
        {
            double g_derivative = 5*params[5]*i*i*i*i+
            4*params[4]*i*i*i+3*params[3]*i*i*+2*params[2]*i+params[1];
            std::cout << "g_derivative = " << g_derivative << std::endl;
            g_derivative_table.at<double>(0, i) = g_derivative;
        }
         
    }
    // 做好时间优化
    void exposure_adjust(cv::Mat &gray_image)
    {
        std::cerr << "begin exposure adjust" << std::endl;
        // 计算梯度
        std::cerr << "begin gradient calculate" << std::endl;
        cv::Mat blurred_image, gradient_image;
        cv::GaussianBlur(gray_image, blurred_image, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
        cv::Mat grad_x, grad_y;
        cv::Sobel(blurred_image, grad_x, CV_64FC1, 1, 0, 3, 1.0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(blurred_image, grad_y, CV_64FC1, 0, 1, 3, 1.0, 0, cv::BORDER_DEFAULT);
       
        cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, gradient_image);
        /*
        cv::imshow("gradient_image", gradient_image);
        cv::waitKey(1);
        */
        std::cerr << "end gradient calculate" << std::endl;

        // 对梯度进行排序
        std::cerr << "begin gradient sort" << std::endl;
        double *p;
        indexs.clear();
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
        std::sort(indexs.begin(), indexs.end(), [](const row_col_gradient &first, const row_col_gradient &second)
        {
            return first.gradient > second.gradient;
        });
        std::cout << "max gradient = " <<indexs[0].gradient << std::endl;
        std::cerr << "end gradient sort" << std::endl;

        // 计算相机响应函数的导数
        std::cerr << "begin g_derivative calculate" << std::endl;
        cv::Mat g_derivative, grad_g_x, grad_g_y;
        cv::Mat temp;
        cv::LUT(gray_image, g_derivative_table, temp);
        cv::divide(1 / exposure_time, temp, g_derivative);
        cv::Sobel(g_derivative, grad_g_x, CV_64FC1, 1, 0, 3, 1.0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(g_derivative, grad_g_y, CV_64FC1, 0, 1, 3, 1.0, 0, cv::BORDER_DEFAULT);
        std::cerr << "end g_derivative calculate" << std::endl;

        // 求和计算曝光修正值
        std::cerr << "begin sum_gradient calculate" << std::endl;
        double sum_gradient = 0;
        for (int i = 0; i < indexs.size(); i++)
        {
            row_col_gradient index = indexs[i];

            sum_gradient += W[i] * 2 * (grad_x.at<double>(index.row, index.col) * grad_g_x.at<double>(index.row, index.col) 
            + grad_y.at<double>(index.row, index.col) * grad_g_y.at<double>(index.row, index.col));
        }
        ROS_INFO("sum_gradient = %f", sum_gradient);
        exposure_time += step * sum_gradient;
        std::cerr << "begin sum_gradient calculate" << std::endl;
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
ExposureController exposure_controller(640, 480, 5, 0.5, 3);
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat colorImg = cv_ptr->image;
        if (colorImg.empty())
        {
            std::cerr << "empty input image" << std::endl;
            return;
        }
        cv::Mat grayImg;
        cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
        cv::imshow("gray Image", grayImg);
        cv::waitKey(1);
        double exposure_time = 5; // us
        double exposure_gain = 50;

        exposure_controller.set_exposure_time(exposure_time);
        exposure_controller.set_exposure_gain(exposure_gain);
        exposure_controller.exposure_adjust(grayImg);
        exposure_time = exposure_controller.get_exposure_time();
        exposure_gain = exposure_controller.get_exposure_gain();
        ROS_INFO("exposure time = %f, exposure gain = %f", exposure_time, exposure_gain);
        /*
        std_msgs::Int32MultiArray array;
        array.data.clear();
        array.data.push_back(exposure_time);
        array.data.push_back(exposure_gain);
        exposure_pub.publish(array);
        */
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exposure_control_optimization");
    ros::NodeHandle n;
    exposure_pub = n.advertise<std_msgs::Int32MultiArray>("/camera/set_exposure", 1);
    std::vector<double> g(256);
    exposure_controller.set_g_derivative_table(g);
    ros::Subscriber image_sub = n.subscribe("/camera/color/image_raw", 1, imageCallback);
    ros::spin();
    return 0;
}
