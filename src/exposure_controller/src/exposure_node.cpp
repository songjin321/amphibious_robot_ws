#include <ros/ros.h>  
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "std_msgs/Int32MultiArray.h" 
#include "std_msgs/String.h"

#define THREAD_NUMS 4
#define GAMMA_NUMS 4
using namespace std;
using namespace cv;
uchar gamma_0[256];
uchar gamma_1[256];
uchar gamma_2[256];
uchar gamma_3[256];
int flag = 0;
Mat colorImg;
vector<Mat>LUT_GAMMA;
ros::Publisher chatter_pub;
ros::Subscriber sub;
struct paramThread
{
    int w;
    int h;
    uchar * data;
    double* quality;
    Mat lut_index;
};
void GammaCorrection(Mat* src, Mat lut_index);
double calculate_m(Mat color_image, double lamda, double delta);
//static vector<double>gamma_parament = { 0.5 ,0.8, 1.1, 1.3 };
static vector<double>gamma_parament;
int exposure_time = 625;
int exposure_gain = 64;
/********************************************************
*   @brief       : ¶àÏß³Ì´¦Àíº¯Êý
*   @param  args : ¶àÏß³Ì´«ÈëµÄ²ÎÊý
*   @return      : void
********************************************************/
void * threadProcess(void* args) {
    pthread_t myid = pthread_self();
    paramThread *para = (paramThread *)args;
    int w = para->w;
    int h = para->h;
    Mat lut_vectors = para->lut_index;
    cv::Mat image(h, w, CV_8UC1, (uchar *)para->data);
    GammaCorrection(&image, lut_vectors);
    *(para->quality) = calculate_m(image, 100, 0.3);
    //cout << *(para->quality) << endl;
    pthread_exit(NULL);
    return NULL;
}

void * threadProcess_base(void* args) {
    pthread_t myid = pthread_self();
    paramThread *para = (paramThread *)args;
    int w = para->w;
    int h = para->h;
    cv::Mat image(h, w, CV_8UC1, (uchar *)para->data);
    *(para->quality) = calculate_m(image, 100, 0.3);
    //cout << *(para->quality) << endl;
    pthread_exit(NULL);
    return NULL;
}
/********************************************************
*   @brief       : ÊµÏÖÍ¼Ïñ·Ö¸î£¬
*   @param  num  :  ·Ö¸î¸öÊý
*   @param  type : 0£º´¹Ö±·Ö¸î(ÍÆ¼ö)£¬1£ºË®Æ½·Ö¸î£¨²»ÍÆ¼ö£©
*   @return      : vector<cv::Mat>
*   PS£ºÊ¹ÓÃË®Æ½·Ö¸îÊ±£¨type=1£©£¬´¦ÀíÍêºó±ØÐëµ÷ÓÃcatImage½øÐÐÆ´½Ó£¬
*   Ê¹ÓÃ´¹Ö±·Ö¸îÊ±£¨type=0£©£¬¿ÉÒÔ²»½øÐÐcatImage£¬ÒòÎªÊÇ¶ÔÔ­Í¼½øÐÐ²Ù×÷µÄ
********************************************************/

vector<cv::Mat> splitImage(cv::Mat image, int num, int type) {
    int rows = image.rows;
    int cols = image.cols;
    vector<cv::Mat> v;
    if (type == 0) {//´¹Ö±·Ö¸î
        for (size_t i = 0; i < num; i++) {
            int star = rows / num * i;
            int end = rows / num * (i + 1);
            if (i == num - 1) {
                end = rows;
            }
            //cv::Mat b = image.rowRange(star, end);
            v.push_back(image.rowRange(star, end));
        }
    }
    else if (type == 1) {//Ë®Æ½·Ö¸î
        for (size_t i = 0; i < num; i++) {
            int star = cols / num * i;
            int end = cols / num * (i + 1);
            if (i == num - 1) {
                end = cols;
            }
            //cv::Mat b = image.colRange(star, end);
            /*½â¾öË®Æ½·Ö¸îµÄBug:±ØÐëclone()*/
            v.push_back(image.colRange(star, end).clone());
        }
    }
    return  v;
}

/********************************************************
*   @brief       : ÊµÏÖÍ¼ÏñÆ´½Ó£¬
*   @param  v    :
*   @param  type : 0£º´¹Ö±Æ´½Ó£¬1£ºË®Æ½Æ´½Ó
*   @return      : Mat
********************************************************/
cv::Mat catImage(vector<cv::Mat> v, int type) {
    cv::Mat dest = v.at(0);
    for (size_t i = 1; i < v.size(); i++)
    {
        if (type == 0)//´¹Ö±Æ´½Ó
        {
            cv::vconcat(dest, v.at(i), dest);
        }
        else if (type == 1)//Ë®Æ½Æ´½Ó
        {
            cv::hconcat(dest, v.at(i), dest);
        }
    }
    return dest;
}
void gamma_vector_creat(uchar* gamma_i, double gamma)
{
    double anti_gamma = 1 / gamma;
    for (int i = 0; i <= 255; i++)
    {
        *(gamma_i + i) = (pow(double((i + 0.5) / 256), anti_gamma) * 256 - 0.5);
    }
}

void GammaCorrection(Mat* src_address, Mat LUT_index)
{
    // build look up table  
    Mat src = *src_address;
    Mat dst;
    cv::LUT(src, LUT_index, dst);
    *src_address = dst;
}
double muti_thread_base(Mat gray_image)
{
    int type = 0;
    vector<cv::Mat> v = splitImage(gray_image, THREAD_NUMS, type);
    vector<double> q(THREAD_NUMS);
    paramThread args_base[THREAD_NUMS];
    pthread_t pt_base[THREAD_NUMS]; //´´½¨THREAD_NUMS¸ö×ÓÏß³Ì
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        args_base[i].h = v.at(i).rows;
        args_base[i].w = v.at(i).cols;
        args_base[i].data = v.at(i).data;
        args_base[i].quality = &q[i];
        pthread_create(&pt_base[i], NULL, &threadProcess_base, (void *)(&args_base[i]));
    }
    /*µÈ´ýÈ«²¿×ÓÏß³Ì´¦ÀíÍê±Ï*/
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        pthread_join(pt_base[i], NULL);
    }
    double r = 0;
    for (int i = 0; i < THREAD_NUMS; i++)
        r += q[i];
    return r;
}

double muti_thread(Mat gray_image, Mat lut_index)
{
    int type = 0;
    Mat temp_image = gray_image.clone();
    vector<cv::Mat> v = splitImage(temp_image, THREAD_NUMS, type);
    vector<double> q(THREAD_NUMS);
    paramThread args[THREAD_NUMS];
    pthread_t pt[THREAD_NUMS];  //´´½¨THREAD_NUMS¸ö×ÓÏß³Ì
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        args[i].h = v.at(i).rows;
        args[i].w = v.at(i).cols;
        args[i].data = v.at(i).data;
        args[i].quality = &q[i];
        args[i].lut_index = lut_index;
        pthread_create(&pt[i], NULL, &threadProcess, (void *)(&args[i]));
    }
    /*µÈ´ýÈ«²¿×ÓÏß³Ì´¦ÀíÍê±Ï*/
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        pthread_join(pt[i], NULL);
    }
    double r = 0;
    for (int i = 0; i < THREAD_NUMS; i++)
        r += q[i];
    //Mat t = catImage(v, 0);
    //imshow("1", t);
    //v.clear();
    return r;
}


double calculate_m(Mat gray_image, double lamda, double delta)
{

    Mat image = gray_image.clone();
    std::vector<KeyPoint> detectKeyPoint;
    // construction of the fast feature detector object
    Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
    // feature point detection
    fast->detect(image, detectKeyPoint);
    return detectKeyPoint.size();
}

double get_suitable_gamma(Mat gray_image, vector<Mat>LUT_GAMMA)
{
    double value = 0;
    int num = -1;
    //double value_0 = muti_thread_base(gray_image);
    double value_0 = muti_thread_base(gray_image);
    //cout << value_0<<" " ;

    double max_value = value_0;
    for (int i = 0; i < GAMMA_NUMS; i++)
    {
        Mat temp_image = gray_image.clone();
        value = muti_thread(gray_image, LUT_GAMMA[i]);
        //cout << value <<" ";
        if (value > max_value)
        {
            num = i;
            max_value = value;
        }
    }
    //cout << endl;
    if (value < 30)
    {
        return -1;
    }
    if (max_value <= 1.2 * value_0)
    {
        return 1;
    }
    else
        return gamma_parament[num];
}
void exposure_adjust(Mat color_image, int* expose_now, int* exposure_gain, vector<Mat>LUT_GAMMA)
{
    Mat gray_image;
    cvtColor(color_image, gray_image, CV_BGR2GRAY);
    double gamma = get_suitable_gamma(gray_image, LUT_GAMMA);

    if (gamma == -1)
    {
        flag = flag + 1;
        if (flag > 50)
        {
            *expose_now = 1250;
            *exposure_gain = 64;
            flag = 0;
        }
    }
    else
    {
        double alfa;
        if (gamma < 1)
            alfa = 1;
        else
            alfa = 1;
        ////ÏßÐÔ¿ØÖÆ
        *expose_now = *expose_now * (1 + alfa * 0.3*(gamma - 1));
          //cout << 1 + alfa * 0.2*(gamma - 1) << endl;
        *exposure_gain = *exposure_gain * (1 + alfa * 0.3*(gamma - 1));

        ////·ÇÏßÐÔ¿ØÖÆ
        //double k_p = 1;
       // double d = 0.2;
        //double R_value = d * tan((gamma - 2) * atan(1 / d) - atan(1 / d)) + 1;
        //*expose_now = *expose_now * (1 + alfa * k_p *(R_value - 1));
        //*exposure_gain = *exposure_gain *  (1 + alfa * k_p *(R_value - 1));

        if (*expose_now > 1250)
            *expose_now = 1250;
        if (*expose_now < 30)
            *expose_now = 30;
        if (*exposure_gain > 127)
            *exposure_gain = 127;
        else if (*exposure_gain < 20)
            *exposure_gain = 20;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    colorImg = cv_ptr->image;  
    //imshow("view",colorImg);  
    //cv::waitKey(1);
    exposure_adjust(colorImg, &exposure_time, &exposure_gain, LUT_GAMMA);
    ROS_INFO("exposure time = %d, exposure gain = %d", exposure_time, exposure_gain);
    std_msgs::Int32MultiArray array;
    array.data.clear();
    array.data.push_back(exposure_time);
    array.data.push_back(exposure_gain);
    chatter_pub.publish(array);
    //ROS_INFO("I publish");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
    gamma_parament.push_back(0.5);
    gamma_parament.push_back(0.8);
    gamma_parament.push_back(1.5);
    gamma_parament.push_back(2);
    gamma_vector_creat(gamma_0, gamma_parament[0]);
    gamma_vector_creat(gamma_1, gamma_parament[1]);
    gamma_vector_creat(gamma_2, gamma_parament[2]);
    gamma_vector_creat(gamma_3, gamma_parament[3]);
    Mat lut0(1, 256, CV_8UC1, gamma_0);
    Mat lut1(1, 256, CV_8UC1, gamma_1);
    Mat lut2(1, 256, CV_8UC1, gamma_2);
    Mat lut3(1, 256, CV_8UC1, gamma_3);
    LUT_GAMMA.push_back(lut0);
    LUT_GAMMA.push_back(lut1);
    LUT_GAMMA.push_back(lut2);
    LUT_GAMMA.push_back(lut3);
    ros::init(argc, argv, "exposure_control");
    ros::NodeHandle n;
    //image_transport::ImageTransport it(n);

    chatter_pub = n.advertise<std_msgs::Int32MultiArray>("/camera/set_exposure", 1);
    sub = n.subscribe("/camera/color/image_raw", 1, imageCallback); 
    ros::Rate loop_rate(10);
    ros::spin();  

    //rospy.Subscriber("camera", Int32MultiArray, callback)    
    //image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1000, imageCallback);
    //std_msgs::Int32MultiArray array;
    //array.data.clear();
    //array.data.push_back(1);
    //array.data.push_back(1);
    //chatter_pub.publish(array);
   // ROS_INFO("I published something!");
    //ros::spin();
    //ros::spin();
    
    return 0 ;
}
