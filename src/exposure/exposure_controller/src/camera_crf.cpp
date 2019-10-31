// pthreadDemo.cpp : �������̨Ӧ�ó������ڵ㡣
// adjust_exposure_value 
// �Ҷ�ͼ����gammaУ�� ����gain���� ���۱�׼��Ϊ�ݶȺ� ���
#include <ros/ros.h>  
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "std_msgs/Int32MultiArray.h" 
#include "std_msgs/String.h"
#include <math.h>  
#include <Eigen/Core>
#include <signal.h>
#include <opencv2/core/eigen.hpp> 

#define THREAD_NUMS 8
#define GAMMA_NUMS 4
using namespace std;
using namespace cv;
using namespace Eigen;
static uchar t[256];
static float g[256];
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
};
double calculate_m(Mat color_image, double lamda, double delta);
//static vector<double>gamma_parament = { 0.5 ,0.8, 1.1, 1.3 };
static vector<double>gamma_parament;
double exposure_time = 625;
double exposure_gain = 64;

static int get_near_index(float value)
{
    int ret_index;                        /* ½á¹ûË÷Òý */
    int mid_index;                        /* ÖÐÎ»ÓÎ±ê */
    int left_index;                       /* ×óÎ»ÓÎ±ê */
    int right_index;                      /* ÓÒÎ»ÓÎ±ê */
    float left_abs;                       /* ×óÎ»µÄÖµÓëÄ¿±êÖµÖ®¼äµÄ²îµÄ¾ø¶ÔÖµ */
    float right_abs;                      /* ÓÒÎ»µÄÖµÓëÄ¿±êÖµÖ®¼äµÄ²îµÄ¾ø¶ÔÖµ */

    ret_index = 0;
    left_index = 0;
    right_index = 256 - 1;
    mid_index = 0;
    left_abs = 0;
    right_abs = 0;

    while (left_index != right_index) {
        mid_index = (right_index + left_index) / 2;
        if (value <= g[mid_index]) {
            right_index = mid_index;
        }
        else {
            left_index = mid_index;
        }
        if (right_index - left_index < 2) {
            break;
        }
    }
    left_abs = fabs(g[left_index] - value);
    right_abs = fabs(g[right_index] - value);
    ret_index = right_abs <= left_abs ? right_index : left_index;
    return ret_index;
}

/********************************************************
*   @brief       : ���̴߳�������
*   @param  args : ���̴߳���Ĳ���
*   @return      : void
********************************************************/
void * threadProcess(void* args) {

    pthread_t myid = pthread_self();
    paramThread *para = (paramThread *)args;
    int w = para->w;
    int h = para->h;
    cv::Mat image(h, w, CV_8UC1, (uchar *)para->data);
    *(para->quality) = calculate_m(image, 100, 0.1);
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
    *(para->quality) = calculate_m(image, 100, 0.1);
    //cout << *(para->quality) << endl;
    pthread_exit(NULL);
    return NULL;
}
/********************************************************
*   @brief       : ʵ��ͼ��ָ
*   @param  num  :  �ָ����
*   @param  type : 0����ֱ�ָ�(�Ƽ�)��1��ˮƽ�ָ���Ƽ���
*   @return      : vector<cv::Mat>
*   PS��ʹ��ˮƽ�ָ�ʱ��type=1�����������������catImage����ƴ�ӣ�
*   ʹ�ô�ֱ�ָ�ʱ��type=0�������Բ�����catImage����Ϊ�Ƕ�ԭͼ���в�����
********************************************************/

vector<cv::Mat> splitImage(cv::Mat image, int num, int type) {
    int rows = image.rows;
    int cols = image.cols;
    vector<cv::Mat> v;
    if (type == 0) {//��ֱ�ָ�
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
    else if (type == 1) {//ˮƽ�ָ�
        for (size_t i = 0; i < num; i++) {
            int star = cols / num * i;
            int end = cols / num * (i + 1);
            if (i == num - 1) {
                end = cols;
            }
            //cv::Mat b = image.colRange(star, end);
            v.push_back(image.colRange(star, end).clone());
        }
    }
    return  v;
}

/********************************************************
*   @brief       : ʵ��ͼ��ƴ�ӣ�
*   @param  v    :
*   @param  type : 0����ֱƴ�ӣ�1��ˮƽƴ��
*   @return      : Mat
********************************************************/
cv::Mat catImage(vector<cv::Mat> v, int type) {
    cv::Mat dest = v.at(0);
    for (size_t i = 1; i < v.size(); i++)
    {
        if (type == 0)//��ֱƴ��
        {
            cv::vconcat(dest, v.at(i), dest);
        }
        else if (type == 1)//ˮƽƴ��
        {
            cv::hconcat(dest, v.at(i), dest);
        }
    }
    return dest;
}

void correct_vector_creat(uchar* gamma_i, float num)
{
    float temp;
    int index;
    for (int i = 0; i <= 255; i++)
    {
        temp = g[i] + log(num);
        if (temp >= 2.0842)
            temp = 2.0842;
        else if (temp <= -5.1898)
            temp = -5.1898;
        index = get_near_index(temp);
        *(gamma_i + i) = index;
    }
}

double muti_thread_base(Mat gray_image)
{
    int type = 0;
    vector<cv::Mat> v = splitImage(gray_image, THREAD_NUMS, type);
    vector<double> q(THREAD_NUMS);
    paramThread args_base[THREAD_NUMS];
    pthread_t pt_base[THREAD_NUMS]; //����THREAD_NUMS�����߳�
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        args_base[i].h = v.at(i).rows;
        args_base[i].w = v.at(i).cols;
        args_base[i].data = v.at(i).data;
        args_base[i].quality = &q[i];
        pthread_create(&pt_base[i], NULL, &threadProcess_base, (void *)(&args_base[i]));
    }
    /*�ȴ�ȫ�����̴߳������*/
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        pthread_join(pt_base[i], NULL);
    }
    double r = 0;
    for (int i = 0; i < THREAD_NUMS; i++)
        r += q[i];
    return r;
}

double muti_thread(Mat gray_image)
{
    int type = 0;
    //һ��Ҫ������һ�䣬��Ȼ����ҡ�������֪��Ϊʲô
    Mat temp_image = gray_image.clone();
    vector<cv::Mat> v = splitImage(temp_image, THREAD_NUMS, type);
    vector<double> q(THREAD_NUMS);
    paramThread args[THREAD_NUMS];
    pthread_t pt[THREAD_NUMS];  //����THREAD_NUMS�����߳�
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        args[i].h = v.at(i).rows;
        args[i].w = v.at(i).cols;
        args[i].data = v.at(i).data;
        args[i].quality = &q[i];
        pthread_create(&pt[i], NULL, &threadProcess, (void *)(&args[i]));
    }
    /*�ȴ�ȫ�����̴߳������*/
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        pthread_join(pt[i], NULL);
    }
    double r = 0;
    for (int i = 0; i < THREAD_NUMS; i++)
        r += q[i];
    //Mat t = catImage(v, 0);
    //imshow("1", t);
    return r;
}


double calculate_m(Mat gray_image, double lamda, double delta)
{
    Mat image = gray_image.clone();
    Mat grad_x, grad_y, grad_mag;
    Sobel(image, grad_x, CV_64F, 1, 0, 3);
    Sobel(image, grad_y, CV_64F, 0, 1, 3);
    magnitude(grad_x, grad_y, grad_mag);
    int col = gray_image.cols;
    int row = gray_image.rows;
    Scalar ss = sum(grad_mag);
    return ss[0]/col/row;
	// normalize(grad_mag, grad_mag, 0, 1, NORM_MINMAX);
	// //normalize(gray_image, gray_image, 0, 1, NORM_MINMAX);
	// double result = 0;
	// double *p;
	// for(int i=0;i<row;i++)
	// {
	// 	p = grad_mag.ptr<double>(i);//获取每行首地址
	// 	for(int j=0;j<col;++j)
	// 	{
	// 		if(p[j]>delta)
	// 			result = result + p[j];
	// 		else
	// 			continue;
	// 	}
	// }
}
double get_suitable_gamma(Mat gray_image, vector<cv::Mat> LUTT)
{
    double value = 0;
    int num = -1;
    double value_0 = muti_thread_base(gray_image);
    //double value_0 = 20000;
    cout << value_0 << endl;

    double max_value = value_0;
    for (int i = 0; i < GAMMA_NUMS; i++)
    {
        Mat temp_image = gray_image.clone();
        Mat index = LUTT[i];
        LUT(temp_image,index,temp_image);
        value = muti_thread(temp_image);
        cout << value << endl;
        if (value > max_value)
        {
            num = i;
            max_value = value;
        }
    }
    if (value < 5)
    {
        return -1;
    }
    else if (max_value <= 1.15*value_0)
    {
        return 1;
    }
    else
        return gamma_parament[num];
}
void exposure_adjust(Mat color_image, double& exposure_time, double& exposure_gain, vector<Mat>LUT)
{
    Mat gray_image;
    cvtColor(color_image, gray_image, CV_BGR2GRAY);
    double gamma = get_suitable_gamma(gray_image, LUT);
    if (gamma == -1)
    {
        flag = flag + 1;
        if (flag > 30)
        {
            exposure_time = 450;
            exposure_gain = 64;
            flag = 0;
        }
    }
    else
    {
        //cout<<gamma<<endl;
        double alfa = 1;
        double k = 0.3;
        if (gamma == 0.5||gamma==2)
        {
        	exposure_time = exposure_time *(1 + k*alfa*(gamma - 1));
        }
        else
        {
        	exposure_gain = exposure_gain *(1 + k*alfa*(gamma - 1));
        }        //���Կ���
        //cout<<exposure_time<<endl;
        //  cout << 1 + alfa * 0.3*(gamma - 1) << endl;
        //cout<<exposure_time<<endl;
        if (exposure_time > 1250)
            exposure_time = 1250;
        else if (exposure_time < 25)
            exposure_time = 25;

        if (exposure_gain > 127)
            exposure_gain = 127;
        else if (exposure_gain < 1)
            exposure_gain = 1;
        //printf("exposure_time = %f\n", exposure_time);
        //printf("exposure_gain = %f\n", exposure_gain);
    }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    colorImg = cv_ptr->image; 
    Mat temp;
    imshow("view",colorImg);  
    cv::waitKey(1);
    //GaussianBlur(colorImg,colorImg,Size(3,3),0);
    double T1 = static_cast<double>(cv::getTickCount());
    exposure_adjust(colorImg, exposure_time, exposure_gain, LUT_GAMMA);
    double T2 = static_cast<double>(cv::getTickCount());
    printf("Thread run times = %3.3fms\n,", (T2 - T1) * 1000 / cv::getTickFrequency());
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
    fstream in("/home/wangyu/data/t.txt"); 
    cin.rdbuf(in.rdbuf()); 
    for (int i = 0; i < 256; i++)
    {
        cin >> t[i];
    }
    fstream in2("/home/wangyu/data/g.txt"); 
    cin.rdbuf(in2.rdbuf()); 
    for (int i = 0; i < 256; i++)
    {
        cin >> g[i]; 
    }   
    gamma_parament.push_back(0.8);
    gamma_parament.push_back(0.5);
    gamma_parament.push_back(1.3);
    gamma_parament.push_back(2);
    correct_vector_creat(gamma_0, gamma_parament[0]);
    correct_vector_creat(gamma_1, gamma_parament[1]);
    correct_vector_creat(gamma_2, gamma_parament[2]);
    correct_vector_creat(gamma_3, gamma_parament[3]);
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
    chatter_pub = n.advertise<std_msgs::Int32MultiArray>("/camera/set_exposure", 1000);
    sub = n.subscribe("/d435i/color/image_raw", 1000, imageCallback); 
    //ros::Rate loop_rate(10);
    ROS_INFO("Please send message!");  
    ros::spin();  
    return 0 ;
} 