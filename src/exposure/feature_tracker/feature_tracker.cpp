/**
 * @file optical_flow.cpp
 * @author Song Jin (songjinxs@163.com)
 * @brief test opencv optical flow
 * @version 0.1
 * @date 2019-11-13
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");
// 相机的对数曝光函数 lnf^{-1}, f为相机响应曲线
std::vector<double> g = {
    -4.13612572654150,
    -3.98620536975460,
    -3.83628501296769,
    -3.68677388249927,
    -3.54297362914333,
    -3.40546501067732,
    -3.27281016696763,
    -3.14712675326595,
    -3.02920701782898,
    -2.92095894324875,
    -2.82270348933015,
    -2.73404861535032,
    -2.65416171840129,
    -2.58195671808627,
    -2.51583639313012,
    -2.45447053966595,
    -2.39679330797833,
    -2.34189402275931,
    -2.28933900588226,
    -2.23925729562224,
    -2.19149998087869,
    -2.14606163045560,
    -2.10337843750101,
    -2.06326684356557,
    -2.02525487370989,
    -1.98887832012158,
    -1.95351871968992,
    -1.91889856237833,
    -1.88448519098669,
    -1.84965967025398,
    -1.81472623518215,
    -1.77998508079245,
    -1.74552465114825,
    -1.71133372233171,
    -1.67792442425530,
    -1.64562863245317,
    -1.61456281713889,
    -1.58471588471900,
    -1.55595411683823,
    -1.52858352003799,
    -1.50213605728866,
    -1.47638117060122,
    -1.45143192277191,
    -1.42727895556727,
    -1.40379638525271,
    -1.38073606930124,
    -1.35797338335871,
    -1.33565019319662,
    -1.31379298041046,
    -1.29269976798533,
    -1.27269684504827,
    -1.25317835101188,
    -1.23370206115518,
    -1.21370272354045,
    -1.19283540320911,
    -1.17134775297209,
    -1.14920194520392,
    -1.12648227918160,
    -1.10334706702969,
    -1.07974258140393,
    -1.05608291231055,
    -1.03260229851967,
    -1.00951438960284,
    -0.986922623874292,
    -0.964946937797760,
    -0.943757780618733,
    -0.923565617772420,
    -0.904377825539251,
    -0.886171645357729,
    -0.868917584279743,
    -0.852419202809824,
    -0.836465158352939,
    -0.820795767652871,
    -0.805303001655798,
    -0.789911039137945,
    -0.774697043991726,
    -0.759540677505913,
    -0.744265855486266,
    -0.728598709735408,
    -0.712448182757279,
    -0.696026273130399,
    -0.679491115478615,
    -0.663098085878169,
    -0.647027147249364,
    -0.631183518055068,
    -0.615444186168850,
    -0.599731855837544,
    -0.583915031420858,
    -0.567913079614355,
    -0.551875263449940,
    -0.535856373498811,
    -0.519735913307625,
    -0.503576522983016,
    -0.487535265532869,
    -0.471664981426393,
    -0.455862592086278,
    -0.440166463870649,
    -0.424563423228175,
    -0.409066399174158,
    -0.393714138525457,
    -0.378602474625987,
    -0.363762473017014,
    -0.349120700654950,
    -0.334631501042437,
    -0.320279786618879,
    -0.306031370581915,
    -0.291922765347449,
    -0.278043816893458,
    -0.264330370903482,
    -0.250740101613296,
    -0.237279829637643,
    -0.223724796950736,
    -0.210088854241341,
    -0.196447726862572,
    -0.182897103209173,
    -0.169526683556997,
    -0.156431224580295,
    -0.143579393308280,
    -0.130926571193653,
    -0.118453504726904,
    -0.105884947348792,
    -0.0930738489695555,
    -0.0800309176938856,
    -0.0668466138600981,
    -0.0536005128735984,
    -0.0403067055246293,
    -0.0269306475278272,
    -0.0134841375633824,
    0.0,
    0.0134628177675788,
    0.0269275784666436,
    0.0403331151690945,
    0.0536776059627099,
    0.0668330104267234,
    0.0797359612636123,
    0.0923607066339252,
    0.104906751735891,
    0.117495181140521,
    0.130258903089815,
    0.143188595380138,
    0.156159768792280,
    0.169135362639509,
    0.181984344016664,
    0.194630519791872,
    0.207044182761987,
    0.219219479949044,
    0.231109966557244,
    0.242732315286275,
    0.254162137071831,
    0.265439168946936,
    0.276539856811172,
    0.287651888160031,
    0.298922299893577,
    0.310443766271778,
    0.322179352829347,
    0.334151331017299,
    0.346202266142862,
    0.358097780540761,
    0.369748395507247,
    0.381035533515540,
    0.391855032488942,
    0.402492284062757,
    0.413157391071103,
    0.424033528508149,
    0.435195713295548,
    0.446657511803366,
    0.458458685184110,
    0.470544788717981,
    0.482775631869739,
    0.495122867020554,
    0.507616233440627,
    0.520229613013802,
    0.532836874164863,
    0.545271639304295,
    0.557369148964958,
    0.569254474577193,
    0.581084213921366,
    0.593069828540776,
    0.605366982519730,
    0.618048814904185,
    0.631208249677560,
    0.644882747030356,
    0.659077300021741,
    0.673628339360982,
    0.688232739394874,
    0.702759470838592,
    0.717162215567645,
    0.731394382809952,
    0.745375096891018,
    0.758981109304428,
    0.772133601922318,
    0.784732010886161,
    0.796946143745303,
    0.808951187601528,
    0.821067793992288,
    0.833046891386351,
    0.844788309400680,
    0.856283381411006,
    0.867468518379035,
    0.878432123949019,
    0.889169542679250,
    0.899614892954831,
    0.909958969176808,
    0.920536402442679,
    0.931328370410010,
    0.942240475132246,
    0.953284281331777,
    0.964470787739559,
    0.975660159937586,
    0.986709264883718,
    0.997467405162046,
    1.00782222383515,
    1.01766157881062,
    1.02695162236301,
    1.03564149326218,
    1.04372122687568,
    1.05112044484293,
    1.05791815121723,
    1.06416152681246,
    1.06989439006554,
    1.07503181459206,
    1.07969150297343,
    1.08400714423860,
    1.08806669027323,
    1.09197150470048,
    1.09589559452030,
    1.10006073300432,
    1.10467618926308,
    1.11006716846271,
    1.11655044023211,
    1.12401050503193,
    1.13235765890165,
    1.14156701242288,
    1.15176566131041,
    1.16313765506725,
    1.17596926581635,
    1.19063307949891,
    1.20761337238212,
    1.22752061718193,
    1.25106682494898,
    1.27904070067112,
    1.31241593503985,
    1.35236895046788,
    1.40045015175533,
    1.45853338192673,
    1.52850566231686,
    1.61250112520894,
    1.71278256298329,
    1.82926864692935,
    1.95907105015425,
    2.09403437707943,
    2.21137234476342,
    2.31342660377610,
    2.40438905630453,
    2.48849479848250,
    2.57260054066048};
class FeatureTracker
{
public:
    FeatureTracker() : g_table(1, 256, CV_64FC1)
    {
        set_g_table(g);
    }

    void readImage(cv::Mat image, int exposure_time)
    {
        if (image.empty())
            std::cerr << "empty image" << std::endl;
        if (image.type() == CV_8UC1)
        {
            new_gray = image;
            cv::cvtColor(image, new_color, CV_GRAY2BGR);
        }
        else if (image.type() == CV_8UC3)
        {
            cv::cvtColor(image, new_color, CV_RGB2BGR);
            cv::cvtColor(image, new_gray, CV_RGB2GRAY);
        }
        // use inverse photometric function to correct image
        cv::LUT(new_gray, g_table, new_correct_gray);
        cv::subtract(new_correct_gray, log(exposure_time), new_correct_gray);

        double g_interval = g[255] - g[0] -1 ;
        new_correct_gray.convertTo(new_correct_gray, CV_8UC1, 255.0/g_interval, -255.0*(g[0]-1)/g_interval);
        std::cout << new_correct_gray.type() << std::endl;
        if (!old_gray.empty())
        {
            std::vector<uchar> status;
            std::vector<float> err;
            std::cerr << "before track pts size = " << old_pts.size() << std::endl;
            cv::calcOpticalFlowPyrLK(old_correct_gray, new_correct_gray, old_pts, new_pts, status, err, cv::Size(21, 21), 3);
            for (size_t i = 0; i < new_pts.size(); i++)
            {
                if (status[i] && !inBorder(new_pts[i]))
                {
                    status[i] = 0;
                }
            }
            for (auto &n : track_cnt)
                n++;

            // plot track result;
            cv::hconcat(old_color, new_color, show_img);
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i] == 1)
                {
                    // cv::line(show_img, old_pts[i], cv::Point2i(new_pts[i].x+old_color.cols, new_pts[i].y), cv::Scalar(0, 255, 0), 1);
                    cv::circle(show_img, old_pts[i], 3, cv::Scalar(0, 255, 0), 2);
                    cv::circle(show_img, cv::Point2i(new_pts[i].x + old_color.cols, new_pts[i].y), 3, cv::Scalar(0, 255, 0), 2);
                }
                else if (status[i] == 0)
                {
                    cv::circle(show_img, old_pts[i], 3, cv::Scalar(255, 0, 0), 2);
                }
            }
            reduceVector(status);
            if (new_pts.size() < 20)
                std::cerr << red << "end track pts size = " << new_pts.size() << reset << std::endl;
            else
            {
                std::cerr << "end track pts size = " << new_pts.size() << std::endl;
            }
        }

        // 对当前帧检测新特征点
        setMask();
        detected_pts.clear();
        if (MAX_CNT - static_cast<int>(new_pts.size()) > 0)
        {
            cv::goodFeaturesToTrack(new_gray, detected_pts, MAX_CNT - new_pts.size(), 0.01, MIN_DIST, mask);
            if (!show_img.empty())
            {
                for (size_t i = 0; i < detected_pts.size(); i++)
                {
                    cv::circle(show_img, cv::Point2i(detected_pts[i].x + old_color.cols, detected_pts[i].y), 3, cv::Scalar(0, 0, 255), 2);
                }
            }
            std::cerr << "detected_pts size = " << detected_pts.size() << std::endl;
        }

        for (size_t i = 0; i < detected_pts.size(); i++)
        {
            new_pts.push_back(detected_pts[i]);
            track_cnt.push_back(1);
            ids.push_back(factory_id);
            factory_id++;
        }

        old_gray = new_gray.clone();
        old_color = new_color.clone();
        old_correct_gray = new_correct_gray.clone();
        old_pts = new_pts;
    }
    int getTrackedNmt()
    {
        return new_pts.size();
    }
    void set_g_table(const std::vector<double> &g)
    {
        //
        if (g.size() != 256)
        {
            ROS_WARN("uncorrect g function size!");
            return;
        }
        // double g_max = g[255] - g[0];
        for (int i = 0; i < 256; ++i)
        {
            // double g_set = (g[i] - g[0]) * 255.0/g_max;
            double g_set = g[i];
            std::cout << "g = " << g_set << std::endl;
            g_table.at<double>(0, i) = g_set;
        }
    }
    cv::Mat show_img;

private:
    void rejectWithF()
    {
    }

    bool inBorder(const cv::Point2f &pt)
    {
        const int BORDER_SIZE = 1;
        int img_x = cvRound(pt.x);
        int img_y = cvRound(pt.y);
        return BORDER_SIZE <= img_x && img_x < old_gray.cols - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < old_gray.rows - BORDER_SIZE;
    }

    void reduceVector(const std::vector<uchar> status)
    {
        if (ids.size() != new_pts.size() || new_pts.size() != track_cnt.size() || ids.size() != track_cnt.size())
            std::cout << "uncorrect variable size()!" << std::endl;
        int j = 0;
        int size = ids.size();

        // reduce id
        j = 0;
        for (int i = 0; i < size; i++)
            if (status[i])
                ids[j++] = ids[i];
        ids.resize(j);

        // reduce pts
        j = 0;
        for (int i = 0; i < size; i++)
            if (status[i])
                new_pts[j++] = new_pts[i];
        new_pts.resize(j);

        // reduce count
        j = 0;
        for (int i = 0; i < size; i++)
            if (status[i])
                track_cnt[j++] = track_cnt[i];
        track_cnt.resize(j);
    }

    void setMask()
    {
        mask = cv::Mat(old_gray.rows, old_gray.cols, CV_8UC1, cv::Scalar(255));

        // prefer to keep features that are tracked for long time
        std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

        for (unsigned int i = 0; i < new_pts.size(); i++)
            cnt_pts_id.push_back(std::make_pair(track_cnt[i], std::make_pair(new_pts[i], ids[i])));

        sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const std::pair<int, std::pair<cv::Point2f, int>> &a, const std::pair<int, std::pair<cv::Point2f, int>> &b) {
            return a.first > b.first;
        });

        new_pts.clear();
        ids.clear();
        track_cnt.clear();

        for (auto &it : cnt_pts_id)
        {
            if (mask.at<uchar>(it.second.first) == 255)
            {
                new_pts.push_back(it.second.first);
                ids.push_back(it.second.second);
                track_cnt.push_back(it.first);
                cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
            }
        }
    }

public:
    cv::Mat old_gray;
    cv::Mat old_color;
    cv::Mat new_gray;
    cv::Mat new_color;
    cv::Mat old_correct_gray;
    cv::Mat new_correct_gray;
    std::vector<cv::Point2f> old_pts;      // 上一帧追踪到的特征点
    std::vector<cv::Point2f> new_pts;      // 这一帧追踪到的特征点
    std::vector<int> track_cnt;            // 每个特征点追踪的次数
    std::vector<int> ids;                  // 每个特征点的id
    std::vector<cv::Point2f> detected_pts; // new points detected in new image
    const int MAX_CNT = 200;
    const int MIN_DIST = 30;
    int factory_id = 0;
    cv::Mat mask;
    cv::Mat g_table; // 反响应函数
};

FeatureTracker feature_tracker;
ros::Publisher pub_match;
ros::Publisher pub_correct;
ros::Publisher pub_tracked_nmt;

void imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // convert ros image to cv mat, color to gray
    cv_bridge::CvImageConstPtr ptr;
    try
    {
        ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::cerr << "begin track image" << std::endl;
    int exposure_time, exposure_gain;
    std::istringstream iss(img_msg->header.frame_id);
    iss >> exposure_time >> exposure_gain;
    feature_tracker.readImage(ptr->image, exposure_time);
    std::cerr << "end track image" << std::endl;

    if (!feature_tracker.show_img.empty())
    {
        cv_bridge::CvImage pub_ptr;
        pub_ptr.image = feature_tracker.show_img;
        pub_ptr.encoding = sensor_msgs::image_encodings::BGR8;
        pub_match.publish(pub_ptr.toImageMsg());
    }
    if (!feature_tracker.show_img.empty())
    {
        cv_bridge::CvImage pub_ptr;
        pub_ptr.image = feature_tracker.show_img;
        pub_ptr.encoding = sensor_msgs::image_encodings::BGR8;
        pub_match.publish(pub_ptr.toImageMsg());
    }
    if (!feature_tracker.new_correct_gray.empty())
    {
        cv_bridge::CvImage pub_ptr;
        pub_ptr.image = feature_tracker.new_correct_gray;
        pub_ptr.encoding = sensor_msgs::image_encodings::MONO8;
        pub_correct.publish(pub_ptr.toImageMsg());
    }
    
    std_msgs::Int32 tracked_cmt_msg;
    tracked_cmt_msg.data = feature_tracker.getTrackedNmt();
    pub_tracked_nmt.publish(tracked_cmt_msg);
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "uncorrect arguments, Use: ./feature_tracker NodeName IMAGE_TPOPIC";
        return -1;
    }
    std::string node_name(argv[1]);
    std::string image_topic(argv[2]);
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Subscriber image_sub = nh.subscribe(image_topic, 1, imageCallback);

    pub_match = nh.advertise<sensor_msgs::Image>(node_name + "/tracker_image", 1);
    pub_correct = nh.advertise<sensor_msgs::Image>(node_name + "/correct_image", 1);
    pub_tracked_nmt = nh.advertise<std_msgs::Int32>(node_name + "/tracked_nmt", 1);
    ros::spin();
    return 0;
}