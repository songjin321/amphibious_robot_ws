#include "feature_tracker.h"
#include <bitset>
int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(cv::Mat &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < v.rows; i++)
        if (status[i])
            v.row(i).copyTo(v.row(j++)); // 注意要使用copyTo。。。，直接访问row，是一个copy，不会改变原mat
    v = v.rowRange(0, j);
}

FeatureTracker::FeatureTracker() : brief_feature_detector()
{
    InitSiftData(siftData, 32768, true, true);
    InitCuda(0);
}

void FeatureTracker::setMask()
{
    if (FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    // track_count, point, id, descriptor
    vector<tuple<int, cv::Point2f, int, cv::Mat>> cnt_pts_id_des;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id_des.push_back(make_tuple(track_cnt[i], forw_pts[i], ids[i], descriptors.row(i).clone()));

    sort(cnt_pts_id_des.begin(), cnt_pts_id_des.end(), [](const tuple<int, cv::Point2f, int, cv::Mat> &a, const tuple<int, cv::Point2f, int, cv::Mat> &b) {
        return std::get<0>(a) > std::get<0>(b);
    });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    descriptors.release();
    for (auto &it : cnt_pts_id_des)
    {
        if (mask.at<uchar>(std::get<1>(it)) == 255)
        {
            track_cnt.push_back(std::get<0>(it));
            forw_pts.push_back(std::get<1>(it));
            ids.push_back(std::get<2>(it));
            cv::circle(mask, std::get<1>(it), MIN_DIST, 0, -1);
            descriptors.push_back(std::get<3>(it));
        }
    }
}

void FeatureTracker::addPoints(cv::Mat n_pts_descriptors)
{
    for (size_t i = 0; i < n_pts.size(); i++)
    {
        forw_pts.push_back(n_pts[i]);
        ids.push_back(-1);
        track_cnt.push_back(1);
        descriptors.push_back(n_pts_descriptors.row(i).clone());
    }
    ROS_DEBUG("add point success!");
}

void FeatureTracker::Init()
{
    // 将第一次初始化gpu放在这,因为第一次检测sift需要接近1s左右
    CudaImage img;
    cv::Mat temp(512, 512, CV_32FC1, cv::Scalar(0.5));
    img.Allocate(512, 512, iAlignUp(512, 128), false, NULL, (float *)temp.data);
    img.Download();
    ExtractSift(siftData, img, 5, initBlur, thresh, 1.0f, false);
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        ROS_DEBUG("the feature number before optical tracking %d", (int)cur_pts.size());
        for (int i = 0; i < int(forw_pts.size()); i++)
        {
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        }

        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        reduceVector(descriptors, status);
        ROS_DEBUG("the feature number after optical tracking %d", (int)forw_pts.size());
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_cal_feature;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        // n_max_cnt = 150; // for debug
        cv::Mat n_pts_descriptors;
        if (n_max_cnt > 0)
        {
            if (mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            /*
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
            for (int i=0; i < n_pts.size(); i++)
            {
                cv::Mat descriptor(1, 128, CV_32F);
                n_pts_descriptors.push_back(descriptor);
            }
            */
            
            std::vector<std::pair<cv::KeyPoint, cv::Mat> > coarse_keypoints_descriptors;
            std::vector<std::pair<cv::KeyPoint, cv::Mat>> fine_keypoints_descriptors;
              
            cv::Mat limg;
            forw_img.convertTo(limg, CV_32FC1);
            unsigned int w = limg.cols;
            unsigned int h = limg.rows;

            CudaImage img;
            img.Allocate(w, h, iAlignUp(w, 128), false, NULL, (float *)limg.data);
            img.Download();
            ExtractSift(siftData, img, 5, initBlur, thresh, scale_thresh, false);
            cout << "sift points size = " << siftData.numPts << endl;
            SiftPoint *sift_points = siftData.h_data;
            for (size_t i = 0; i < siftData.numPts; i++)
            {
                cv::KeyPoint keypoint;
                keypoint.pt.x = sift_points[i].xpos;
                keypoint.pt.y = sift_points[i].ypos;
                keypoint.response = sift_points[i].sharpness;
                keypoint.angle = sift_points[i].orientation;
                keypoint.octave = sift_points[i].edgeness; // 把边缘响应值赋值给octave
                cv::Mat descriptor(1, 128, CV_32F);
                for (size_t j = 0; j < 128; j++)
                {
                    descriptor.at<float>(0, j) = sift_points[i].data[j];
                }
                coarse_keypoints_descriptors.push_back({keypoint, descriptor});
            }
            
            // opencv sift detection
            /*
            vector<KeyPoint> sift_keypoints_opencv;
            f2d->detect(forw_img, sift_keypoints_opencv);
            for (auto keypoint : sift_keypoints_opencv)
            {
                cv::Mat descriptor(1, 128, CV_32F);

                coarse_keypoints_descriptors.push_back({keypoint, descriptor});
            }
            */
            // 这部分算法来自cv::goodFeatureToTrack
            // 对每一个点如果其MIN_DIST距离内有比其响应强的，则不考虑这个点
            // 如果这个点在mask中，也不考虑这个点
            // 对最后的点，我们考虑scale大的，即宏观特征，而不是细节特征,保证点数不超过MAX_CNT个
            // 按照响应从大到小排序
            std::sort(coarse_keypoints_descriptors.begin(), coarse_keypoints_descriptors.end(), 
            [](const std::pair<cv::KeyPoint, cv::Mat>& temp_1, const std::pair<cv::KeyPoint, cv::Mat>& temp_2){
                return temp_1.first.response > temp_2.first.response;
            });
            auto minDistance = MIN_DIST;
            size_t maxCorners = n_max_cnt;
            size_t ncorners = 0;
            if (minDistance >= 1)
            {
                const int cell_size = cvRound(minDistance);
                const int grid_width = (w + cell_size - 1) / cell_size;
                const int grid_height = (h + cell_size - 1) / cell_size;

                std::vector<std::vector<cv::Point2f>> grid(grid_width * grid_height);

                minDistance *= minDistance;

                for (auto keypoint_descriptor : coarse_keypoints_descriptors)
                {
                    int y = (int)(keypoint_descriptor.first.pt.y);
                    int x = (int)(keypoint_descriptor.first.pt.x);

                    bool good = true;

                    int x_cell = x / cell_size;
                    int y_cell = y / cell_size;

                    int x1 = x_cell - 1;
                    int y1 = y_cell - 1;
                    int x2 = x_cell + 1;
                    int y2 = y_cell + 1;

                    // boundary check
                    x1 = std::max(0, x1);
                    y1 = std::max(0, y1);
                    x2 = std::min(grid_width - 1, x2);
                    y2 = std::min(grid_height - 1, y2);

                    for (int yy = y1; yy <= y2; yy++)
                    {
                        for (int xx = x1; xx <= x2; xx++)
                        {
                            std::vector<cv::Point2f> &m = grid[yy * grid_width + xx];

                            if (m.size())
                            {
                                for (int j = 0; j < m.size(); j++)
                                {
                                    float dx = x - m[j].x;
                                    float dy = y - m[j].y;

                                    if (dx * dx + dy * dy < minDistance)
                                    {
                                        good = false;
                                        goto break_out;
                                    }
                                }
                            }
                        }
                    }

                break_out:

                    if (good)
                    {
                        if (mask.at<uchar>(cv::Point2f((float)x, (float)y)) == 0)
                            continue;
                        grid[y_cell * grid_width + x_cell].push_back(cv::Point2f((float)x, (float)y));

                        fine_keypoints_descriptors.push_back(keypoint_descriptor);
                        ++ncorners;

                        if (maxCorners > 0 && (int)ncorners == maxCorners)
                            break;
                    }
                }
            }
            else
            {
                for (auto keypoint_descriptor : coarse_keypoints_descriptors)
                {
                    fine_keypoints_descriptors.push_back(keypoint_descriptor);
                    ++ncorners;
                    if (maxCorners > 0 && (int)ncorners == maxCorners)
                        break;
                }
            }

            n_pts.clear();
            for (auto keypoint_descriptor : fine_keypoints_descriptors)
            {
                n_pts.push_back(cv::Point2f(keypoint_descriptor.first.pt.x, keypoint_descriptor.first.pt.y));
                n_pts_descriptors.push_back(keypoint_descriptor.second.clone());
            }
            
            // draw it 
            /*
            cv::Mat drawImg = forw_img.clone();
            for (auto pt : n_pts)
                cv::circle(drawImg, pt, 2, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Detected Sift Feature", drawImg);
            cv::waitKey(2);
            */
        }
        else
        {
            n_pts.clear();
        }

        ROS_DEBUG("detect feature costs: %fms", t_cal_feature.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints(n_pts_descriptors);
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(descriptors, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
    {
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
