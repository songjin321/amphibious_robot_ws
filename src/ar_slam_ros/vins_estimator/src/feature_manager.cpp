#include "feature_manager.h"
#include <cv_bridge/cv_bridge.h>
int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frame_count, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, feature_tracker::FeaturePtr frame, double td)
{
    // from frame to construct image;
    // 获得描述子信息
    cv_bridge::CvImageConstPtr ptr;
    try
    {
        ptr = cv_bridge::toCvCopy(frame->descriptors, frame->descriptors.encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    map<int, cv::Mat> descriptors;
    // 将图像特征点数据存到一个map容器中，key是特征点id
    auto img_msg = &(frame->keypoints);
    for (unsigned int i = 0; i < img_msg->points.size(); i++)
    {
        int v = img_msg->channels[0].values[i] + 0.5; // ？？？这是什么操作
        int feature_id = v / NUM_OF_CAM;
        int camera_id = v % NUM_OF_CAM;
        double x = img_msg->points[i].x;
        double y = img_msg->points[i].y;
        double z = img_msg->points[i].z;
        double p_u = img_msg->channels[1].values[i];
        double p_v = img_msg->channels[2].values[i];
        double velocity_x = img_msg->channels[3].values[i];
        double velocity_y = img_msg->channels[4].values[i];
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        descriptors[feature_id] = ptr->image.row(i);
    }

    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;

    // 候选待匹配的特征，id，描述子，每帧信息
    std::vector<std::tuple<int, cv::Mat, FeaturePerFrame>> candidate_feature;
    // int index_des = 0; // fuck,这是一个map,不是顺序容器!FUCK!!! 5个小时浪费了!!!
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it) {
            return it.feature_id == feature_id;
        });

        if (it == feature.end())
        {
            // 将这个点加入候选匹配集合
            cout << " candidate feature id = " << feature_id << endl;
            candidate_feature.push_back(std::make_tuple(feature_id, descriptors[feature_id].clone(), f_per_fra));
        }
        else
        {
            f_per_fra.offset = frame_count - it->start_frame; 
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }
    // 对候选特征点和地图匹配
    cout << "candidate_feature size = " << candidate_feature.size() << endl;
    unordered_map<int, int> candidate_map; // 匹配上的点对，下标索引, 候选<->地图
    cv::Mat train_dep;
    cv::Mat query_dep;
    std::vector<std::vector<cv::DMatch>> matches;
    std::vector<cv::DMatch> good_matches;
    for (auto &f : feature)
    {
        train_dep.push_back(f.descriptor);
    }
    for (auto &f : candidate_feature)
    {
        query_dep.push_back(std::get<1>(f));
    }
    cout << "train rows = " << train_dep.rows << " train cols = " << train_dep.cols << endl;
    cout << "query_dep rows = " << query_dep.rows << " query_dep cols = " << query_dep.cols << endl;
    if (train_dep.rows !=0 && query_dep.rows != 0)
    {
        // matcher_flann.match(query_dep, train_dep, matches); 
        cv::BFMatcher matcher;
        matcher.knnMatch(query_dep, train_dep, matches, 2);
    }

    // select the best matches
    /*
    float min_dis = std::min_element(
                        matches.begin(), matches.end(),
                        [](const cv::DMatch &m1, const cv::DMatch &m2) {
                            return m1.distance < m2.distance;
                        })
                        ->distance;
    float match_ratio = 2.0; // ratio for selecting  good matches
    */
    for(int i=0; i < matches.size(); i++)
    {
        if (matches[i][0].distance < 0.5*matches[i][1].distance)
            good_matches.push_back(matches[i][0]);  
    }


    for (cv::DMatch &m : good_matches)
    {
        // if (m.distance < 0.05)
        {
            cout << "good match " << endl;
            cout << "m.distance = " << m.distance;
            cout << " query id = " << std::get<0>(candidate_feature[m.queryIdx]);
            auto iter_feature = feature.begin();
            std::advance(iter_feature, m.trainIdx);
            cout << " train id = " << iter_feature->feature_id << endl;
            candidate_map.insert({std::get<0>(candidate_feature[m.queryIdx]), iter_feature->feature_id});
            // cout << "query descriptors = " << query_dep.row(m.queryIdx) << endl;
            // cout << "train descriptors = " << train_dep.row(m.trainIdx) << endl;
        }
    }
    cout << "good matches: " << candidate_map.size() << endl;
    // candidate_map.clear(); //clear之后相当于关闭匹配的作用
    // 根据匹配结果处理特征点
    match_show.clear();
    for (int i = 0; i < (int)candidate_feature.size(); i++)
    {
        auto iter = candidate_map.find(std::get<0>(candidate_feature[i]));
        // 没匹配上的，新建一个特征点
        if (iter == candidate_map.end())
        {
            
            FeaturePerId feature_per_id = FeaturePerId(std::get<0>(candidate_feature[i]), frame_count);
            feature_per_id.descriptor = std::get<1>(candidate_feature[i]).clone();
            feature.push_back(feature_per_id);
            FeaturePerFrame f_per_frame = std::get<2>(candidate_feature[i]);
            f_per_frame.offset = 0;
            feature.back().feature_per_frame.push_back(f_per_frame);
            
        }
        else
        // 匹配上的，对应id的frame上加一个观测帧
        {
            for(FeaturePerId& iter_feature : feature)
            {
                if (iter_feature.feature_id == iter->second)
                {
                    FeaturePerFrame f_per_frame = std::get<2>(candidate_feature[i]);
                    f_per_frame.offset = frame_count - iter_feature.start_frame;
                    // for showing track
                    FeaturePerId iter_feature_for_match = iter_feature;
                    iter_feature_for_match.feature_per_frame.push_back(f_per_frame);
                    match_show.push_back({iter->first, iter_feature_for_match});

                    // 修改image中对应元素的id
                    /*
                    auto iter_image_id = image.find(std::get<0>(candidate_feature[i]));
                    image.insert({iter->second, iter_image_id->second});
                    image.erase(iter_image_id->first);
                    */

                    // add to sliding window
                    // 判断一下,
                    bool wrong_match = false;
                    for (auto& per_frame : iter_feature.feature_per_frame)
                    {
                        if (per_frame.offset == f_per_frame.offset )
                            wrong_match = true;
                    }
                    if (!wrong_match)
                    {
                        iter_feature.feature_per_frame.push_back(f_per_frame);
                        last_track_num++;
                    }    
                    break;
                }
            }
        }
    }

    // feature_per_id.descriptor = frame->descriptors.row(id_index[feature_id]).clone();

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);
        if (it.feature_id == 59)
        {
            ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
            int sum = 0;
            for (auto &j : it.feature_per_frame)
            {
                //ROS_DEBUG("is_used %d,", int(j.is_used));
                printf("offset = %d \n", j.offset);
                sum += 1;
                printf("(%lf,%lf) \n", j.point(0), j.point(1));
            }
            // ROS_ASSERT(it.used_num == sum);
        }
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }
    }
}

void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            for (FeaturePerFrame& feature_frame : it->feature_per_frame) // fuck! auto害死人啊,auto默认是拷贝,不是引用
                feature_frame.offset--;
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            for (FeaturePerFrame& feature_frame : it->feature_per_frame) // fuck! auto害死人啊,这是拷贝,不是引用
                feature_frame.offset--;
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            it->feature_per_frame[j].offset--;
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}