/**
 * @file exp_optim_orient_nrsl.cpp
 * @author Song Jin (songjinxs@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-10-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "act_map/optim_orient.h"

#include <random>

#include <rpg_common/main.h>
#include <rpg_common/timer.h>
#include <rpg_common/save.h>
#include <vi_utils/map.h>
#include <vi_utils/cam_min.h>
#include <vi_utils/common_utils.h>
#include <vi_utils/vi_jacobians.h>
#include "act_map/sampler.h"
#include "act_map/conversion.h"
#include<fstream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

DEFINE_string(abs_map, "", "absolute path of the map file.");
DEFINE_string(abs_trace_dir, "", "trace dir to save results");

DEFINE_double(xrange, 5.0, "X range centered at zero.");
DEFINE_double(yrange, 5.0, "Y range centered at zero.");
DEFINE_double(zrange, 2.0, "Z range centered at zero.");
DEFINE_double(vox_res, 0.1, "Voxel size.");

DEFINE_double(angle_res_deg, 10.0, "resolution to sample points on a sphere.");
DEFINE_double(check_ratio, 0.1, "ratio of voxels to compute.");

using namespace act_map;
using namespace act_map::optim_orient;

struct CostFunctor{

    CostFunctor(double info, Eigen::Vector3d w_p, Eigen::Vector3d w_c):
    info_(info), w_p_(w_p), w_c_(w_c)
    {
    }

    template <typename T>
    bool operator()(const T* const w_c_view, T* residuals) const
    {
        T c_e3[3];
        T c_view[3];
        T w_e3[3];
        w_e3[0] = T(0.0);
        w_e3[1] = T(0.0);
        w_e3[2] = T(1.0);
        c_view[0] = T(w_p_[0] - w_c_[0]);
        c_view[1] = T(w_p_[1] - w_c_[1]);
        c_view[2] = T(w_p_[2] - w_c_[2]);
        ceres::QuaternionRotatePoint(w_c_view, w_e3, c_e3);
        T normal_z = (c_view[0] * c_e3[0] + c_view[1] * c_e3[1] + c_view[2] * c_e3[2])
        /(sqrt(c_view[0] * c_view[0] + c_view[1] * c_view[1] + c_view[2] * c_view[2]));
        residuals[0] = T(info_) * exp(-normal_z);
        // std::cout << " info = " << info_  << " w_p = " << w_p_ 
        // << " normal_z = " << normal_z  << " residual = " << residuals[0] << std::endl;
        return true;
    }

    static ceres::CostFunction* Create(const double info, const Eigen::Vector3d w_p, const Eigen::Vector3d w_c)
    {
        return (new ceres::AutoDiffCostFunction<CostFunctor, 1, 4>(
            new CostFunctor(info, w_p, w_c)
        ));
    }
private:
    double info_; // 路标点对当前位置相机的信息量
    Eigen::Vector3d w_p_; // 路标点在世界坐标的位置
    Eigen::Vector3d w_c_; // 当前相机在世界坐标系的位置
};

RPG_COMMON_MAIN
{
    CHECK(!FLAGS_abs_map.empty());
    CHECK(!FLAGS_abs_trace_dir.empty());

    std::cout << "This is to test the best view algorithm "
                 "\n";

    std::cout << "Experiment parameters:\n"
              << "- abs_map: " << FLAGS_abs_map << std::endl
              << "- abs_trace_dir: " << FLAGS_abs_trace_dir << std::endl
              << "- xrange: " << FLAGS_xrange << std::endl
              << "- yrange: " << FLAGS_yrange << std::endl
              << "- zrange: " << FLAGS_zrange << std::endl;


    vi_utils::Map map;
    map.load(FLAGS_abs_map, std::string());
    std::cerr << "Loaded " << map.n_points_ << " map points." << std::endl;

    // 以原点为中心，xrange/2作为半径，采样7个点
    std::cerr << "Random sampling voxel positions..." << std::endl;
    rpg::PositionVec vox_pos;
    double radius = FLAGS_yrange/3;
    vox_pos.emplace_back(0, -radius, 0);
    vox_pos.emplace_back(radius/2, -radius/2*sqrt(3), 0);
    vox_pos.emplace_back(radius/2*sqrt(3), -radius/2, 0);
    vox_pos.emplace_back(radius, 0, 0);
    vox_pos.emplace_back(radius/2*sqrt(3), radius/2, 0);
    vox_pos.emplace_back(radius/2, radius/2*sqrt(3), 0);
    vox_pos.emplace_back(0, radius, 0);
    

    // 这里偷个懒：
    // 使用kTrace表示优化得到的max_info视角方向
    // 使用kMinEig表示运动方向
    // 使用kDet表示最优视角方向
    std::vector<InfoMetricType> test_types{ InfoMetricType::kMinEig,
                                          InfoMetricType::kDet,
                                          InfoMetricType::kTrace };
    
    std::map<InfoMetricType, OptimOrientRes> res_exact;
    for (const InfoMetricType v : test_types)
    {
        const std::string nm = kInfoMetricNames[v];
        res_exact.insert({v, OptimOrientRes(vox_pos.size(), nm + "_ceres_optimization")});
    }

    std::cerr << ">>> Determine the best orientation from exact "
               "information..." << std::endl;

    // 确定运动方向
    res_exact[InfoMetricType::kMinEig].optim_views_[0] = Eigen::Vector3d(1,0,0);
    res_exact[InfoMetricType::kMinEig].optim_views_[1] = Eigen::Vector3d(sqrt(3)/2,1/2.0,0);
    res_exact[InfoMetricType::kMinEig].optim_views_[2] = Eigen::Vector3d(1/2.0,sqrt(3)/2,0);
    res_exact[InfoMetricType::kMinEig].optim_views_[3] = Eigen::Vector3d(0,1,0);
    res_exact[InfoMetricType::kMinEig].optim_views_[4] = Eigen::Vector3d(-1/2.0,sqrt(3)/2,0);
    res_exact[InfoMetricType::kMinEig].optim_views_[5] = Eigen::Vector3d(-sqrt(3)/2,1/2.0,0);
    res_exact[InfoMetricType::kMinEig].optim_views_[6] = Eigen::Vector3d(-1,0,0);

    //  用第一个点的朝向来初始化机器人的地图点
    std::map<size_t, Eigen::Vector3d> vis_points;
    const size_t kNPts = static_cast<size_t>(map.points_.cols());
    for (size_t pt_i = 0; pt_i < kNPts; pt_i++)
    {
        Eigen::Vector3d pw = map.points_.col(static_cast<int>(pt_i));
        double w = std::sqrt((pw.x()-vox_pos[0].x())*(pw.x()-vox_pos[0].x()) + (pw.y()-vox_pos[0].y())*
        (pw.y()-vox_pos[0].y()) + (pw.z()-vox_pos[0].z()) * (pw.z()-vox_pos[0].z()));
        double cos_val = ((pw.x()-vox_pos[0].x())*res_exact[InfoMetricType::kMinEig].optim_views_[0].x()
                        + (pw.y()-vox_pos[0].y())*res_exact[InfoMetricType::kMinEig].optim_views_[0].y()
                        + (pw.z()-vox_pos[0].z())*res_exact[InfoMetricType::kMinEig].optim_views_[0].z())/(w);
        double theta = acos(cos_val) * 180/M_PI;
        std::cerr << "theta = " << theta << std::endl;
        if (theta < 45)
            vis_points.insert({pt_i, pw});
    }
    std::ofstream map_file("vis_point0.txt");
    for(auto iter = vis_points.begin(); iter!=vis_points.end(); iter++)
    {
        map_file << iter->second.x() << " " << iter->second.y() << " " << iter->second.z() << std::endl;
    }
    map_file.close();

    for (int i = 0; i < vox_pos.size(); i++)
    {
        // 求解最大信息视角
        Eigen::Vector3d& twc_i = vox_pos[i];
        // 使用ceres优化来进行求解
        // 待优化变量是相机的视角
        // 将优化求的结果和采样的结果进行对比，时间和精度
        ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
        double w_c_view_para[4];
        // 设置初始值
        w_c_view_para[0] =  0.653;
        w_c_view_para[1] = -0.653;
        w_c_view_para[2] = 0.271;
        w_c_view_para[3] = -0.271;

        ceres::Problem problem;
        problem.AddParameterBlock(w_c_view_para, 4, local_parameterization);
        rpg::Rotation rot;
        rot.setIdentity();
        rpg::Pose Twc_identity(rot, twc_i); // J和旋转无关，优化过程中是常数
        for (auto iter = vis_points.begin(); iter!=vis_points.end(); iter++)
        {
            Eigen::Vector3d pw = iter->second;
            // Eigen::Vector3d pw(1,1,1);
            rpg::Matrix36 J =
                vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Twc_identity.inverse());
            rpg::Matrix66 curr_info;
            curr_info = J.transpose() * J;
            double info = act_map::getInfoMetric(curr_info, InfoMetricType::kTrace);
            // std::cout << "twc_i = " << twc_i << std::endl << "pw = " << pw << std::endl << " pt_i = " << pt_i << " info = " << info << std::endl;
            ceres::CostFunction* cost_function = CostFunctor::Create(info, pw, twc_i);
            problem.AddResidualBlock(cost_function, NULL, w_c_view_para);
        }
        ceres::Solver::Options options;
        options.num_threads = 16;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // std::cout << summary.FullReport() << "\n";
        Eigen::Quaterniond optim_view(w_c_view_para[0], w_c_view_para[1], w_c_view_para[2], w_c_view_para[3]);
        // std::cout << "final result  = " << optim_view * Eigen::Vector3d(0,0,1);
        res_exact[InfoMetricType::kTrace].optim_views_[i] = optim_view * Eigen::Vector3d(0,0,1);
        res_exact[InfoMetricType::kTrace].optim_vals_[i] = summary.final_cost;
    
        // 求解最优视角
        // 计算得到best_view
        if ( i == 0)
        {
            res_exact[InfoMetricType::kDet].optim_views_[i] = res_exact[InfoMetricType::kTrace].optim_views_[i];
            res_exact[InfoMetricType::kDet].optim_vals_[i] = res_exact[InfoMetricType::kTrace].optim_vals_[i];
            continue;
        }

        double max_info_view_value = res_exact[InfoMetricType::kTrace].optim_vals_[i]; 
        double curr_view_value = res_exact[InfoMetricType::kDet].optim_vals_[i-1];
        double alpha = curr_view_value/(curr_view_value + max_info_view_value);
        res_exact[InfoMetricType::kDet].optim_views_[i] = (alpha*res_exact[InfoMetricType::kMinEig].optim_views_[i] + (1-alpha)*res_exact[InfoMetricType::kTrace].optim_views_[i]).normalized();
        

        // 确定在这个位姿上可以看到哪些点，写入到map_points_i_view.txt中
        const size_t kNPts = static_cast<size_t>(map.points_.cols());
        for (size_t pt_i = 0; pt_i < kNPts; pt_i++)
        {
            Eigen::Vector3d pw = map.points_.col(static_cast<int>(pt_i));
            double w = sqrt((pw.x()-vox_pos[i].x())*(pw.x()-vox_pos[i].x()) + (pw.y()-vox_pos[i].y())*
            (pw.y()-vox_pos[i].y()) + (pw.z()-vox_pos[i].z()) * (pw.z()-vox_pos[i].z()));
            double cos_val = ((pw.x()-vox_pos[i].x())*res_exact[InfoMetricType::kDet].optim_views_[i].x()
                            + (pw.y()-vox_pos[i].y())*res_exact[InfoMetricType::kDet].optim_views_[i].y()
                            + (pw.z()-vox_pos[i].z())*res_exact[InfoMetricType::kDet].optim_views_[i].z())/w;
            double theta = acos(cos_val) * 180/M_PI;
            if (theta < 45 && vis_points.find(pt_i) == vis_points.end())
            {
                vis_points.insert({pt_i, pw});
            }
        }
        
        // 更新best view value
        double best_view_value = 0;
        for (auto iter = vis_points.begin(); iter!=vis_points.end(); iter++)
        {
            Eigen::Vector3d pw = iter->second;
            // Eigen::Vector3d pw(1,1,1);
            rpg::Matrix36 J =
                vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Twc_identity.inverse());
            rpg::Matrix66 curr_info;
            curr_info = J.transpose() * J;
            best_view_value += act_map::getInfoMetric(curr_info, InfoMetricType::kTrace);
        }        
        res_exact[InfoMetricType::kDet].optim_vals_[i] = best_view_value;

        // save map points
        std::string file_name = std::string("vis_point") + std::to_string(i) + ".txt";
        std::ofstream map_file(file_name);
        for(auto iter = vis_points.begin(); iter!=vis_points.end(); iter++)
        {
            map_file << iter->second.x() << " " << iter->second.y() << " " << iter->second.z() << std::endl;
        }
        map_file.close();
    }

    std::cerr << "Saving results...";
    Eigen::MatrixX3d vox_pos_mat;
    VecKVecToEigenXK(vox_pos, &vox_pos_mat);
    rpg::save(FLAGS_abs_trace_dir + "/vox_pos.txt", vox_pos_mat);
    for (const InfoMetricType mtype : test_types)
    {
        res_exact[mtype].save(FLAGS_abs_trace_dir);
    }
}
