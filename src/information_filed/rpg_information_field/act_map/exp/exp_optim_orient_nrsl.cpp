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

    std::cout << "This is to test whether the map can be used to select motion. "
                 "In this case, we select the optimal orientation at each point. "
                 "Samples on SO3 are checked to find the optimal orientation."
                 "Apply exp function as viscore, and use gradient descent as optimization methods."
                 "\n";

    std::cout << "Experiment parameters:\n"
              << "- abs_map: " << FLAGS_abs_map << std::endl
              << "- abs_trace_dir: " << FLAGS_abs_trace_dir << std::endl
              << "- xrange: " << FLAGS_xrange << std::endl
              << "- yrange: " << FLAGS_yrange << std::endl
              << "- zrange: " << FLAGS_zrange << std::endl
              << "- vox_res: " << FLAGS_vox_res << std::endl
              << "- angel_res_deg: " << FLAGS_angle_res_deg << std::endl
              << "- check_ratio: " << FLAGS_check_ratio << std::endl;

    std::random_device rd;
    std::mt19937 rng(rd());

    rpg::Timer timer;

    vi_utils::Map map;
    map.load(FLAGS_abs_map, std::string());
    std::cerr << "Loaded " << map.n_points_ << " map points." << std::endl;

    std::vector<double> xvalues, yvalues, zvalues;
    rpg::PositionVec uniform_grid;
    utils::generateUniformPointsWithin(
        FLAGS_vox_res, FLAGS_xrange, FLAGS_yrange, FLAGS_zrange, &uniform_grid);
    const size_t kNPts = uniform_grid.size();
    const size_t kNCompute = static_cast<size_t>(FLAGS_check_ratio * kNPts);
    std::uniform_int_distribution<size_t> pts_uni(0, kNPts - 1);
    std::cerr << "Random sampling voxel positions..." << std::endl;
    rpg::PositionVec vox_pos;
    for (size_t i = 0; i < kNCompute; i++)
    {
        vox_pos.emplace_back(uniform_grid[pts_uni(rng)]);
    }

    std::vector<InfoMetricType> test_types{InfoMetricType::kTrace};
    
    std::map<InfoMetricType, OptimOrientRes> res_exact;
    for (const InfoMetricType v : test_types)
    {
        const std::string nm = kInfoMetricNames[v];
        res_exact.insert({v, OptimOrientRes(kNCompute, nm + "_ceres_optimization")});
    }

    std::cerr << ">>> Determine the optimal orientation from exact "
               "information..." << std::endl;
    std::map<InfoMetricType, double> time_exact;
    for (const InfoMetricType v : test_types)
    {
        time_exact.insert({v, 0});
    }
    double cnt_r = 0.0;
    for (int i = 0; i < kNCompute; i++)
    {
        if (i > cnt_r * kNCompute)
        {
            std::cout << "." << std::flush;
            cnt_r += 0.1;
        }
        Eigen::Vector3d& twc_i = vox_pos[i];
        // Eigen::Vector3d twc_i(0,0,0);
        for (const InfoMetricType v : test_types)
        {
            timer.start();
            // 使用ceres优化来进行求解
            // 待优化变量是相机的视角
            // 将优化求的结果和采样的结果进行对比，时间和精度
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
            double w_c_view_para[4];
            w_c_view_para[0] =  0.653;
            w_c_view_para[1] = -0.653;
            w_c_view_para[2] = 0.271;
            w_c_view_para[3] = -0.271;

            ceres::Problem problem;
            problem.AddParameterBlock(w_c_view_para, 4, local_parameterization);
            rpg::Rotation rot;
            rot.setIdentity();
            rpg::Pose Twc_identity(rot, twc_i); // J和旋转无关，优化过程中是常数
            const size_t kNPts = static_cast<size_t>(map.points_.cols());
            for (size_t pt_i = 0; pt_i < kNPts; pt_i++)
            {
                Eigen::Vector3d pw = map.points_.col(static_cast<int>(pt_i));
                // Eigen::Vector3d pw(1,1,1);
                rpg::Matrix36 J =
                    vi_utils::jacobians::dBearing_dIMUPoseGlobal(pw, Twc_identity.inverse());
                rpg::Matrix66 curr_info;
                curr_info = J.transpose() * J;
                double info = act_map::getInfoMetric(curr_info, v);
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
            res_exact[v].optim_views_[i] = optim_view * Eigen::Vector3d(0,0,1);
            res_exact[v].optim_vals_[i] = summary.final_cost;
            time_exact[v] += timer.stop();
        }
    }
    std::cerr << "Timing (sec):\n"
            << "MinEig: " << time_exact[InfoMetricType::kMinEig] << ", Trace - "
            << time_exact[InfoMetricType::kTrace] << ", Det - "
            << time_exact[InfoMetricType::kDet];

    std::cerr << "Saving results...";
    Eigen::MatrixX3d vox_pos_mat;
    VecKVecToEigenXK(vox_pos, &vox_pos_mat);
    rpg::save(FLAGS_abs_trace_dir + "/vox_pos.txt", vox_pos_mat);
    for (const InfoMetricType mtype : test_types)
    {
        res_exact[mtype].save(FLAGS_abs_trace_dir);
    }
}
