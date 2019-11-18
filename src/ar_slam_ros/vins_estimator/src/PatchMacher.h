/**
 * @file PatchMatcher.h
 * @author Song Jin (songjinxs@163.com)
 * @brief patch matcher
 * @version 0.1
 * @date 2019-11-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace Eigen;
class PatchMatcher
{
    PatchMatcher();

    /**
     * @brief 
     * 
     * @param p_w 
     * @return size_t 
     */
    size_t reprojectPoints(std::vector<Eigen::Vector3d> &p_w);

    /**
     * @brief 
     * 
     * @param tracked_pts 
     * @param added_pts 
     * @return size_t 
     */
    size_t findCandidataPoints(std::vector<cv::Point2f> tracked_pts, std::vector<cv::Point2f> added_pts);

    bool align2D(
        const cv::Mat &cur_img,
        uint8_t *ref_patch_with_border,
        uint8_t *ref_patch,
        const int n_iter,
        Eigen::Vector2d &cur_px_estimate,
        bool no_simd)
    {
#ifdef __ARM_NEON__
        if (!no_simd)
            return align2D_NEON(cur_img, ref_patch_with_border, ref_patch, n_iter, cur_px_estimate);
#endif

        const int halfpatch_size_ = 4;
        const int patch_size_ = 8;
        const int patch_area_ = 64;
        bool converged = false;

        // compute derivative of template and prepare inverse compositional
        float __attribute__((__aligned__(16))) ref_patch_dx[patch_area_];
        float __attribute__((__aligned__(16))) ref_patch_dy[patch_area_];
        Eigen::Matrix3f H;
        H.setZero();

        // compute gradient and hessian
        const int ref_step = patch_size_ + 2;
        float *it_dx = ref_patch_dx;
        float *it_dy = ref_patch_dy;
        for (int y = 0; y < patch_size_; ++y)
        {
            uint8_t *it = ref_patch_with_border + (y + 1) * ref_step + 1;
            for (int x = 0; x < patch_size_; ++x, ++it, ++it_dx, ++it_dy)
            {
                Eigen::Vector3f J;
                J[0] = 0.5 * (it[1] - it[-1]);
                J[1] = 0.5 * (it[ref_step] - it[-ref_step]);
                J[2] = 1;
                *it_dx = J[0];
                *it_dy = J[1];
                H += J * J.transpose();
            }
        }
        Eigen::Matrix3f Hinv = H.inverse();
        float mean_diff = 0;

        // Compute pixel location in new image:
        float u = cur_px_estimate.x();
        float v = cur_px_estimate.y();

        // termination condition
        const float min_update_squared = 0.03 * 0.03;
        const int cur_step = cur_img.step.p[0];
        //  float chi2 = 0;
        Eigen::Vector3f update;
        update.setZero();
        for (int iter = 0; iter < n_iter; ++iter)
        {
            int u_r = floor(u);
            int v_r = floor(v);
            if (u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols - halfpatch_size_ || v_r >= cur_img.rows - halfpatch_size_)
                break;

            if (std::isnan(u) || std::isnan(v)) // TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
                return false;

            // compute interpolation weights
            float subpix_x = u - u_r;
            float subpix_y = v - v_r;
            float wTL = (1.0 - subpix_x) * (1.0 - subpix_y);
            float wTR = subpix_x * (1.0 - subpix_y);
            float wBL = (1.0 - subpix_x) * subpix_y;
            float wBR = subpix_x * subpix_y;

            // loop through search_patch, interpolate
            uint8_t *it_ref = ref_patch;
            float *it_ref_dx = ref_patch_dx;
            float *it_ref_dy = ref_patch_dy;
            //    float new_chi2 = 0.0;
            Eigen::Vector3f Jres;
            Jres.setZero();
            for (int y = 0; y < patch_size_; ++y)
            {
                uint8_t *it = (uint8_t *)cur_img.data + (v_r + y - halfpatch_size_) * cur_step + u_r - halfpatch_size_;
                for (int x = 0; x < patch_size_; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy)
                {
                    float search_pixel = wTL * it[0] + wTR * it[1] + wBL * it[cur_step] + wBR * it[cur_step + 1];
                    float res = search_pixel - *it_ref + mean_diff;
                    Jres[0] -= res * (*it_ref_dx);
                    Jres[1] -= res * (*it_ref_dy);
                    Jres[2] -= res;
                    //        new_chi2 += res*res;
                }
            }

            /*
    if(iter > 0 && new_chi2 > chi2)
    {
#if SUBPIX_VERBOSE
      cout << "error increased." << endl;
#endif
      u -= update[0];
      v -= update[1];
      break;
    }
    chi2 = new_chi2;
*/
            update = Hinv * Jres;
            u += update[0];
            v += update[1];
            mean_diff += update[2];

#if SUBPIX_VERBOSE
            cout << "Iter " << iter << ":"
                 << "\t u=" << u << ", v=" << v
                 << "\t update = " << update[0] << ", " << update[1]
//         << "\t new chi2 = " << new_chi2 << endl;
#endif

                if (update[0] * update[0] + update[1] * update[1] < min_update_squared)
            {
#if SUBPIX_VERBOSE
                cout << "converged." << endl;
#endif
                converged = true;
                break;
            }
        }

        cur_px_estimate << u, v;
        return converged;
    }

    void getWarpMatrixAffine(
        const vk::AbstractCamera& cam_ref,
        const vk::AbstractCamera& cam_cur,
        const Vector2d& px_ref,
        const Vector3d& f_ref,patch_size
        const double depth_ref,patch_size
        const SE3& T_cur_ref,patch_size
        const int level_ref,patch_size
        Matrix2d& A_cur_ref)
    {
    // Compute affine warp matrix A_ref_cur
    const int halfpatch_size = 5;
    const Vector3d xyz_ref(f_ref*depth_ref);
    Vector3d xyz_du_ref(cam_ref.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)));
    Vector3d xyz_dv_ref(cam_ref.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)));
    xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];
    xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];
    const Vector2d px_cur(cam_cur.world2cam(T_cur_ref*(xyz_ref)));
    const Vector2d px_du(cam_cur.world2cam(T_cur_ref*(xyz_du_ref)));
    const Vector2d px_dv(cam_cur.world2cam(T_cur_ref*(xyz_dv_ref)));
    A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
    A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
    }

void warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int search_level,
    const int halfpatch_size,
    uint8_t* patch)
{
  const int patch_size = halfpatch_size*2 ;
  const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  if(isnan(A_ref_cur(0,0)))
  {
    printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
    return;
  }

  // Perform the warp on a larger patch.
  uint8_t* patch_ptr = patch;
  const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
  for (int y=0; y<patch_size; ++y)
  {
    for (int x=0; x<patch_size; ++x, ++patch_ptr)
    {
      Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      px_patch *= (1<<search_level);
      const Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
      if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
        *patch_ptr = 0;
      else
        *patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);
    }
  }
}

    Eigen::Isometry3d T_world_camera_curr;  //< 当前相机在世界坐标系下的位姿态
    Eigen::Matrix3d K;                      //< The camera intrinsic
    int image_width;                        //< The image width
    int image_heigh;                        //< The image heigh;
    std::vector<cv::Point2f> projected_pts; //< The projected points
    std::vector<cv::Point2f> candidate_pts; //< Candiate points which wait for matching
};