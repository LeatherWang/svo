// This file is part of SVO - Semi-direct Visual Odometry.
// 图优化（光束法平差最优化重投影误差）

#include <stdexcept>
#include <pose_optimizer.h>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <vikit/robust_cost.h>
#include <vikit/math_utils.h>

namespace svo {
namespace pose_optimizer {

//pose_optimizer::optimizeGaussNewton(
//Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
//new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
void optimizeGaussNewton(
        const double reproj_thresh, //reproj_thresh: 2
        const size_t n_iter,
        const bool verbose,         //false
        FramePtr& frame,            //new_frame_
        double& estimated_scale,
        double& error_init,
        double& error_final,
        size_t& num_obs)
{
    // init
    double chi2(0.0);
    vector<double> chi2_vec_init, chi2_vec_final;
    // 参考: http://en.wikipedia.org/wiki/Redescending_M-estimator
    vk::robust_cost::TukeyWeightFunction weight_function; /** @attention 鲁棒核函数*/
    SE3 T_old(frame->T_f_w_);
    Matrix6d A;
    Vector6d b;

    // compute the scale of the error for robust estimation
    std::vector<float> errors;
    errors.reserve(frame->fts_.size()); //预留大小
    for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
        if((*it)->point == NULL)
            continue;
        Vector2d e = vk::project2d((*it)->f)
                - vk::project2d(frame->T_f_w_ * (*it)->point->pos_); //!@attention 这里project2d函数是转换到归一化坐标系，而不是映射到像素坐标系
        e *= 1.0 / (1<<(*it)->level); //转换到feature所在的层
        errors.push_back(e.norm());
    }
    if(errors.empty())
        return;
    vk::robust_cost::MADScaleEstimator scale_estimator;
    estimated_scale = scale_estimator.compute(errors); /** @todo */

    num_obs = errors.size();
    chi2_vec_init.reserve(num_obs);
    chi2_vec_final.reserve(num_obs);
    double scale = estimated_scale;

    // 迭代优化10次
    for(size_t iter=0; iter<n_iter; iter++)
    {
        // overwrite scale
        if(iter == 5)
            scale = 0.85/frame->cam_->errorMultiplier2();

        b.setZero();
        A.setZero();
        double new_chi2(0.0);

        // compute residual
        for(auto it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
        {
            if((*it)->point == NULL)
                continue;
            Matrix26d J;
            Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
            Frame::jacobian_xyz2uv(xyz_f, J);
            Vector2d e = vk::project2d((*it)->f) - vk::project2d(xyz_f);
            double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
            e *= sqrt_inv_cov;
            if(iter == 0)
                chi2_vec_init.push_back(e.squaredNorm()); // just for debug

            J *= sqrt_inv_cov; //雅可比矩阵<转换到feature所在的层>

            double weight = weight_function.value(e.norm()/scale); //鲁棒系数
            A.noalias() += J.transpose()*J*weight;
            b.noalias() -= J.transpose()*e*weight;
            new_chi2 += e.squaredNorm()*weight; //chi2
        }

        // solve linear system
        const Vector6d dT(A.ldlt().solve(b));

        // check if error increased
        //!@attention 当chi2误差增加，则回滚为上一次迭代后的值
        if((iter > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dT[0]))
        {
            if(verbose)
                std::cout << "it " << iter
                          << "\t FAILURE \t new_chi2 = " << new_chi2 << std::endl;
            frame->T_f_w_ = T_old; // roll-back
            break;
        }

        // update the model
        SE3 T_new = SE3::exp(dT)*frame->T_f_w_;
        T_old = frame->T_f_w_;
        frame->T_f_w_ = T_new;
        chi2 = new_chi2;
        if(verbose)
            std::cout << "it " << iter
                      << "\t Success \t new_chi2 = " << new_chi2
                      << "\t norm(dT) = " << vk::norm_max(dT) << std::endl;

        // stop when converged
        if(vk::norm_max(dT) <= EPS)
            break;
    }

    // Set covariance as inverse information matrix. Optimistic estimator!
    //!@attention 优化结束后，计算位姿的协方差
    // 假设，在对应的层数上，测量值的协方差都为1个像素，即测量值满足方差为1的高斯分布
    // 参考: http://www.cnblogs.com/ilekoaiq/p/8659631.html
    const double pixel_variance=1.0;
    frame->Cov_ = pixel_variance*(A*std::pow(frame->cam_->errorMultiplier2(),2)).inverse();

    // Remove Measurements with too large reprojection error//
    // 移除重映射误差太大的测量
    double reproj_thresh_scaled = reproj_thresh / frame->cam_->errorMultiplier2(); /** @attention reproj_thresh=2，
                                                                                         即表示两个像素的误差，除以f转换到归一化坐标系*/
    size_t n_deleted_refs = 0;
    for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
        if((*it)->point == NULL)
            continue;
        Vector2d e = vk::project2d((*it)->f) - vk::project2d(frame->T_f_w_ * (*it)->point->pos_); /** @attention 这里project2d函数是转换到
                                                                                                        归一化坐标系，而不是映射到像素坐标系*/
        double sqrt_inv_cov = 1.0 / (1<<(*it)->level);
        e *= sqrt_inv_cov;
        chi2_vec_final.push_back(e.squaredNorm());

        // 如果有些点的的重投影误差，映射到第0层上，模大于2个像素的话，则把这个特征点指向地图点的指针赋值为空
        if(e.norm() > reproj_thresh_scaled) /** @done 见上*/
        {
            // we don't need to delete a reference in the point since it was not created yet
            (*it)->point = NULL;
            ++n_deleted_refs;
        }
    }

    error_init=0.0;
    error_final=0.0;
    if(!chi2_vec_init.empty())
        error_init = sqrt(vk::getMedian(chi2_vec_init))*frame->cam_->errorMultiplier2();
    if(!chi2_vec_final.empty())
        error_final = sqrt(vk::getMedian(chi2_vec_final))*frame->cam_->errorMultiplier2();

    estimated_scale *= frame->cam_->errorMultiplier2();
    if(verbose)
        std::cout << "n deleted obs = " << n_deleted_refs
                  << "\t scale = " << estimated_scale
                  << "\t error init = " << error_init
                  << "\t error end = " << error_final << std::endl;
    num_obs -= n_deleted_refs;
}

} // namespace pose_optimizer
} // namespace svo
