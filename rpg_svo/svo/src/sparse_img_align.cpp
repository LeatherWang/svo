// This file is part of SVO - Semi-direct Visual Odometry.
// 	直接法优化位姿（最小化光度误差）

#include <algorithm>
#include <sparse_img_align.h>
#include <frame.h>
#include <feature.h>
#include <config.h>
#include <point.h>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>

namespace svo {

SparseImgAlign::SparseImgAlign(
        int max_level, int min_level, int n_iter,
        Method method, bool display, bool verbose) :
    display_(display),
    max_level_(max_level),
    min_level_(min_level)
{
    n_iter_ = n_iter;
    n_iter_init_ = n_iter_;
    method_ = method;
    verbose_ = verbose;
    eps_ = 0.000001;
}

size_t SparseImgAlign::run(FramePtr ref_frame, FramePtr cur_frame)
{
    reset();

    if(ref_frame->fts_.empty())
    {
        SVO_WARN_STREAM("SparseImgAlign: no features to track!");
        return 0;
    }

    ref_frame_ = ref_frame;
    cur_frame_ = cur_frame;
    ref_patch_cache_ = cv::Mat(ref_frame_->fts_.size(), patch_area_, CV_32F); //n行乘16列，n表示参考帧上面的特征点的个数，16代表要取的图块的像素数量
    jacobian_cache_.resize(Eigen::NoChange, ref_patch_cache_.rows*patch_area_); //6行乘n*16列的矩阵，代表图块上的每个像素点误差对相机位姿的雅克比
    visible_fts_.resize(ref_patch_cache_.rows, false); // TODO: should it be reset at each level?

    SE3 T_cur_from_ref(cur_frame_->T_f_w_ * ref_frame_->T_f_w_.inverse()); //当cur_frame_->T_f_w_=ref_frame_->T_f_w_时，T_cur_from_ref为单位矩阵

    // 对位于`min_level_`与`max_level_`之间的每一层进行遍历，优化
    for(level_=max_level_; level_>=min_level_; --level_)
    {
        mu_ = 0.1;
        jacobian_cache_.setZero();
        have_ref_patch_cache_ = false;
        if(verbose_)
            printf("\nPYRAMID LEVEL %i\n---------------\n", level_);
        optimize(T_cur_from_ref); /** @attention 找到了^_^ */
    }
    cur_frame_->T_f_w_ = T_cur_from_ref * ref_frame_->T_f_w_;

    return n_meas_/patch_area_;
}

Matrix<double, 6, 6> SparseImgAlign::getFisherInformation()
{
    double sigma_i_sq = 5e-4*255*255; // image noise
    Matrix<double,6,6> I = H_/sigma_i_sq;
    return I;
}

/**
 * @brief 预计算参考帧的图像块，包括feature的可见性、周围4x4范围亚像素坐标、雅可比矩阵
 */
void SparseImgAlign::precomputeReferencePatches()
{
    const int border = patch_halfsize_+1; //+1,=3
    const cv::Mat& ref_img = ref_frame_->img_pyr_.at(level_); //取当前帧对应的金字塔层
    const int stride = ref_img.cols;
    const float scale = 1.0f/(1<<level_); //1左移level_位
    const Vector3d ref_pos = ref_frame_->pos();
    const double focal_length = ref_frame_->cam_->errorMultiplier2();
    size_t feature_counter = 0;
    std::vector<bool>::iterator visiblity_it = visible_fts_.begin();

    // 遍历参考帧中的每一个feature
    for(auto it=ref_frame_->fts_.begin(), ite=ref_frame_->fts_.end();
        it!=ite; ++it, ++feature_counter, ++visiblity_it)
    {
        // check if reference with patch size is within image
    /*【步骤1】:检查这个feature算上patch size后，然后尺度化，是否还在图像范围内*/
        const float u_ref = (*it)->px[0]*scale; //根据金字塔层，尺度化，float类型
        const float v_ref = (*it)->px[1]*scale;
        const int u_ref_i = floorf(u_ref); //向下取整
        const int v_ref_i = floorf(v_ref);
        if((*it)->point == NULL || u_ref_i-border < 0 || v_ref_i-border < 0 ||    // feature距离边界应大于等于3个像素
                u_ref_i+border >= ref_img.cols || v_ref_i+border >= ref_img.rows) // (v_ref_i,u_ref_i)位置: [x x x (.) x]
            continue;

        *visiblity_it = true;

        // cannot just take the 3d points coordinate because of the reprojection errors in the reference image!!!
    /*【步骤2】:不能使用3D点坐标，这是因为重投影误差是在参考帧上面*/
        const double depth(((*it)->point->pos_ - ref_pos).norm()); //计算深度(光心与3D点之间的直线距离)，非z轴坐标
        const Vector3d xyz_ref((*it)->f*depth); /** @done 使用的是不是Z轴坐标而是深度恢复出来的是什么，因为(*it)->f是单位向量*/

        // evaluate projection jacobian
    /*【步骤3】:计算投影雅可比矩阵，2x6维*/
        Matrix<double,2,6> frame_jac;
        Frame::jacobian_xyz2uv(xyz_ref, frame_jac);

        // compute bilateral interpolation weights for reference image
    /*【步骤4】:对参考帧计算双线性插权重*/
        const float subpix_u_ref = u_ref-u_ref_i;
        const float subpix_v_ref = v_ref-v_ref_i;
        const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
        const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
        const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
        const float w_ref_br = subpix_u_ref * subpix_v_ref;
        size_t pixel_counter = 0;
        float* cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter; //取地址
        // 遍历行
        for(int y=0; y<patch_size_; ++y)
        {
            // (v_ref_i,u_ref_i)位置: [x x x (.) x]
            uint8_t* ref_img_ptr = (uint8_t*) ref_img.data + (v_ref_i-patch_halfsize_+y)*stride //行
                                                           + (u_ref_i-patch_halfsize_); //列
            // 遍历列
            for(int x=0; x<patch_size_; ++x, ++ref_img_ptr, ++cache_ptr, ++pixel_counter)
            {
                // precompute interpolated reference patch color
    /*【步骤4.1】:使用相邻四个像素，计算双线性差值*/
                *cache_ptr = w_ref_tl*ref_img_ptr[0] + w_ref_tr*ref_img_ptr[1] +
                             w_ref_bl*ref_img_ptr[stride] + w_ref_br*ref_img_ptr[stride+1];

                // we use the inverse compositional: thereby we can take the gradient always at the same position
                // get gradient of warped image (~gradient at warped position)
    /*【步骤4.2】:使用`inverse compositional`，因此我们可以在相同的位置计算梯度*/
                // 使用<4.1步骤插值位置>的<前后的>插值计算当前x方向的梯度
                float dx = 0.5f * ((w_ref_tl*ref_img_ptr[1] + w_ref_tr*ref_img_ptr[2] +
                                                w_ref_bl*ref_img_ptr[stride+1] + w_ref_br*ref_img_ptr[stride+2])
                                    -(w_ref_tl*ref_img_ptr[-1] + w_ref_tr*ref_img_ptr[0] +
                                                w_ref_bl*ref_img_ptr[stride-1] + w_ref_br*ref_img_ptr[stride]));
                // 使用<4.1步骤插值位置>的<上下的>插值计算当前y方向的梯度
                float dy = 0.5f * ((w_ref_tl*ref_img_ptr[stride] + w_ref_tr*ref_img_ptr[1+stride] +
                                                w_ref_bl*ref_img_ptr[stride*2] + w_ref_br*ref_img_ptr[stride*2+1])
                                    -(w_ref_tl*ref_img_ptr[-stride] + w_ref_tr*ref_img_ptr[1-stride] +
                                                w_ref_bl*ref_img_ptr[0] + w_ref_br*ref_img_ptr[1]));
                // cache the jacobian
    /*【步骤4.3】:存储雅可比矩阵*/
                //! @attention 参考论文章节: Sparse Model-based Image Alignment
                // 分别给每一列雅可比矩阵赋值，论文公式12
                jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter) =
                        (dx*frame_jac.row(0) + dy*frame_jac.row(1))*(focal_length/(1<<level_));// 根据特征点所在的层数算出focal_length，其中fx=fy

            }
        }
    }
    have_ref_patch_cache_ = true;
}


// computeResiduals(model, false, true);
// model: SE3
double SparseImgAlign::computeResiduals(
        const SE3& T_cur_from_ref,
        bool linearize_system,
        bool compute_weight_scale)
{
    // Warp the (cur)rent image such that it aligns with the (ref)erence image
    /*【步骤1】: 取当前帧的不同层*/
    const cv::Mat& cur_img = cur_frame_->img_pyr_.at(level_);

    if(linearize_system && display_)
        resimg_ = cv::Mat(cur_img.size(), CV_32F, cv::Scalar(0));

    /*【步骤2】: 预计算参考帧的图像块，包括feature的可见性、周围4x4范围亚像素坐标、雅可比矩阵*/
    if(have_ref_patch_cache_ == false)
        precomputeReferencePatches();

    // compute the weights on the first iteration
    std::vector<float> errors;
    if(compute_weight_scale)
        errors.reserve(visible_fts_.size());
    const int stride = cur_img.cols;
    const int border = patch_halfsize_+1;
    const float scale = 1.0f/(1<<level_);
    const Vector3d ref_pos(ref_frame_->pos());
    float chi2 = 0.0;
    size_t feature_counter = 0; // is used to compute the index of the cached jacobian\

    std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
    /*【步骤3】: 遍历<参考帧每一个关键点>用于与对应的<当前帧相对应的关键点>进行计算残差*/
    for(auto it=ref_frame_->fts_.begin(); it!=ref_frame_->fts_.end();
        ++it, ++feature_counter, ++visiblity_it)
    {
        // check if feature is within image
        if(!*visiblity_it)
            continue;

    /*【步骤3.1】: 计算feature在当前帧(对应的层)下粗略(未优化之前)坐标*/
        // compute pixel location in cur img
        const double depth = ((*it)->point->pos_ - ref_pos).norm();
        const Vector3d xyz_ref((*it)->f*depth); //3D点在参考帧下的坐标
        const Vector3d xyz_cur(T_cur_from_ref * xyz_ref); //将参考帧下的3D点坐标转换到当前帧下
        /** @attention 除以尺度，表示在该金字塔层对应图像下的像素坐标(减半采样)*/
        const Vector2f uv_cur_pyr(cur_frame_->cam_->world2cam(xyz_cur).cast<float>() * scale);
        const float u_cur = uv_cur_pyr[0];
        const float v_cur = uv_cur_pyr[1];
        const int u_cur_i = floorf(u_cur);
        const int v_cur_i = floorf(v_cur);

        // check if projection is within the image
        if(u_cur_i < 0 || v_cur_i < 0 || u_cur_i-border < 0 || v_cur_i-border < 0 ||
                u_cur_i+border >= cur_img.cols || v_cur_i+border >= cur_img.rows)
            continue;

        // compute bilateral interpolation weights for the current image
        const float subpix_u_cur = u_cur-u_cur_i;
        const float subpix_v_cur = v_cur-v_cur_i;
        const float w_cur_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
        const float w_cur_tr = subpix_u_cur * (1.0-subpix_v_cur);
        const float w_cur_bl = (1.0-subpix_u_cur) * subpix_v_cur;
        const float w_cur_br = subpix_u_cur * subpix_v_cur;
        float* ref_patch_cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
        size_t pixel_counter = 0; // is used to compute the index of the cached jacobian
    /*【步骤3.2】: 遍历像素块中每一个像素*/
        for(int y=0; y<patch_size_; ++y)
        {
            // 取当前帧像素坐标指针
            uint8_t* cur_img_ptr = (uint8_t*) cur_img.data + (v_cur_i+y-patch_halfsize_)*stride + (u_cur_i-patch_halfsize_);

            for(int x=0; x<patch_size_; ++x, ++pixel_counter, ++cur_img_ptr, ++ref_patch_cache_ptr)
            {
                // compute residual
    /*【步骤3.2.1】: 对当前双线性插值*/
                const float intensity_cur = w_cur_tl*cur_img_ptr[0] + w_cur_tr*cur_img_ptr[1] +
                                            w_cur_bl*cur_img_ptr[stride] + w_cur_br*cur_img_ptr[stride+1];
    /*【步骤3.2.2】: 计算像素块对应位置灰度残差*/
                const float res = intensity_cur - (*ref_patch_cache_ptr);

                // used to compute scale for robust cost
                if(compute_weight_scale)
                    errors.push_back(fabsf(res));

                // robustification
                // 鲁棒核函数
                float weight = 1.0;
                if(use_weights_) { //默认: false
                    weight = weight_function_->value(res/scale_); /** @todo */
                }

                chi2 += res*res*weight;
                n_meas_++; //测量数目

                // 如果是线性系统
                //! 为了节省存储空间，提前就转换成了H矩阵
                // 注意这里与论文的区别，没有使用16x6的雅可比，而是转换成1x6的雅可比
                if(linearize_system)
                {
                    // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
                    const Vector6d J(jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter));
                    H_.noalias() += J*J.transpose()*weight; //加上权重
                    Jres_.noalias() -= J*res*weight;
                    if(display_)
                        resimg_.at<float>((int) v_cur+y-patch_halfsize_, (int) u_cur+x-patch_halfsize_) = res/255.0;
                }
            }
        }
    }

    // compute the weights on the first iteration
    //!@todo 第一次迭代结果，计算下scale??
    if(compute_weight_scale && iter_ == 0)
        scale_ = scale_estimator_->compute(errors);

    /*【步骤4】: 返回的是什么鬼*/
    return chi2/n_meas_;
}

// 线性求解器，参考论文公式12
int SparseImgAlign::solve()
{
    x_ = H_.ldlt().solve(Jres_);
    if((bool) std::isnan((double) x_[0]))
        return 0;
    return 1;
}

// 更新，参考论文公式9
void SparseImgAlign::update(
        const ModelType& T_curold_from_ref,
        ModelType& T_curnew_from_ref)
{
    T_curnew_from_ref =  T_curold_from_ref * SE3::exp(-x_); //!@attention 为了加快运算，T(x)^(-1)近似等于T(-x)
}

void SparseImgAlign::startIteration()
{}

void SparseImgAlign::finishIteration()
{
    if(display_)
    {
        cv::namedWindow("residuals", CV_WINDOW_AUTOSIZE);
        cv::imshow("residuals", resimg_*10);
        cv::waitKey(0);
    }
}

} // namespace svo

