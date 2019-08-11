// This file is part of SVO - Semi-direct Visual Odometry.
// 重投影匹配与极线搜索

#include <cstdlib>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/patch_score.h>
#include <matcher.h>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <config.h>
#include <feature_alignment.h>

namespace svo {

namespace warp {

void getWarpMatrixAffine(
        const vk::AbstractCamera& cam_ref,
        const vk::AbstractCamera& cam_cur,
        const Vector2d& px_ref,
        const Vector3d& f_ref,
        const double depth_ref,
        const SE3& T_cur_ref,
        const int level_ref,
        Matrix2d& A_cur_ref)
{
    // Compute affine warp matrix A_ref_cur
    const int halfpatch_size = 5;
    const Vector3d xyz_ref(f_ref*depth_ref); //f_ref是单位向量，并非归一化坐标，所以其与真实3D坐标相差一个depth

    /*【步骤1】:然后它的对应的特征点，在它提取层上，取右边的第5个像素位置和下边的第5个像素位置，再映射到第0层*/
    //! 因为px_ref是角点在<提取层>下的坐标转换到<0层>下，所以使用`Vector2d(halfpatch_size,0)*(1<<level_ref)`转换
    Vector3d xyz_du_ref(cam_ref.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref))); //有畸变去畸变
    Vector3d xyz_dv_ref(cam_ref.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)));

    /*【步骤2】:再转换到单位球上，再映射到三维空间中，直到与地图点的模一样的长度(划掉)，其实是深度一样(Z坐标一样)*/
    xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];
    xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];

    /*【步骤3】:然后，再把这3个点映射到当前帧的（有畸变的）图像上*/
    const Vector2d px_cur(cam_cur.world2cam(T_cur_ref*(xyz_ref)));  //有畸变去畸变
    const Vector2d px_du(cam_cur.world2cam(T_cur_ref*(xyz_du_ref)));
    const Vector2d px_dv(cam_cur.world2cam(T_cur_ref*(xyz_dv_ref)));

    /*【步骤4】:根据它们与中心投影点的位置变换，算出了仿射矩阵A_cur_ref*/
    // 推导见笔记
    //! @attention 把参考帧上的图块在它自己对应的层数上，转换到当前帧的第0层上
    A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
    A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
}

int getBestSearchLevel(
        const Matrix2d& A_cur_ref,
        const int max_level)
{
    // Compute patch level in other image
    int search_level = 0;

    // 通过计算仿射矩阵A_cur_ref的行列式，其实就是面积放大率
    double D = A_cur_ref.determinant();
    // 如果面积放大率超过3，就往上一层，面积放大率变为原来的四分之一
    while(D > 3.0 && search_level < max_level)
    {
        search_level += 1;
        D *= 0.25;
    }
    return search_level;
}

void warpAffine(
        const Matrix2d& A_cur_ref,
        const cv::Mat& img_ref, //<提取层>
        const Vector2d& px_ref, //feature<0层>像素坐标
        const int level_ref,
        const int search_level,
        const int halfpatch_size,
        uint8_t* patch)
{
    const int patch_size = halfpatch_size*2 ; //patch_size: 10
    const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>(); //求逆
    if(isnan(A_ref_cur(0,0)))
    {
        printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
        return;
    }

    // Perform the warp on a larger patch.
    uint8_t* patch_ptr = patch;
    const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref); //得到feature在<参考帧><提取层>下的坐标
    for (int y=0; y<patch_size; ++y)
    {
        for (int x=0; x<patch_size; ++x, ++patch_ptr)
        {
            // 在<search_level>取10x10的图块，
            Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
            // 所以需要转换到<0层>
            px_patch *= (1<<search_level);

            // `A_ref_cur*px_patch`: 通过逆仿射矩阵，得到<对应的参考帧>上的<提取层>图像上的（相对中心点的）像素位置
            const Vector2f px(A_ref_cur*px_patch + px_ref_pyr);

            // 插值得到px像素值
            if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
                *patch_ptr = 0;
            else
                *patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]); //feature所在<提取层>
        }
    }
}

} // namespace warp

bool depthFromTriangulation(
        const SE3& T_search_ref,
        const Vector3d& f_ref, //feature对应的在参考帧上的单位向量，不是归一化向量
        const Vector3d& f_cur, //feature对应的在当前帧上的单位向量，不是归一化向量
        double& depth)
{
    Matrix<double,3,2> A;
    A << T_search_ref.rotation_matrix() * f_ref, f_cur;
    const Matrix2d AtA = A.transpose()*A;
    if(AtA.determinant() < 0.000001)
        return false;

    // 参考slambook: p154
    // 或者: http://www.cnblogs.com/ilekoaiq/p/8659631.html
    const Vector2d depth2 = - AtA.inverse()*A.transpose()*T_search_ref.translation();
    depth = fabs(depth2[0]); //这是sr，即3D点在参考帧相机坐标系下的深度
    return true;
}

// 取10x10中心8x8范围图像块
void Matcher::createPatchFromPatchWithBorder()
{
    uint8_t* ref_patch_ptr = patch_;
    for(int y=1; y<patch_size_+1; ++y, ref_patch_ptr += patch_size_)
    {
        uint8_t* ref_patch_border_ptr = patch_with_border_ + y*(patch_size_+2) + 1;
        for(int x=0; x<patch_size_; ++x)
            ref_patch_ptr[x] = ref_patch_border_ptr[x];
    }
}

/** @brief 针对栅格中每一个3D点，寻找直接匹配，优化这一个feature在<当前帧>的位置
 *
 */
bool Matcher::findMatchDirect(
        const Point& pt,        //3D点的世界坐标
        const Frame& cur_frame, //当前帧
        Vector2d& px_cur)       //投影，3D点在当前帧的投影
{
    /*【步骤1】:选出夹角最小的那个关键帧作为参考帧，以及对应的特征点*/
    /** @todo */
    //!注意，这里的这种选夹角的情况，是只适合无人机那样的视角一直朝下的情况的，应该改成ORBSLAM那样，还要再把视角转换到对应的相机坐标系下，再筛选一遍??
    if(!pt.getCloseViewObs(cur_frame.pos(), ref_ftr_))
        return false;

    /*【步骤2】:检查ref_ftr_是否在feature对应图像金字塔内，边界为6*/
    // feature在不同图像金字塔层提取的特征点的坐标都是原始图像像素坐标系
    if(!ref_ftr_->frame->cam_->isInFrame(
                ref_ftr_->px.cast<int>()/(1<<ref_ftr_->level), halfpatch_size_+2, ref_ftr_->level)) /** @todo 边界为6?*/
        return false;

    // warp affine
    /*【步骤3】:计算从参考关键帧到当前帧的仿射变换*/
    // 仿射矩阵A，就是把参考帧上的图块在它自己对应的层数(使用的其是0层的像素坐标)上，转换到当前帧的<第0层>上
    warp::getWarpMatrixAffine(
                *ref_ftr_->frame->cam_, *cur_frame.cam_, ref_ftr_->px, ref_ftr_->f,
                (ref_ftr_->frame->pos() - pt.pos_).norm(), //获取地图点在参考帧上的与光心连线的模
                cur_frame.T_f_w_ * ref_ftr_->frame->T_f_w_.inverse(), ref_ftr_->level, A_cur_ref_);

    /*【步骤4】:计算在当前帧的目标搜索层数，如果面积超过3，说明仿射变换中的缩放是放大的，所以需要增加搜索层，缩小像素坐标*/
    // 如果面积放大率超过3，就往上一层，面积放大率变为原来的四分之一(长度的二分之一)。知道面积放大率不再大于3，或者到最高层。就得到了目标要搜索的层数
    search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);

    /*【步骤5】:使用search_level_和逆仿射变换，取参考帧<提取层>上的10x10的图块*/
    warp::warpAffine(A_cur_ref_, ref_ftr_->frame->img_pyr_[ref_ftr_->level], ref_ftr_->px,
            ref_ftr_->level, search_level_, halfpatch_size_+1, patch_with_border_);  //patch_with_border_: 是在<提取层>下的像素值

    /*【步骤6】:取10x10中心8x8范围图像块*/
    createPatchFromPatchWithBorder();

    // px_cur should be set
    // 将3D点在当前帧的投影变换到<search_level_>
    Vector2d px_scaled(px_cur/(1<<search_level_));

    /*【步骤7】:对这个图块的位置进行优化调整，使得它与目标位置的图块最匹配*/
    bool success = false;
    if(ref_ftr_->type == Feature::EDGELET)
    {
        // 边特征
        Vector2d dir_cur(A_cur_ref_*ref_ftr_->grad); /** @todo */
        dir_cur.normalize();
        success = feature_alignment::align1D(
                    cur_frame.img_pyr_[search_level_], dir_cur.cast<float>(),
                    patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
    }
    else
    {
        // 角点特征
        // 在<search_level_>层进行优化
        success = feature_alignment::align2D(
                    cur_frame.img_pyr_[search_level_], patch_with_border_, patch_, //patch_with_border_和patch_: 是在参考帧<提取层>下的像素值
                    options_.align_max_iter, px_scaled);
    }

    /*【步骤8】:转换到<0层>*/
    px_cur = px_scaled * (1<<search_level_);
    return success;
}

/** @brief 针对栅格中每一个3D点，寻找直接匹配，优化这一个feature在<当前帧>的位置
 *
 */
bool Matcher::findEpipolarMatchDirect(
        const Frame& ref_frame,
        const Frame& cur_frame,
        const Feature& ref_ftr,
        const double d_estimate,
        const double d_min,
        const double d_max,
        double& depth)
{
    SE3 T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.inverse();
    int zmssd_best = PatchScore::threshold();
    Vector2d uv_best;

    // Compute start and end of epipolar line in old_kf for match search, on unit plane!
    /*【步骤1】:将old_kf中最远和最近的深度估计值投影到当前帧的<归一化平面>*/
    Vector2d A = vk::project2d(T_cur_ref * (ref_ftr.f*d_min)); //最近
    Vector2d B = vk::project2d(T_cur_ref * (ref_ftr.f*d_max)); //最远
    epi_dir_ = A - B;

    // Compute affine warp matrix
    /*【步骤2】:计算仿射变换矩阵*/
    warp::getWarpMatrixAffine(
                *ref_frame.cam_, *cur_frame.cam_, ref_ftr.px, ref_ftr.f,
                d_estimate, T_cur_ref, ref_ftr.level, A_cur_ref_);

    // feature pre-selection
    /*【步骤3】:对于边缘点，如果把梯度仿射过来后，梯度的方向与极线方向的夹角大于45度，就认为沿着极线找，图块像素也不会变化很大，就不搜索了，直接返回false*/
    reject_ = false;
    if(ref_ftr.type == Feature::EDGELET && options_.epi_search_edgelet_filtering)
    {
        const Vector2d grad_cur = (A_cur_ref_ * ref_ftr.grad).normalized(); //对梯度进行仿射变换
        const double cosangle = fabs(grad_cur.dot(epi_dir_.normalized()));
        if(cosangle < options_.epi_search_edgelet_max_angle) { //epi_search_edgelet_max_angle: 0.7<->45度
            reject_ = true;
            return false;
        }
    }

    /*【步骤4】:计算在<当前帧>的目标搜索层数*/
    // 如果面积超过3，说明仿射变换中的缩放是放大的，所以需要增加搜索层，缩小像素坐标
    search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);

    // Find length of search range on epipolar line
    /*【步骤5】:寻找在极线上的搜索范围长度*/
    Vector2d px_A(cur_frame.cam_->world2cam(A)); //转换到像素坐标系
    Vector2d px_B(cur_frame.cam_->world2cam(B));
    epi_length_ = (px_A-px_B).norm()/(1<<search_level_); //在`search_level_`层上的对极搜索长度

    // Warp reference patch at ref_level
    /*【步骤6】:在`ref_level`进行仿射变换*/
    warp::warpAffine(A_cur_ref_, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
            ref_ftr.level, search_level_, halfpatch_size_+1, patch_with_border_);
    createPatchFromPatchWithBorder();


    /*【步骤7】:把极线线段投影到<search_level_>层数上，如果两个端点间的像素距离小于2个像素，就直接进行优化位置*/
    if(epi_length_ < 2.0)
    {
        px_cur_ = (px_A+px_B)/2.0;
        Vector2d px_scaled(px_cur_/(1<<search_level_));
        bool res;
        if(options_.align_1d)
            res = feature_alignment::align1D(
                        cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
                        patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
        else
            res = feature_alignment::align2D(
                        cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
                        options_.align_max_iter, px_scaled);
        if(res)
        {
            px_cur_ = px_scaled*(1<<search_level_); //优化后的feature在当前帧下的位置
            if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth)) //更新深度
                return true;
        }
        return false;
    }

    /*【步骤8】:如果两个端点间像素距离大于2个像素，就在极线上进行搜索*/

    // 确定总步长数，以两端点间的距离除以0.7，得到总步长数n_steps
    size_t n_steps = epi_length_/0.7; // one step per pixel
    // 把单位深度平面上的极线线段分n_steps段
    Vector2d step = epi_dir_/n_steps;

    if(n_steps > options_.max_epi_search_steps) //max_epi_search_steps: 1000
    {
        printf("WARNING: skip epipolar search: %zu evaluations, px_lenght=%f, d_min=%f, d_max=%f.\n",
               n_steps, epi_length_, d_min, d_max);
        return false;
    }

    // for matching, precompute sum and sum2 of warped reference patch
    int pixel_sum = 0;
    int pixel_sum_square = 0;
    PatchScore patch_score(patch_);

    // now we sample along the epipolar line
    // 沿着极线采样
    Vector2d uv = B-step; //莫怕，往下看
    Vector2i last_checked_pxi(0,0);
    ++n_steps; //上面减了一个step，所以这里加上一个
    /*【步骤9】:从一个端点开始往另外一个端点走，每走一步，就把位置投影（包括畸变）到对应层数的图像上，坐标取整后，获取图块*/
    for(size_t i=0; i<n_steps; ++i, uv+=step)
    {
        Vector2d px(cur_frame.cam_->world2cam(uv)); //转换到像素坐标系
        Vector2i pxi(px[0]/(1<<search_level_)+0.5,
                px[1]/(1<<search_level_)+0.5); // +0.5 to round to closest int

        if(pxi == last_checked_pxi)
            continue;
        last_checked_pxi = pxi;

        // check if the patch is full within the new frame
        if(!cur_frame.cam_->isInFrame(pxi, patch_size_, search_level_))
            continue;

        // TODO interpolation would probably be a good idea
        // 这里可以改进，不应该对坐标进行取整，而应该改成插值
        uint8_t* cur_patch_ptr = cur_frame.img_pyr_[search_level_].data
                + (pxi[1]-halfpatch_size_)*cur_frame.img_pyr_[search_level_].cols
                + (pxi[0]-halfpatch_size_);

        // 计算投影过来的图块与投影位置图块的相似度
        int zmssd = patch_score.computeScore(cur_patch_ptr, cur_frame.img_pyr_[search_level_].cols);

        // 取最高相似度
        if(zmssd < zmssd_best) {
            zmssd_best = zmssd;
            uv_best = uv;
        }
    }

    /*【步骤10】:如果分数小于阈值，就认为两个图块是相似的*/
    if(zmssd_best < PatchScore::threshold())
    {
        if(options_.subpix_refinement) //亚像素级优化
        {
            px_cur_ = cur_frame.cam_->world2cam(uv_best);
            Vector2d px_scaled(px_cur_/(1<<search_level_));
            bool res;
            if(options_.align_1d)
                res = feature_alignment::align1D(
                            cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
                            patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
            else
                res = feature_alignment::align2D(
                            cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
                            options_.align_max_iter, px_scaled);
            if(res)
            {
                px_cur_ = px_scaled*(1<<search_level_);
                if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
                    return true; //返回true
            }
            return false; //返回false
        }

        px_cur_ = cur_frame.cam_->world2cam(uv_best);
        if(depthFromTriangulation(T_cur_ref, ref_ftr.f, vk::unproject2d(uv_best).normalized(), depth))
            return true;
    }
    return false;
}

} // namespace svo
