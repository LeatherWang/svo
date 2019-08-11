// This file is part of SVO - Semi-direct Visual Odometry.
// 特征检测

#include <feature_detection.h>
#include <feature.h>
#include <fast/fast.h>
#include <vikit/vision.h>

namespace svo {
namespace feature_detection {

AbstractDetector::AbstractDetector(
        const int img_width,
        const int img_height,
        const int cell_size,
        const int n_pyr_levels) :
    cell_size_(cell_size), //30, 一个feature的网格大小
    n_pyr_levels_(n_pyr_levels), //3
    grid_n_cols_(ceil(static_cast<double>(img_width)/cell_size_)),
    grid_n_rows_(ceil(static_cast<double>(img_height)/cell_size_)),
    grid_occupancy_(grid_n_cols_*grid_n_rows_, false)
{}

void AbstractDetector::resetGrid()
{
    std::fill(grid_occupancy_.begin(), grid_occupancy_.end(), false);
}

void AbstractDetector::setExistingFeatures(const Features& fts)
{
    std::for_each(fts.begin(), fts.end(), [&](Feature* i){
        grid_occupancy_.at(
                    static_cast<int>(i->px[1]/cell_size_)*grid_n_cols_
                + static_cast<int>(i->px[0]/cell_size_)) = true;
    });
}

void AbstractDetector::setGridOccpuancy(const Vector2d& px)
{
    grid_occupancy_.at(
                static_cast<int>(px[1]/cell_size_)*grid_n_cols_
            + static_cast<int>(px[0]/cell_size_)) = true;
}

FastDetector::FastDetector(
        const int img_width,
        const int img_height,
        const int cell_size,
        const int n_pyr_levels) :
    AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

void FastDetector::detect(
        Frame* frame,
        const ImgPyr& img_pyr,
        const double detection_threshold,
        Features& fts)
{
    Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));
    // 遍历每一层，检测角点
    for(int L=0; L<n_pyr_levels_; ++L)
    {
        const int scale = (1<<L);
        vector<fast::fast_xy> fast_corners;

#if __SSE2__
        // 检测角点
        fast::fast_corner_detect_10_sse2(
                    (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
                    img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#elif HAVE_FAST_NEON
        fast::fast_corner_detect_9_neon(
                    (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
                    img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#else
        fast::fast_corner_detect_10(
                    (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
                    img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#endif

        vector<int> scores, nm_corners;
        fast::fast_corner_score_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, fast_corners, 20, scores);
        fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

        for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
        {
            fast::fast_xy& xy = fast_corners.at(*it);
            //【1】根据角点出现的位置，计算该角点在哪个栅格内，即k
            const int k = static_cast<int>((xy.y*scale)/cell_size_)*grid_n_cols_  //scale: 1、2、4...
                        + static_cast<int>((xy.x*scale)/cell_size_);

            //【2】如果栅格已经被占据了(为什么会被占据?)，skip
            if(grid_occupancy_[k])
                continue;

            //【3】计算得分
            // Shi-Tomasi 分数，这个分数越高则特征越优先
            const float score = vk::shiTomasiScore(img_pyr[L], xy.x, xy.y);

            //【4】因为可能会有很多角点落在这个栅格里，所以需要选取得分最高的角点
            if(score > corners.at(k).score)
                corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f); //乘以scale是为了将角点当前层下的坐标转换到原始图像坐标系下，即映射到第0层的网格上
        }
    }

    // Create feature for every corner that has high enough corner score
    //【5】角点得分大于阈值，加入关键点list中
    std::for_each(corners.begin(), corners.end(), [&](Corner& c) {
        if(c.score > detection_threshold)
            fts.push_back(new Feature(frame, Vector2d(c.x, c.y), c.level)); //save feature
    });

    resetGrid();
}

} // namespace feature_detection
} // namespace svo

