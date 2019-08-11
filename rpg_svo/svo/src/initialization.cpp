// This file is part of SVO - Semi-direct Visual Odometry.
// 初始化
#include <config.h>
#include <frame.h>
#include <point.h>
#include <feature.h>
#include <initialization.h>
#include <feature_detection.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>

namespace svo {
namespace initialization {

InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
    reset();
    // 提取frame_ref中的fast特征点
    // px_ref_: keypoints to be tracked in reference frame
    // f_ref_ : bearing vectors corresponding to the keypoints in the reference image
    detectFeatures(frame_ref, px_ref_, f_ref_);

    // 特征点少于100,skip
    if(px_ref_.size() < 100)
    {
        SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
        return FAILURE;
    }
    frame_ref_ = frame_ref;
    px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end()); //为addSecondFrame中的trackKlt初始化一个好的值
    return SUCCESS;
}

InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
    //【步骤1】LK光流法跟踪
    trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
    SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");

    //【步骤2】少于50个，skip
    if(disparities_.size() < Config::initMinTracked())
        return FAILURE;

    double disparity = vk::getMedian(disparities_);
    SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");

    //【步骤3】视差不够大，skip
    if(disparity < Config::initMinDisparity()) //initMinDisparity: 50个像素长度
        return NO_KEYFRAME;

    //【步骤4】计算单应矩阵，三角化生成MapPoints
    computeHomography(
                f_ref_, f_cur_,
                frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
                inliers_, xyz_in_cur_, T_cur_from_ref_);
    SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");

    //【步骤5】匹配成功的点小于40，skip
    if(inliers_.size() < Config::initMinInliers())
    {
        SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
        return FAILURE;
    }

    // Rescale the map such that the mean scene depth is equal to the specified scale
    //【步骤6】调整尺度
    vector<double> depth_vec;
    for(size_t i=0; i<xyz_in_cur_.size(); ++i)
        depth_vec.push_back((xyz_in_cur_[i]).z());
    double scene_depth_median = vk::getMedian(depth_vec);
    double scale = Config::mapScale()/scene_depth_median; //Config::mapScale()=1
    frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
/** @todo 不能理解*/
    frame_cur->T_f_w_.translation() =
            -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

    // For each inlier create 3D point and add feature in both frames
    //【步骤7】向关键帧中插入feature，向<Point>加入看到它的<关键帧对应的feature>
    SE3 T_world_cur = frame_cur->T_f_w_.inverse();
    for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
    {
        Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
        Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
        // 去除边缘点
        if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10)         //frame_ref_和frame_cur的isInFrame()是一样的
                && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10)
                && xyz_in_cur_[*it].z() > 0)
        {
            Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
            Point* new_point = new Point(pos);

            // 当前帧
            Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0)); //都是位于<0层>??
            frame_cur->addFeature(ftr_cur);  //向关键帧加入能观测到的feature
            new_point->addFrameRef(ftr_cur); //向<Point>加入看到它的<关键帧对应的feature>

            // 参考帧
            Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
            frame_ref_->addFeature(ftr_ref);
            new_point->addFrameRef(ftr_ref);
        }
    }
    return SUCCESS;
}

void KltHomographyInit::reset()
{
    px_cur_.clear();
    frame_ref_.reset();
}

void detectFeatures(
        FramePtr frame,
        vector<cv::Point2f>& px_vec,
        vector<Vector3d>& f_vec)
{
    Features new_features;
    feature_detection::FastDetector detector(
                frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels()); //gridSize: 30, nPyrLevels: 3
    // 提取特征点
    detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

    // now for all maximum corners, initialize a new seed
    px_vec.clear(); px_vec.reserve(new_features.size());
    f_vec.clear(); f_vec.reserve(new_features.size());
    std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
        px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1])); //像素坐标
        f_vec.push_back(ftr->f); /** @attention ftr->f: 这是单位向量，并非归一化平面坐标*/
        delete ftr; //注意:析构
    });
}

void trackKlt(
        FramePtr frame_ref,
        FramePtr frame_cur,
        vector<cv::Point2f>& px_ref,
        vector<cv::Point2f>& px_cur,
        vector<Vector3d>& f_ref,
        vector<Vector3d>& f_cur,
        vector<double>& disparities)
{
    const double klt_win_size = 30.0;
    const int klt_max_iter = 30;
    const double klt_eps = 0.001;
    vector<uchar> status;
    vector<float> error;
    vector<float> min_eig_vec;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
    //! @attention 使用multi-scale Lucas-Kanade算法计算稀疏光流
    // slambook里面demo的方法
    cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
            px_ref, px_cur,
            status, error,
            cv::Size2i(klt_win_size, klt_win_size),
            4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW); //设置层:4

    vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
    vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
    vector<Vector3d>::iterator f_ref_it = f_ref.begin();

    // 保留空间
    f_cur.clear(); f_cur.reserve(px_cur.size());
    disparities.clear(); disparities.reserve(px_cur.size());

    for(size_t i=0; px_ref_it != px_ref.end(); ++i)
    {
        // 跟踪效果不好，从参考帧中擦除该角点
        if(!status[i])
        {
            px_ref_it = px_ref.erase(px_ref_it);
            px_cur_it = px_cur.erase(px_cur_it);
            f_ref_it = f_ref.erase(f_ref_it);
            continue;
        }
        f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y)); //将关键点像素坐标转换为单位向量，而非归一化平面坐标
        disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm()); //视差
        ++px_ref_it;
        ++px_cur_it;
        ++f_ref_it;
    }
}

void computeHomography(
        const vector<Vector3d>& f_ref,
        const vector<Vector3d>& f_cur,
        double focal_length,
        double reprojection_threshold,
        vector<int>& inliers,
        vector<Vector3d>& xyz_in_cur,
        SE3& T_cur_from_ref)
{
    vector<Vector2d > uv_ref(f_ref.size());
    vector<Vector2d > uv_cur(f_cur.size());
    for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
    {
        uv_ref[i] = vk::project2d(f_ref[i]);
        uv_cur[i] = vk::project2d(f_cur[i]);
    }
    vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
    Homography.computeSE3fromMatches();
    vector<int> outliers;
    vk::computeInliers(f_cur, f_ref,
                       Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
                       reprojection_threshold, focal_length,
                       xyz_in_cur, inliers, outliers);
    T_cur_from_ref = Homography.T_c2_from_c1;
}


} // namespace initialization
} // namespace svo
