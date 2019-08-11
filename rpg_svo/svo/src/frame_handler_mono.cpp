// This file is part of SVO - Semi-direct Visual Odometry.
// 视觉前端原理  深度滤波线程 基于 fast特征检测 和 地图点深度值滤波回调函数
/*
==============frame_handler_mono.cpp  ============================================       
processFirstFrame(); // 作用是处理第1帧并将其设置为关键帧；
processSecondFrame();// 作用是处理第1帧后面所有帧，直至找到一个新的关键帧；
processFrame();      // 作用是处理两个关键帧之后的所有帧；
relocalizeFrame(=);  //作用是在相关位置重定位帧以提供关键帧

startFrameProcessingCommon(timestamp) 设置处理第一帧  stage_ = STAGE_FIRST_FRAME;

processFirstFrame(); 处理第一帧，直到找打第一个关键帧==============================================
                1. fast角点特征，单应变换求取位姿变换，特点点数超过100个点才设置第一帧为为关键帧，
                2. 计算单应性矩阵（根据前两个关键帧）来初始化位姿，3d点
                3. 且设置系统处理标志为第二帧 stage_ = STAGE_SECOND_FRAME

processSecondFrame(); 处理第一帧关键帧到第二帧关键帧之间的所有帧(跟踪上一帧) ========================
                1. 光流法 金字塔多尺度 跟踪关键点,根据跟踪的数量和阈值，做跟踪成功/失败的判断
                2. 前后两帧计算单应性矩阵，根据重投影误差记录内点数量，根据阈值，判断跟踪 成功/失败,计算3d点
                3. 集束调整优化, 非线性最小二乘优化,
                4. 计算场景深度均值和最小值
                5. 深度滤波器对深度进行滤波，高斯均值混合模型进行更新深度值
                6. 设置系统状态 stage_= STAGE_DEFAULT_FRAME；

processFrame(); 处理前两个关键帧后的所有帧(跟踪上一帧) ============================================
                1. 设置初始位姿, 上帧的位姿初始化为当前帧的位姿
                2. 直接法 最小化 3d-2d 重投影 像素误差 求解位姿 LM算法优化位姿)
                3. 最小化 3d-2d 重投影像素误差 优化特征块的预测位置 更新3d-2d特征块匹配关系
                4. 使用优化后的 3d-2d特征块匹配关系， 类似直接法，最小化2d位置误差，来优化相机位姿(pose)
                5. 使用优化后的 3d-2d特征块匹配关系， 类似直接法，最小化2d位置误差，来优化3D点(structure)
                6. 深度值滤波

relocalizeFrame(); 重定位模式（跟踪参考帧）=======================================================
                1. 得到上一帧附近的关键帧作为参考帧 ref_keyframe
                2. 进行直接法 最小化 3d-2d 重投影 像素误差 求解当前帧的位姿
                3. 若跟踪质量较高(匹配特征点数大于30) 将参考帧设置为上一帧
                4. 直接法跟踪上一帧参考帧进行processFrame()处理( 其实就是跟踪参考帧模式 )
                5. 如果跟踪成功就设置为相应的跟踪后的位姿，否者设置为参考帧的位姿
*/
#include <config.h>
#include <frame_handler_mono.h>
#include <map.h>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <pose_optimizer.h>
#include <sparse_img_align.h>
#include <vikit/performance_monitor.h>
#include <depth_filter.h>
#ifdef USE_BUNDLE_ADJUSTMENT
#include <bundle_adjustment.h>
#endif

namespace svo {

FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam) :
    FrameHandlerBase(),// 继承于(基于)视觉前端基础类
    cam_(cam),
    reprojector_(cam_, map_),//继承于 重投影类 相机类 地图的生成与管理类
    depth_filter_(NULL)// 深度值滤波器
{
    initialize();
}

void FrameHandlerMono::initialize()
{
    // 特征检测类(FastDetector) 指针
    feature_detection::DetectorPtr feature_detector(
                new feature_detection::FastDetector( //fast特征检测器
                    cam_->width(),
                    cam_->height(),
                    Config::gridSize(),
                    Config::nPyrLevels() ) );

    // 地图点深度值滤波回调函数，回调函数指针所指内容，即由bind函数生成的指针
    DepthFilter::callback_t depth_filter_cb = boost::bind(
                &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);

    // 设置了特征检测器指针、回调函数指针、随机种子、线程、新关键帧深度的初值
    depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);

    // 启动线程，深度滤波器启动线程
    depth_filter_->startThread();
}

FrameHandlerMono::~FrameHandlerMono()
{
    delete depth_filter_;
}

void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)
{
    /*【1】 首先进行if判断，如果startFrameProcessingCommon返回false(暂停状态stage_ == STAGE_PAUSED )，*/
    if(!startFrameProcessingCommon(timestamp))
        return;

    /*【2】清空上次迭代变量*/
    // some cleanup from last iteration, can't do before because of visualization
    // Frame类型的智能指针shared_ptr，用于表示一帧周围的关键帧。
    core_kfs_.clear();
    // 一个向量，存储一个指针和一个数值构成的组合变量,用于表示具有重叠视野的关键帧，数值代表了重叠视野中的地标数。
    overlap_kfs_.clear();

    // create new frame
    SVO_START_TIMER("pyramid_creation");
    /*【3】对传入的img创建图像金字塔img_pyr_（一个Mat型向量）*/
    new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
    SVO_STOP_TIMER("pyramid_creation");

    /*【4】处理帧，首先设置UpdateResult型枚举变量res，值为RESULT_FAILURE*/
    // process frame
    UpdateResult res = RESULT_FAILURE;

    if(stage_ == STAGE_DEFAULT_FRAME)
        res = processFrame();// 作用是处理前两个关键帧之后的所有帧；
    else if(stage_ == STAGE_SECOND_FRAME)
        res = processSecondFrame();// 作用是处理第1帧后面所有帧，直至找到一个新的关键帧；
    else if(stage_ == STAGE_FIRST_FRAME)
        res = processFirstFrame();// 作用是处理第1帧并将其设置为关键帧( 特点点数超过100个点才设置第一帧为为关键帧)
                                  // 并且设置  stage_ = STAGE_SECOND_FRAME 处理第二帧模式
    else if(stage_ == STAGE_RELOCALIZING)
        res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()), //作用是在相关位置重定位帧以提供关键帧
                              map_.getClosestKeyframe(last_frame_));

    /*【5】迭代步骤，将new_frame_赋给last_frame_，然后将new_frame_给清空，供下次使用*/
    // set last frame
    last_frame_ = new_frame_;
    new_frame_.reset();

    /*【6】完成帧处理， 执行finishFrameProcessingCommon，传入的参数为last_frame_的id号和图像中的特征数nOb的值、res的值*/
    // finish processing
    finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
}

/**
 * @brief 处理第一帧，直到找打第一个关键帧
 * 1. fast角点匹配
 * 2. 单应变换求变换矩阵
 * 3. 设置系统状态标志 stage_ = STAGE_SECOND_FRAME
 * 4. 特点点数超过100个点才设置第一帧为为关键帧，且设置系统处理标志为第二帧 stage_ = STAGE_SECOND_FRAME
 */
FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
{
    //【1】初始化第一帧的位姿
    new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());

    //【2】检测第一帧初始化情况
    // 其中klt_homography_init_是KltHomographyInit（FrameHandlerMono的友元类）类型的类，
    // 用于计算单应性矩阵（根据前两个关键帧）来初始化位姿
    if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
        return RESULT_NO_KEYFRAME;// 第一帧初始化错误 没有关键帧

    //【3】第一帧初始化成功，设置关键帧(标志位、五个关键点)
    new_frame_->setKeyframe();
    map_.addKeyframe(new_frame_);

    //【4】设置状态为:
    stage_ = STAGE_SECOND_FRAME;

    // 将信息“Init: Selected first frame”记录至日志
    SVO_INFO_STREAM("Init: Selected first frame.");
    return RESULT_IS_KEYFRAME;
}

/**
 * @brief 处理第一帧关键帧到第二帧关键帧之间的所有帧，作用是处理第1帧后面所有帧，直至找到一个新的关键帧
 * 1. 金字塔多尺度光流跟踪关键点
 * 2. 计算前后两帧的单应变换矩阵，计算3d点
 * 3. 集束调整优化, 非线性最小二乘优化
 * 4. 计算场景深度均值和最小值
 * 5. 深度滤波器对深度进行滤波，高斯均值混合模型进行更新深度值
 * 6. 设置系统状态 stage_= STAGE_DEFAULT_FRAME；
 */
FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
{
    //【1】首先调用trackKlt函数跟踪特征（LK光流法） 金字塔多尺度跟踪
    initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);

    if(res == initialization::FAILURE)
        return RESULT_FAILURE; //第二帧跟踪失败
    else if(res == initialization::NO_KEYFRAME)
        return RESULT_NO_KEYFRAME; //移动距离过小，不设置为关键帧

    //【2】条件编译，如果定义了USE_BUNDLE_ADJUSTMENT，就进行BA优化，通过调用ba::twoViewBA函数
#ifdef USE_BUNDLE_ADJUSTMENT
    // 优化第二个关键帧的位姿(第一个关键帧固定)和MapPoints的位置
    // 使用的是重投影误差
    ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_); //lobaThresh()=2
#endif

    //【3】设置关键帧
    /** @attention 最终选出最具有代表性的5个作为关键点, 实质上是1个靠近图像中心的点和4个靠近图像四个角的点*/
    new_frame_->setKeyframe();

    //【4】通过函数getSceneDepth获取场景平均深度（depth_mean）最小深度（depth_min）
    double depth_mean, depth_min;
    frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);

    //【5】向深度滤波器depth_filter中添加关键帧（当前帧），传入参数depth_mean、0.5 * depth_min（不知道为啥除以2）进行初始化
    depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

    //【6】想地图中添加关键帧
    // add frame to map
    map_.addKeyframe(new_frame_);

    //【7】设置系统状态 stage_= STAGE_DEFAULT_FRAME。
    stage_ = STAGE_DEFAULT_FRAME;

    //【8】调用klt_homography_init_.reset()，初始化px_cur_和frame_ref_,迭代为下一次左准备
    klt_homography_init_.reset();
    SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
    return RESULT_IS_KEYFRAME;
}

/**
 * @brief 处理前两个关键帧后的所有帧
 * 1. 设置初始位姿,上帧的位姿初始化为当前帧的位姿
 * 2. 直接法，最小化 3d-2d 重投影 像素误差 求解位姿 LM算法优化位姿)
 * 3. 最小化 3d-2d 重投影像素误差 优化特征块的预测位置 更新3d-2d特征块匹配关系
 * 4. 使用优化后的 3d-2d特征块匹配关系， 类似直接法，最小化2d位置误差，来优化相机位姿(pose)
 * 5. 使用优化后的 3d-2d特征块匹配关系， 类似直接法，最小化2d位置误差，来优化3D点(structure)
 * 6. 深度值滤波
 */
FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
{
    /*【步骤1】:设置初始位姿，即将上帧（last_frame_）的变换矩阵（T_f_w_）赋给当前帧的变换矩阵（T_f_w_）*/
    // Set initial pose TODO use prior
    new_frame_->T_f_w_ = last_frame_->T_f_w_;

    /*【步骤2】:直接法最小化 3d-2d 重投影像素误差,求解位姿*/
    // sparse image align
    SVO_START_TIMER("sparse_img_align");
    SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(), //kltMaxLevel: 4, kltMinLevel: 2
                             30, SparseImgAlign::GaussNewton, false, false); /** @attention display=false, verbose=false*/

    /*【步骤2.1】:图像的稀疏对齐（应该是匹配的意思）（类似于直接法）：3d点重投影，最小化像素匹配差值*/
    // SVO通过直接特征对齐获得亚像素特征匹配精度
    // 用于图像对齐的地图点是上一帧所能看到的地图点，按先验知识来讲，图像帧间变换比较小，我们有理由相信上一帧和当前帧所能看到场景大部分相同。
    // 使用上一帧中看到的3d点(对应着上一帧图像的2d像素坐标)
    // 根据位姿变换投影到当前帧像素平面上，计算亚像素值，和上一帧对应点的像素值做差，LM算法优化误差，得到优化后的位姿和3d点(还有3d点???)
    size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);
    SVO_STOP_TIMER("sparse_img_align");
    SVO_LOG(img_align_n_tracked);
    SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

    /*【步骤3】:最小化 3d-2d 重投影像素误差，优化特征块的预测位置，更新3d-2d特征块匹配关系*/
    /*【步骤3.1】:最小化地图重投影和特征对齐（或许是匹配）*/
    // 然后进行判断，如果匹配到的特征数小于阈值，则打印没有匹配到足够的特征信息，同时设置<当前帧变换矩阵>为<上一帧变换矩阵>
    // map reprojection & feature alignment
    SVO_START_TIMER("reproject");
    reprojector_.reprojectMap(new_frame_, overlap_kfs_);// 地图重投影
    SVO_STOP_TIMER("reproject");
    const size_t repr_n_new_references = reprojector_.n_matches_;
    const size_t repr_n_mps = reprojector_.n_trials_;
    SVO_LOG2(repr_n_mps, repr_n_new_references);
    SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);
    /*【步骤3.2】:如果成功匹配的网格的数量小于30个，就认为当前帧的匹配失败*/
    if(repr_n_new_references < Config::qualityMinFts())
    {
        SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
        new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
        tracking_quality_ = TRACKING_INSUFFICIENT;
        return RESULT_FAILURE;
    }

    /*【步骤4】:使用优化后的3d-2d特征块匹配关系，类似直接法，最小化2d位置误差，来对相机位姿(pose)进行反向优化*/
    //相机位姿优化 pose optimization
    SVO_START_TIMER("pose_optimizer");
    size_t sfba_n_edges_final;
    double sfba_thresh, sfba_error_init, sfba_error_final;
    pose_optimizer::optimizeGaussNewton(
                Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
                new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
    SVO_STOP_TIMER("pose_optimizer");
    SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
    SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
    SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
    // 如果最后剩下的匹配成功点的数量大于20个，就认为优化成功
    if(sfba_n_edges_final < 20)
        return RESULT_FAILURE;

    /*【步骤5】:3d地图点优化 structure optimization*/
    SVO_START_TIMER("point_optimizer");
    optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter()); //20, 5
    SVO_STOP_TIMER("point_optimizer");

    /*【步骤6】:选择关键帧 select keyframe*/
    core_kfs_.insert(new_frame_);// 将当前帧插入core_kfs_（用于存储附近关键帧）

    /*【步骤7】:将跟踪质量设置为 sfba_n_edges_final*/
    setTrackingQuality(sfba_n_edges_final);

    /*【步骤8】:判断tracking_quality_ ，若等于TRACKING_INSUFFICIENT，同时设置当前帧变换矩阵为上帧变换矩阵*/
    if(tracking_quality_ == TRACKING_INSUFFICIENT)//跟踪的点不多，值不准确，使用上次的位姿
    {
        new_frame_->T_f_w_ = last_frame_->T_f_w_; // 避免位姿乱跳，抑制噪声 reset to avoid crazy pose jumps
        return RESULT_FAILURE;
    }

    /*【步骤9】:获取场景最小和平均深度，根据平均深度判断是否符合关键帧选择标准*/
    // 若不合适或者tracking_quality_ 值为 TRACKING_BAD，就将当前帧添加入深度滤波器，然后返回RESULT_NO_KEYFRAME。
    double depth_mean, depth_min;
    frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);// 场景最小和平均深度

    //  根据距离<相邻关键帧>的位移量，判断是否需要新建关键帧
    if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
    {
        depth_filter_->addFrame(new_frame_);/** @todo 对当前帧(候选关键帧)的深度值使用深度滤波器进行滤波更新*/
        return RESULT_NO_KEYFRAME;
    }

    /*【步骤10】:将当前帧的深度值进行滤波更新后，设置为关键帧*/
    new_frame_->setKeyframe();
    SVO_DEBUG_STREAM("New keyframe selected.");

    // new keyframe selected
    for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
        if((*it)->point != NULL)
            (*it)->point->addFrameRef(*it);//参考帧

    /*【步骤11】:将map_.point_candidates_中与当前帧相关的特征点添加到当前帧*/
    map_.point_candidates_.addCandidatePointToFrame(new_frame_);

    /*【步骤12】:条件编译，如果定义了USE_BUNDLE_ADJUSTMENT，则进行BA优化，optional bundle adjustment*/
#ifdef USE_BUNDLE_ADJUSTMENT
    if(Config::lobaNumIter() > 0) //lobaNumIter: 0
    {
        SVO_START_TIMER("local_ba");
        setCoreKfs(Config::coreNKfs());
        size_t loba_n_erredges_init, loba_n_erredges_fin;
        double loba_err_init, loba_err_fin;
        // 局部两帧之间的 BA优化
        ba::localBA(new_frame_.get(), &core_kfs_, &map_,
                    loba_n_erredges_init, loba_n_erredges_fin,
                    loba_err_init, loba_err_fin);
        SVO_STOP_TIMER("local_ba");
        SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
        SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                                                                                                        "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
    }
#endif

    /*【步骤13】:将当前关键帧添加到深度滤波器 init new depth-filters*/
    depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

    /*【步骤14】:移除map_中距离较远的关键帧*/
    // if limited number of keyframes, remove the one furthest apart
    if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
    {
        FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
        depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
        map_.safeDeleteFrame(furthest_frame);// 移除map_中距离较远的关键帧。
    }

    /*【步骤15】:添加当前关键帧到map_地图, add keyframe to map*/
    map_.addKeyframe(new_frame_);

    return RESULT_IS_KEYFRAME;
}

/**
 * @brief 重定位模式，需要回到跟丢的位置才有可能找回
 */
FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
        const SE3& T_cur_ref,
        FramePtr ref_keyframe)
{
    /*【步骤1】:调用时，使用 map_.getClosestKeyframe()得到上一帧附近的关键帧作为参考帧*/
    //    res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
    //    map_.getClosestKeyframe(last_frame_));
    SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");

    /*【步骤1】:判断ref_keyframe值，若为nullptr，则结束并返回RESULT_FAILURE*/
    if(ref_keyframe == nullptr)// 没找到关键帧
    {
        SVO_INFO_STREAM("No reference keyframe.");
        return RESULT_FAILURE;// 返回错误
    }

    /*【步骤3】:直接法,最小化3d-2d重投影像素块误差，求解位姿*/
    SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                             30, SparseImgAlign::GaussNewton, false, false);
    size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);

    /*【步骤4】:如果匹配特征数大于30*/
    if(img_align_n_tracked > 30)
    {
        SE3 T_f_w_last = last_frame_->T_f_w_;//将上帧变换矩阵赋给T_f_w_last，
        last_frame_ = ref_keyframe;// 设置last_frame为参考帧ref_keyframe
        FrameHandlerMono::UpdateResult res = processFrame();// 跟踪上一帧, 即参考帧
        // 跟踪上一帧参考帧成功 进入普通的跟踪上一帧模式
        if(res != RESULT_FAILURE)
        {
            stage_ = STAGE_DEFAULT_FRAME;
            SVO_INFO_STREAM("Relocalization successful.");
        }
        else // 设置为 最近的关键帧处的位姿
            new_frame_->T_f_w_ = T_f_w_last; // reset to last well localized pose
        return res;
    }

    return RESULT_FAILURE;
}


// ========================================================
bool FrameHandlerMono::relocalizeFrameAtPose(
        const int keyframe_id,
        const SE3& T_f_kf,
        const cv::Mat& img,
        const double timestamp)
{
    FramePtr ref_keyframe;
    if(!map_.getKeyframeById(keyframe_id, ref_keyframe))
        return false;
    new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
    UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
    if(res != RESULT_FAILURE) {
        last_frame_ = new_frame_;
        return true;
    }
    return false;
}

// 重置 =============================================================
void FrameHandlerMono::resetAll()
{
    resetCommon();
    last_frame_.reset();
    new_frame_.reset();
    core_kfs_.clear();
    overlap_kfs_.clear();
    depth_filter_->reset();
}

// 设置第一帧为关键帧 ==========================================================
void FrameHandlerMono::setFirstFrame(const FramePtr& first_frame)
{
    resetAll();
    last_frame_ = first_frame;
    last_frame_->setKeyframe();
    map_.addKeyframe(last_frame_);
    stage_ = STAGE_DEFAULT_FRAME;
}


// 根据距离<相邻关键帧>的位移量，判断是否需要新建关键帧 =============================================
bool FrameHandlerMono::needNewKf(double scene_depth_mean)
{
    for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
    {
        Vector3d relpos = new_frame_->w2f(it->first->pos());
        if(fabs(relpos.x())/scene_depth_mean < Config::kfSelectMinDist() &&         //kfSelectMinDist: 0.12
                fabs(relpos.y())/scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
                fabs(relpos.z())/scene_depth_mean < Config::kfSelectMinDist()*1.3)
            return false;
    }
    return true;
}

// CoreKfs 用于表示一帧周围的关键帧 =========================================
void FrameHandlerMono::setCoreKfs(size_t n_closest)
{
    size_t n = min(n_closest, overlap_kfs_.size()-1);
    std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin()+n, overlap_kfs_.end(),
                      boost::bind(&pair<FramePtr, size_t>::second, _1) >
                      boost::bind(&pair<FramePtr, size_t>::second, _2));
    std::for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i)
    { core_kfs_.insert(i.first); });
}

} // namespace svo
