// This file is part of SVO - Semi-direct Visual Odometry.
// 视觉前端基础类
// FrameHandlerBase先完成了一些设置，如最近10帧中特征数量等。
// 然后初始化了一系列算法性能监视器


#include <vikit/abstract_camera.h>
#include <stdlib.h>
#include <Eigen/StdVector>
#include <boost/bind.hpp>
#include <fstream>
#include <frame_handler_base.h>
#include <config.h>
#include <feature.h>
#include <matcher.h>
#include <map.h>
#include <point.h>

namespace svo
{

// definition of global and static variables which were declared in the header
// 全局静态变量 跟踪
#ifdef SVO_TRACE
vk::PerformanceMonitor* g_permon = NULL;
#endif

FrameHandlerBase::FrameHandlerBase() :
    stage_(STAGE_PAUSED),// 系统状态阶段
    set_reset_(false),//重置
    set_start_(false),//启动
    acc_frame_timings_(10),// 持续10帧
    acc_num_obs_(10),// 最近10帧中特征数量
    num_obs_last_(0),
    tracking_quality_(TRACKING_INSUFFICIENT)
{
#ifdef SVO_TRACE
    // Initialize Performance Monitor
    // 初始化了一系列算法性能监视器
    g_permon = new vk::PerformanceMonitor();
    g_permon->addTimer("pyramid_creation");//金字塔
    g_permon->addTimer("sparse_img_align");//配准
    g_permon->addTimer("reproject");//重投影
    g_permon->addTimer("reproject_kfs");
    g_permon->addTimer("reproject_candidates");
    g_permon->addTimer("feature_align");//特征匹配
    g_permon->addTimer("pose_optimizer");//位姿优化
    g_permon->addTimer("point_optimizer");//地图点优化
    g_permon->addTimer("local_ba");//局部ba优化
    g_permon->addTimer("tot_time");
    g_permon->addLog("timestamp");
    g_permon->addLog("img_align_n_tracked");
    g_permon->addLog("repr_n_mps");
    g_permon->addLog("repr_n_new_references");
    g_permon->addLog("sfba_thresh");
    g_permon->addLog("sfba_error_init");
    g_permon->addLog("sfba_error_final");
    g_permon->addLog("sfba_n_edges_final");
    g_permon->addLog("loba_n_erredges_init");
    g_permon->addLog("loba_n_erredges_fin");
    g_permon->addLog("loba_err_init");
    g_permon->addLog("loba_err_fin");
    g_permon->addLog("n_candidates");
    g_permon->addLog("dropout");
    g_permon->init(Config::traceName(), Config::traceDir());
#endif

    SVO_INFO_STREAM("SVO initialized");
}
//析构函数
FrameHandlerBase::~FrameHandlerBase()
{
    SVO_INFO_STREAM("SVO destructor invoked");
#ifdef SVO_TRACE
    delete g_permon;//删除 算法性能监视器
#endif
}

// 开始处理帧 判断系统状态  设置系统状态为处理第一帧 stage_ = STAGE_FIRST_FRAME
bool FrameHandlerBase::startFrameProcessingCommon(const double timestamp)
{
    // 首先判断set_start_，值为true时，（在创建vo_node时就已通过VoNode构造函数将set_start_设置为true）
    if(set_start_)
    {
        // 主要完成了Map型变量map_的初始化（包括关键帧和候选点的清空等）
        // 同时stage_被改为STAGE_PAUSED，set_reset和set_start都被设置为false
        resetAll();

        stage_ = STAGE_FIRST_FRAME;//开始处理第一帧 设置为处理第一帧
    }

    // 暂停状态 的话就不处理帧
    if(stage_ == STAGE_PAUSED)
        return false;

    // 将传进函数的系统时间和“New Frame”等信息记录至日志文件中。
    SVO_LOG(timestamp);
    SVO_DEBUG_STREAM("New Frame");
    SVO_START_TIMER("tot_time");
    
    // 启动vk::Timer型变量timer_（用于计量程序执行时间，可精确到毫秒级）。
    timer_.start();
    
    // 最后清空map_的垃圾箱trash
    map_.emptyTrash();
    return true;
}

// 完成帧处理
int FrameHandlerBase::finishFrameProcessingCommon(
        const size_t update_id,
        const UpdateResult dropout,
        const size_t num_observations)
{
    SVO_DEBUG_STREAM("Frame: "<<update_id<<"\t fps-avg = "<< 1.0/acc_frame_timings_.getMean()<<"\t nObs = "<<acc_num_obs_.getMean());
    SVO_LOG(dropout);
    // 统计当前帧处理时间并压入acc_frame_timings_ 来统计最近10帧的总处理时间。
    acc_frame_timings_.push_back(timer_.stop());

    // 如果stage_值为STAGE_DEFAULT_FRAME，还将nOb传入的值压入acc_num_obs_以统计最近10帧的检测出的特征点总数。
    if(stage_ == STAGE_DEFAULT_FRAME)
        acc_num_obs_.push_back(num_observations);
    num_obs_last_ = num_observations;//迭代
    SVO_STOP_TIMER("tot_time");

    // 是一个条件编译判断，如果定义了SVO_TRACE，跟踪
#ifdef SVO_TRACE
    g_permon->writeToFile();// 性能监控 写入日志（定义在performance_monitor.cpp中）
    {
        // 然后用互斥锁对线程进行写保护，
        boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
        // 再将特征点数量记录至日志
        size_t n_candidates = map_.point_candidates_.candidates_.size();
        SVO_LOG(n_candidates);
    }
#endif

    // 然后判断dropout值
    if(dropout == RESULT_FAILURE &&
            (stage_ == STAGE_DEFAULT_FRAME || stage_ == STAGE_RELOCALIZING ))
    {
        stage_ = STAGE_RELOCALIZING;// 重定位模型
        tracking_quality_ = TRACKING_INSUFFICIENT;//跟踪质量
    }
    else if (dropout == RESULT_FAILURE)// 当只有dropout == RESULT_FAILURE时，就执行resetAll()
        resetAll();// 进行Map型变量map_的初始化（包括关键帧和候选点的清空等）

    // 后判断set_reset_，如果为真，同样执行resetAll()
    if(set_reset_)
        resetAll();

    return 0;
}

// 重置
void FrameHandlerBase::resetCommon()
{
    map_.reset();
    stage_ = STAGE_PAUSED;
    set_reset_ = false;
    set_start_ = false;
    tracking_quality_ = TRACKING_INSUFFICIENT;
    num_obs_last_ = 0;
    SVO_INFO_STREAM("RESET");
}

// 设置跟踪质量
void FrameHandlerBase::setTrackingQuality(const size_t num_observations)
{
    tracking_quality_ = TRACKING_GOOD;
    if(num_observations < Config::qualityMinFts()) //50
    {
        SVO_WARN_STREAM_THROTTLE(0.5, "Tracking less than "<< Config::qualityMinFts() <<" features!");
        tracking_quality_ = TRACKING_INSUFFICIENT; //点数量不够
    }
    const int feature_drop = static_cast<int>(std::min(num_obs_last_, Config::maxFts())) - num_observations; //maxFts: 120
    if(feature_drop > Config::qualityMaxFtsDrop()) //容许相邻帧匹配到特征点的最大落差: 40
    {
        SVO_WARN_STREAM("Lost "<< feature_drop <<" features!");
        tracking_quality_ = TRACKING_INSUFFICIENT;
    }
}

// 排序
bool ptLastOptimComparator(Point* lhs, Point* rhs)
{
    return (lhs->last_structure_optim_ < rhs->last_structure_optim_);
}

void FrameHandlerBase::optimizeStructure(
        FramePtr frame,
        size_t max_n_pts, //20
        int max_iter)     //5
{
    deque<Point*> pts;
    for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
    {
        if((*it)->point != NULL)
            pts.push_back((*it)->point);
    }
    max_n_pts = min(max_n_pts, pts.size());
    nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), ptLastOptimComparator);
    for(deque<Point*>::iterator it=pts.begin(); it!=pts.begin()+max_n_pts; ++it)
    {
        (*it)->optimize(max_iter);
        (*it)->last_structure_optim_ = frame->id_;
    }
}

} // namespace svo
