// This file is part of SVO - Semi-direct Visual Odometry.
// 像素深度估计（基于概率）高斯均匀分布混合

#include <algorithm>
#include <vikit/math_utils.h>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <boost/bind.hpp>
#include <boost/math/distributions/normal.hpp>
#include <global.h>
#include <depth_filter.h>
#include <frame.h>
#include <point.h>
#include <feature.h>
#include <matcher.h>
#include <config.h>
#include <feature_detection.h>

namespace svo {

int Seed::batch_counter = 0;
int Seed::seed_counter = 0;

Seed::Seed(Feature* ftr, float depth_mean, float depth_min) :
    batch_id(batch_counter),
    id(seed_counter++),
    ftr(ftr),                  //对应的feature
    a(10),
    b(10),
    mu(1.0/depth_mean),        //均值，深度的倒数
    z_range(1.0/depth_min),    //深度范围为当前帧的最近的深度的倒数
    sigma2(z_range*z_range/36) //方差
{}

DepthFilter::DepthFilter(feature_detection::DetectorPtr feature_detector, callback_t seed_converged_cb) :
    feature_detector_(feature_detector),
    seed_converged_cb_(seed_converged_cb), //收敛回调
    seeds_updating_halt_(false),
    thread_(NULL),
    new_keyframe_set_(false),
    new_keyframe_min_depth_(0.0),
    new_keyframe_mean_depth_(0.0)
{}

DepthFilter::~DepthFilter()
{
    stopThread();
    SVO_INFO_STREAM("DepthFilter destructed.");
}

void DepthFilter::startThread()
{
    thread_ = new boost::thread(&DepthFilter::updateSeedsLoop, this);
}

void DepthFilter::stopThread()
{
    SVO_INFO_STREAM("DepthFilter stop thread invoked.");
    if(thread_ != NULL)
    {
        SVO_INFO_STREAM("DepthFilter interrupt and join thread... ");
        seeds_updating_halt_ = true;
        thread_->interrupt();
        thread_->join();
        thread_ = NULL;
    }
}

void DepthFilter::addFrame(FramePtr frame)
{
    if(thread_ != NULL)
    {
        {
            lock_t lock(frame_queue_mut_);
            if(frame_queue_.size() > 2) //只缓存最近的三个普通帧
                frame_queue_.pop(); //FIFO, pop删除最早的一个元素
            frame_queue_.push(frame); //第三个
        }
        seeds_updating_halt_ = false;
        frame_queue_cond_.notify_one();
    }
    else
        updateSeeds(frame);
}

void DepthFilter::addKeyframe(FramePtr frame, double depth_mean, double depth_min)
{
    new_keyframe_min_depth_ = depth_min;
    new_keyframe_mean_depth_ = depth_mean;

    // 线程已经开始，则条件成立
    // 好像条件一直成立?!
    if(thread_ != NULL)
    {
        new_keyframe_ = frame;
        new_keyframe_set_ = true;
        seeds_updating_halt_ = true;    /** @attention 当加入关键帧时，中断updateSeeds()函数*/
        frame_queue_cond_.notify_one(); //条件变量
    }
    else
        initializeSeeds(frame);
}

void DepthFilter::initializeSeeds(FramePtr frame)
{
    Features new_features;

    // 设置包含feature的栅格设置为占据状态
    // 这时的feature可能只来自与相邻关键帧匹配得到，所以只是一小部分，参考reprojectMap->reprojectCell函数
    feature_detector_->setExistingFeatures(frame->fts_);

    // 通过特征检测，提取更多feature
    feature_detector_->detect(frame.get(), frame->img_pyr_,
                              Config::triangMinCornerScore(), new_features);

    // initialize a seed for every new feature
    seeds_updating_halt_ = true;
    lock_t lock(seeds_mut_); // by locking the updateSeeds function stops
    ++Seed::batch_counter;
    std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
        seeds_.push_back(Seed(ftr, new_keyframe_mean_depth_, new_keyframe_min_depth_));
    });

    if(options_.verbose)
        SVO_INFO_STREAM("DepthFilter: Initialized "<<new_features.size()<<" new seeds");
    seeds_updating_halt_ = false;
}

void DepthFilter::removeKeyframe(FramePtr frame)
{
    seeds_updating_halt_ = true;
    lock_t lock(seeds_mut_);
    std::list<Seed>::iterator it=seeds_.begin();
    size_t n_removed = 0;
    while(it!=seeds_.end())
    {
        if(it->ftr->frame == frame.get())
        {
            it = seeds_.erase(it);
            ++n_removed;
        }
        else
            ++it;
    }
    seeds_updating_halt_ = false;
}

void DepthFilter::reset()
{
    seeds_updating_halt_ = true;
    {
        lock_t lock(seeds_mut_);
        seeds_.clear();
    }
    lock_t lock();
    while(!frame_queue_.empty())
        frame_queue_.pop();
    seeds_updating_halt_ = false;

    if(options_.verbose)
        SVO_INFO_STREAM("DepthFilter: RESET.");
}

void DepthFilter::updateSeedsLoop()
{
    while(!boost::this_thread::interruption_requested())
    {
        FramePtr frame;
        {
            lock_t lock(frame_queue_mut_);
            while(frame_queue_.empty() && new_keyframe_set_ == false)
                frame_queue_cond_.wait(lock); //解开互斥琐，挂起线程

            if(new_keyframe_set_)
            {
                new_keyframe_set_ = false;
                seeds_updating_halt_ = false;
                clearFrameQueue();           /** @attention 如果来了新的关键帧，则`清空普通帧队列`*/
                frame = new_keyframe_;       //使用关键帧更新
            }
            else
            {
                frame = frame_queue_.front(); //使用普通帧更新
                frame_queue_.pop();
            }
        }

        updateSeeds(frame);

        // 如果是关键帧，再检测一遍是否还有新的feature
        if(frame->isKeyframe())
            initializeSeeds(frame);
    }
}

void DepthFilter::updateSeeds(FramePtr frame)
{
    // update only a limited number of seeds, because we don't have time to do it for all the seeds in every frame!
    size_t n_updates=0, n_failed_matches=0, n_seeds = seeds_.size();
    lock_t lock(seeds_mut_);
    std::list<Seed>::iterator it=seeds_.begin();

    const double focal_length = frame->cam_->errorMultiplier2();
    double px_noise = 1.0;
    //!@attention law of chord (sehnensatz)，参考slambook公式13.8
    double px_error_angle = atan(px_noise/(2.0*focal_length))*2.0; // law of chord (sehnensatz)

    // 遍历每一个seed
    while( it!=seeds_.end())
    {
        // set this value true when seeds updating should be interrupted
        if(seeds_updating_halt_)
            return;

        // check if seed is not already too old
    /*【步骤1】:检查seed是否过去很久了?*/
        if((Seed::batch_counter - it->batch_id) > options_.max_n_kfs) { //max_n_kfs: 3
            it = seeds_.erase(it);
            continue;
        }

        // check if point is visible in the current image
    /*【步骤2】:检查seed在当前帧是否可见?*/
        SE3 T_ref_cur = it->ftr->frame->T_f_w_ * frame->T_f_w_.inverse();
        const Vector3d xyz_f(T_ref_cur.inverse()*(1.0/it->mu * it->ftr->f) );
        if(xyz_f.z() < 0.0)  {
            ++it; // behind the camera
            continue;
        }
        if(!frame->cam_->isInFrame(frame->f2c(xyz_f).cast<int>())) {
            ++it; // point does not project in image
            continue;
        }

        // we are using inverse depth coordinates
        float z_inv_min = it->mu + sqrt(it->sigma2); //越大，倒数越小
        float z_inv_max = max(it->mu - sqrt(it->sigma2), 0.00000001f); //防止变成负值
        double z;
    /*【步骤3】:使用feature所在的<参考帧>与<当前处理的帧>寻找对极几何直接法匹配*/
        if(!matcher_.findEpipolarMatchDirect(*it->ftr->frame, *frame, *it->ftr, 1.0/it->mu, 1.0/z_inv_min, 1.0/z_inv_max, z))
        {
            it->b++; // increase outlier probability when no match was found
            ++it;
            ++n_failed_matches;
            continue;
        }

        // compute tau
    /*【步骤4】:计算此时产生一个像素的误差导致的深度不确定性*/
        double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
    /*【步骤5】:在逆深度上的标准差*/
        double tau_inverse = 0.5 * (1.0/max(0.0000001, z-tau) - 1.0/(z+tau));

        // update the estimate
    /*【步骤6】:更新种子点的深度分布*/
        /** @todo 参考论文: Video-based, Real-Time Multi View Stereo*/
        updateSeed(1./z, tau_inverse*tau_inverse, &*it);
        ++n_updates;

        if(frame->isKeyframe())
        {
            // The feature detector should not initialize new seeds close to this location
    /*【步骤7】:如果是关键帧，则feature detector不应在px_cur_周围再生成新的seeds*/
            feature_detector_->setGridOccpuancy(matcher_.px_cur_);
        }

        // if the seed has converged, we initialize a new candidate point and remove the seed
    /*【步骤8】:如果种子点的方差，小于深度范围/200的时候，就认为收敛了，它就不再是种子点，而是candidate点*/
        if(sqrt(it->sigma2) < it->z_range/options_.seed_convergence_sigma2_thresh)
        {
            assert(it->ftr->point == NULL); // TODO this should not happen anymore
            Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (1.0/it->mu)));
            Point* point = new Point(xyz_world, it->ftr); //构造point对象时相当于执行了一次addFrameRef
            it->ftr->point = point;
            /* FIXME it is not threadsafe to add a feature to the frame here.
      if(frame->isKeyframe())
      {
        Feature* ftr = new Feature(frame.get(), matcher_.px_cur_, matcher_.search_level_);
        ftr->point = point;
        point->addFrameRef(ftr);
        frame->addFeature(ftr);
        it->ftr->frame->addFeature(it->ftr);
      }
      else
      */
    /*【步骤8.1】:使用回调函数，加入到候选seed list中*/
            {
                seed_converged_cb_(point, it->sigma2); // put in candidate list
            }
            it = seeds_.erase(it);
        }
        else if(isnan(z_inv_min))
        {
            SVO_WARN_STREAM("z_min is NaN");
            it = seeds_.erase(it);
        }
        else
            ++it;
    }
}

void DepthFilter::clearFrameQueue()
{
    while(!frame_queue_.empty())
        frame_queue_.pop();
}

void DepthFilter::getSeedsCopy(const FramePtr& frame, std::list<Seed>& seeds)
{
    lock_t lock(seeds_mut_);
    for(std::list<Seed>::iterator it=seeds_.begin(); it!=seeds_.end(); ++it)
    {
        if (it->ftr->frame == frame.get())
            seeds.push_back(*it);
    }
}

void DepthFilter::updateSeed(const float x, const float tau2, Seed* seed)
{
    float norm_scale = sqrt(seed->sigma2 + tau2);
    if(std::isnan(norm_scale))
        return;

    boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
    float s2 = 1./(1./seed->sigma2 + 1./tau2);
    float m = s2*(seed->mu/seed->sigma2 + x/tau2);
    float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
    float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
    float normalization_constant = C1 + C2;
    C1 /= normalization_constant;
    C2 /= normalization_constant;
    float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
    float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
            + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));

    // update parameters
    float mu_new = C1*m+C2*seed->mu;
    seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
    seed->mu = mu_new;
    seed->a = (e-f)/(f-e/f);
    seed->b = seed->a*(1.0f-f)/f;
}

//参考slambook
double DepthFilter::computeTau(
        const SE3& T_ref_cur,
        const Vector3d& f,
        const double z,
        const double px_error_angle)
{
    Vector3d t(T_ref_cur.translation());
    Vector3d a = f*z-t; //feature对应的3D点与当前帧的距离向量
    double t_norm = t.norm();
    double a_norm = a.norm();

    double alpha = acos(f.dot(t)/t_norm); // dot product
    double beta = acos(a.dot(-t)/(t_norm*a_norm)); // dot product
    double beta_plus = beta + px_error_angle;
    double gamma_plus = PI-alpha-beta_plus; // triangle angles sum to PI
    double z_plus = t_norm*sin(beta_plus)/sin(gamma_plus); // law of sines
    return (z_plus - z); // tau
}

} // namespace svo
