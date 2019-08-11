// This file is part of SVO - Semi-direct Visual Odometry.
// 重投影

#include <algorithm>
#include <stdexcept>
#include <reprojector.h>
#include <frame.h>
#include <point.h>
#include <feature.h>
#include <map.h>
#include <config.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>
#include <vikit/timer.h>

namespace svo {

Reprojector::Reprojector(vk::AbstractCamera* cam, Map& map) :
    map_(map)
{
    initializeGrid(cam);
}

Reprojector::~Reprojector()
{
    std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ delete c; });
}

void Reprojector::initializeGrid(vk::AbstractCamera* cam)
{
    grid_.cell_size = Config::gridSize();
    grid_.grid_n_cols = ceil(static_cast<double>(cam->width())/grid_.cell_size);
    grid_.grid_n_rows = ceil(static_cast<double>(cam->height())/grid_.cell_size);
    grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);
    std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });
    grid_.cell_order.resize(grid_.cells.size());
    for(size_t i=0; i<grid_.cells.size(); ++i)
        grid_.cell_order[i] = i;
    // 将[first,last)的元素次序随机重排，均匀分布
    random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // maybe we should do it at every iteration!
}

void Reprojector::resetGrid()
{
    n_matches_ = 0;
    n_trials_ = 0;
    std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ c->clear(); });
}

void Reprojector::reprojectMap(
        FramePtr frame,
        std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs)
{
    resetGrid();

    // Identify those Keyframes which share a common field of view.
    /*【步骤1】:寻找当前帧与<Map中存储的关键帧>的有共视的关键帧，使用<创建关键帧时>生成的5个keyPoints投影进行寻找*/
    SVO_START_TIMER("reproject_kfs");
    list< pair<FramePtr,double> > close_kfs; //`double`项表示当前帧与<其它有共视的帧>之间的直线距离
    map_.getCloseKeyframes(frame, close_kfs);

    // Sort KFs with overlap according to their closeness
    /*【步骤2】:按照<有共视关系关键帧>与当前帧的距离从小到大排序*/
    close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) <
                   boost::bind(&std::pair<FramePtr, double>::second, _2));

    // Reproject all mappoints of the closest N kfs with overlap. We only store in which grid cell the points fall.
    /*【步骤3】:重投影所有有共视关系的关键帧中的MapPoints到当前帧，然后只存储落在栅格内的MapPoint*/
    // 按照关键帧距离从近到远的顺序，依次把这些关键帧上面的特征点对应的地图点都往当前帧上面投影，<同一个地图点>只被投影一次
    size_t n = 0;
    overlap_kfs.reserve(options_.max_n_kfs); //max_n_kfs默认值: 10
    for(auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end();
        it_frame!=ite_frame && n<options_.max_n_kfs; ++it_frame, ++n)
    {
        FramePtr ref_frame = it_frame->first;
        overlap_kfs.push_back(pair<FramePtr,size_t>(ref_frame,0));

        // Try to reproject each mappoint that the other KF observes
    /*【步骤3.1】:对每一个关键帧观测到的MapPoint重投影，计算<当前帧>能看到<这个关键帧>中多少个feature*/
        for(auto it_ftr=ref_frame->fts_.begin(), ite_ftr=ref_frame->fts_.end();
            it_ftr!=ite_ftr; ++it_ftr)
        {
            // check if the feature has a mappoint assigned
    /*【步骤3.1.1】:检查关键帧里面的feature是否有一个point*/
            if((*it_ftr)->point == NULL)
                continue;

            // make sure we project a point only once
    /*【步骤3.1.2】:<同一个地图点>只被投影一次*/
            // 一个point对应多个关键帧里面的feature，参照:addSecondFrame函数
            if((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
                continue;
            (*it_ftr)->point->last_projected_kf_id_ = frame->id_;

    /*【步骤3.1.3】:如果地图点在当前帧上的投影位置，能取到8x8的图块，就把这个地图点存入到当前帧投影位置的网格中*/
            if(reprojectPoint(frame, (*it_ftr)->point))
                overlap_kfs.back().second++;
        }
    }
    SVO_STOP_TIMER("reproject_kfs");

    // Now project all point candidates
    /*【步骤3.2】:重映射<候选point>(深度滤波中确定)到当前帧，如果映射失败次数过多，删除该候选point*/
    SVO_START_TIMER("reproject_candidates");
    {
        boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
        auto it=map_.point_candidates_.candidates_.begin();
        while(it!=map_.point_candidates_.candidates_.end())
        {
    /*【步骤3.2.1】:能取到8x8的图块，就把这个候选地图点存入到当前帧投影位置的网格中*/
            if( !reprojectPoint(frame, it->first) )
            {
                it->first->n_failed_reproj_ += 3;
    /*【步骤3.2.2】:如果一个候选点有10帧投影不成功，就把这个候选点删除掉*/
                if(it->first->n_failed_reproj_ > 30)
                {
                    map_.point_candidates_.deleteCandidate(*it);
                    it = map_.point_candidates_.candidates_.erase(it);
                    continue;
                }
            }
            ++it;
        }
    } // unlock the mutex when out of scope
    SVO_STOP_TIMER("reproject_candidates");

    // Now we go through each grid cell and select one point to match.
    // At the end, we should have at maximum one reprojected point per cell.
    /*【步骤4】:对于每一个网格，把其中对应的地图点，按照地图点的质量进行排序*/
    SVO_START_TIMER("feature_align");
    for(size_t i=0; i<grid_.cells.size(); ++i)
    {
        // we prefer good quality points over unkown quality (more likely to match)
        // and unknown quality over candidates (position not optimized)
        if(reprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame)) //grid_.cells在reprojectPoint函数中修改
            ++n_matches_;

        // 每个网格里，只要有一个地图点匹配成，就跳出这个网格的遍历循环
        // 如果有180个网格匹配成功了，直接跳出所有网格的循环
        if(n_matches_ > (size_t) Config::maxFts()) //maxFts: 120
            break;
    }
    SVO_STOP_TIMER("feature_align");
}

bool Reprojector::pointQualityComparator(Candidate& lhs, Candidate& rhs)
{
    // 对于每一个网格，把其中对应的地图点，按照地图点的质量进行排序
    // TYPE_GOOD> TYPE_UNKNOWN> TYPE_CANDIDATE> TYPE_DELETED
    if(lhs.pt->type_ > rhs.pt->type_)
        return true;
    return false;
}


bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)
{
    /*【步骤1】:对于每一个网格，把其中对应的地图点，按照地图点的质量进行排序，从高到低*/
    cell.sort(boost::bind(&Reprojector::pointQualityComparator, _1, _2));

    Cell::iterator it=cell.begin();
    // 遍历栅格中每一个地图点
    while(it!=cell.end())
    {
        ++n_trials_;

        if(it->pt->type_ == Point::TYPE_DELETED)
        {
            it = cell.erase(it);
            continue;
        }

    /*【步骤2】:针对每一个地图点，通过优化寻找直接法匹配*/
        // 参考论文章节: Relaxation Through Feature Alignment
        bool found_match = true;
        if(options_.find_match_direct)
            found_match = matcher_.findMatchDirect(*it->pt, *frame, it->px); /** @todo */

    /*【步骤3】:根据匹配结果，进行调整*/
        if(!found_match)
        {
            it->pt->n_failed_reproj_++;
            // what is type_unknown??
            // 如果是一个TYPE_UNKNOWN类型的地图点，它找匹配失败的次数大于15次，就把它变为delete类型的点
            if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_failed_reproj_ > 15)
                map_.safeDeletePoint(it->pt);
            // 如果是一个TYPE_CANDIDATE类型的点，它匹配失败的次数大于30次，就把它变为delete类型的点
            if(it->pt->type_ == Point::TYPE_CANDIDATE  && it->pt->n_failed_reproj_ > 30)
                map_.point_candidates_.deleteCandidatePoint(it->pt);
            // 擦除该能投影到cell中的point项
            it = cell.erase(it);
            continue;
        }
        it->pt->n_succeeded_reproj_++;

        // 转换为good
        if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_succeeded_reproj_ > 10)
            it->pt->type_ = Point::TYPE_GOOD;

        //!@attention 向当前帧加入feature
        Feature* new_feature = new Feature(frame.get(), it->px, matcher_.search_level_);
        frame->addFeature(new_feature);

        // Here we add a reference in the feature to the 3D point, the other way
        // round is only done if this frame is selected as keyframe.
        new_feature->point = it->pt;

    /*【步骤4】:如果对应的参考帧上的特征点是边缘点的话，则新的特征点的类型也设为边缘点，把梯度也仿射过来，归一化后，作为这个新特征点的梯度*/
        if(matcher_.ref_ftr_->type == Feature::EDGELET)
        {
            new_feature->type = Feature::EDGELET;
            new_feature->grad = matcher_.A_cur_ref_*matcher_.ref_ftr_->grad;
            new_feature->grad.normalize();
        }

        // If the keyframe is selected and we reproject the rest, we don't have to
        // check this point anymore.
        it = cell.erase(it);

        // Maximum one point per cell.
        // 每一个栅格内最大的一个点
        return true;
    }
    return false;
}

// 如果地图点在当前帧上的投影位置，能取到8x8的图块，就把这个地图点存入到当前帧投影位置的网格中
bool Reprojector::reprojectPoint(FramePtr frame, Point* point)
{
    Vector2d px(frame->w2c(point->pos_)); //3D点在当前帧的投影
    if(frame->cam_->isInFrame(px.cast<int>(), 8)) // 8px is the patch size in the matcher
    {
        const int k = static_cast<int>(px[1]/grid_.cell_size)*grid_.grid_n_cols
                + static_cast<int>(px[0]/grid_.cell_size);
        grid_.cells.at(k)->push_back(Candidate(point, px)); //加入到push到第k个list中
        return true;
    }
    return false;
}

} // namespace svo
