// This file is part of SVO - Semi-direct Visual Odometry.
// 特征定义  高斯金字塔 faster角点 

#ifndef SVO_FEATURE_H_
#define SVO_FEATURE_H_

#include <frame.h>

namespace svo {

/// A salient image region that is tracked across frames.
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType {
    CORNER,
    EDGELET
  };

  FeatureType type;     //!< Type can be corner or edgelet.
  Frame* frame;         //!< Pointer to frame in which the feature was detected.
  Vector2d px;          //!< Coordinates in pixels on pyramid level 0. 在0层的像素坐标系上
  Vector3d f;           //!< Unit-bearing vector of the feature. 这是单位向量，并非归一化平面坐标
  int level;            //!< Image pyramid level where feature was extracted.
  Point* point;         //!< Pointer to 3D point which corresponds to the feature.
  Vector2d grad;        //!< Dominant gradient direction for edglets, normalized.

  Feature(Frame* _frame, const Vector2d& _px, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(frame->cam_->cam2world(px)), //这里如果图像是有畸变的，则会做畸变处理
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {}

  Feature(Frame* _frame, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {}

  Feature(Frame* _frame, Point* _point, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(_point),
    grad(1.0,0.0)
  {}
};

} // namespace svo

#endif // SVO_FEATURE_H_
