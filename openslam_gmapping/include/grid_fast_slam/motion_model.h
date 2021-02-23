#pragma once

#include "type/oriented_point.h"

namespace gmapping {

struct MotionModel {
 public:
  /**
   * @brief Compute a new pose with current pose, current and last odometry
   */
  OrientedPoint2d DrawFromMotion(const OrientedPoint2d& pose_cur,
                                 const OrientedPoint2d& odom_cur,
                                 const OrientedPoint2d& odom_prev) const;

 public:
  /** Linear error's variance due to linear motion */
  double srr;

  /** Angular error's variance due to linear motion */
  double srt;

  /** Linear error's variance due to angular motion */
  double str;

  /** Angular error's variance due to angular motion */
  double stt;
};

}  // namespace gmapping
