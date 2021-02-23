#include "grid_fast_slam/motion_model.h"

#include <math.h>

#include "utility/random_helper.h"

namespace gmapping {

OrientedPoint2d MotionModel::DrawFromMotion(
    const OrientedPoint2d& pose_cur, const OrientedPoint2d& odom_cur,
    const OrientedPoint2d& odom_prev) const {
  // Compute relative displacement wrt. last odom
  const OrientedPoint2d delta = AbsoluteDifference(odom_cur, odom_prev);

  // Compute a noisy relative displacement for now
  // This is our motion model
  OrientedPoint2d noisy_delta(delta);

  // Add noises
  const double sxy = 0.3 * srr;
  noisy_delta.x += RandomHelper::SampleGaussian(
      srr * fabs(delta.x) + str * fabs(delta.theta) + sxy * fabs(delta.y));
  noisy_delta.y += RandomHelper::SampleGaussian(
      srr * fabs(delta.y) + str * fabs(delta.theta) + sxy * fabs(delta.x));
  noisy_delta.theta += RandomHelper::SampleGaussian(
      stt * fabs(delta.theta) +
      srt * sqrt(delta.x * delta.x + delta.y * delta.y));

  // Constrain the theta inside [-pi, pi)
  noisy_delta.theta = fmod(noisy_delta.theta, 2. * M_PI);
  if (noisy_delta.theta >= M_PI) {
    noisy_delta.theta -= 2 * M_PI;
  }

  // New pose = current pose + noisy motion
  return AbsoluteSum(pose_cur, noisy_delta);
}

}  // namespace gmapping
