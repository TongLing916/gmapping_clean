#pragma once

#include <cstddef>
#include <vector>

#include "grid/hierarchical_array_2d.h"
#include "type/oriented_point.h"

namespace gmapping {

class ScanMatcher {
 public:
  /** Search directions */
  enum Move { kFront, kBack, kLeft, kRight, kTurnLeft, kTurnRight, kDone };

 public:
  ScanMatcher();

  /**
   * @brief Set laser parameters
   *
   * @param num_beams - Number of laser beams
   * @param angles    - Corresponding beam angles
   * @param pose      - Laser's pose
   */
  void SetLaserParamters(const size_t num_beams,
                         const std::vector<double>& angles,
                         const OrientedPoint2d& pose);

  void SetMaxRange(const double range);

  void SetUsableRange(const double range);

  void SetGenerateMap(const bool generate_map);

  void SetTranslationalSamplingRange(const double ll_sample_range);

  void SetAngularSamplingRange(const double la_sample_range);

  void SetTranslationalSamplingStep(const double ll_sample_step);

  void SetAngularSamplingStep(const double la_sample_step);

  void SetMatchingParameters(const double urange, const double max_range,
                             const double sigma, const int kernel_size,
                             const double lopt, const double aopt,
                             const size_t iterations,
                             const double likelihhod_sigma = 1.,
                             const size_t likelihood_skip = 0);

  /**
   * @brief Occupancy grid map algorithm
   *
   * @param pose             - Odom pose (base link)
   * @param ranges           - Laser readings
   * @param scan_matcher_map - Map
   * @return sum of entropy change
   */
  double RegisterScan(const OrientedPoint2d& pose,
                      const std::vector<double>& ranges,
                      ScanMatcherMap* const scan_matcher_map);

  /** Need to call every time before ComputeActiveArea() */
  void InvalidateActiveArea();

  void ComputeActiveArea(const OrientedPoint2d& pose,
                         const std::vector<double>& ranges,
                         ScanMatcherMap* const scan_matcher_map);

  double Optimize(const ScanMatcherMap& scan_matcher_map,
                  const OrientedPoint2d& init_pose,
                  const std::vector<double>& ranges,
                  OrientedPoint2d* const optimized_pose);

  size_t ComputeScoreAndLikelihood(const ScanMatcherMap& scan_matcher_map,
                                   const OrientedPoint2d& init_pose,
                                   const std::vector<double>& ranges,
                                   double* const score,
                                   double* const likelihood);

 private:
  double ComputeScore(const ScanMatcherMap& scan_matcher_map,
                      const OrientedPoint2d& pose,
                      const std::vector<double>& ranges) const;

 private:
  size_t num_beams_;
  std::vector<double> angles_;

  /** Laser's pose wrt. base link */
  OrientedPoint2d laser_pose_;

  bool generate_map_;

  /** Translational sampling range */
  double ll_sample_range_;

  /** Angular sampling range */
  double la_sample_range_;

  /** Translational sampling step */
  double ll_sample_step_;

  /** Angular sampling step */
  double la_sample_step_;

  /** Laser's usable range */
  double usable_range_;

  /** Laser's max range */
  double max_range_;

  /** Size of search window used for computing score */
  int kernel_size_;

  /** Linear step size during optimization */
  double opt_linear_delta_;

  /** Angular step size during optimization */
  double opt_angular_delta_;

  /** Number of iterations */
  size_t opt_recursive_iterations_;

  /** Variance for computing score */
  double gauss_sigma_;

  /** Variance for computing likelihood */
  double likelihood_sigma_;

  /** Number of beams we skip when computing likelihood */
  size_t likelihood_skip_;

  bool computed_active_area_;

  /** Number of first beams to skip */
  size_t initial_beams_skip_;

  /** Size for extending map */
  size_t enlarge_step_;

  /** Reliability of odometry */
  double angular_odometry_reliability = 0.;

  /** Reliability of odometry */
  double linear_odometry_reliability = 0.;

  double free_cell_ratio_;

  double fullness_threshold_;

  double likelihood_miss_;
};
}  // namespace gmapping
