#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <vector>

#include "grid_fast_slam/motion_model.h"
#include "grid_fast_slam/node.h"
#include "grid_fast_slam/particle.h"
#include "scan_matcher/scan_matcher.h"
#include "sensor/range_reading.h"
#include "type/oriented_point.h"

namespace gmapping {

class GridSlamProcessor {
 public:
  GridSlamProcessor();

  /** Get the particle with the largest accumulated weight */
  const Particle& GetBestParticle() const;

  const std::vector<Particle>& GetParticles() const;

  /** Get the index of the particle with the largest accumulated weight */
  const size_t GetBestParticleIndex() const;

  /** @brief Set scan matcher for the used range sensor */
  void SetSensorMap(const SensorMap& names_2_sensors);

  /**
   * @brief Set motion model parameters
   *
   * @param srr - linear error's variance due to linear motion
   * @param srt - angular error's variance due to linear motion
   * @param str - linear error's variance due to angular motion
   * @param stt - angular error's variance due to angular motion
   */
  void SetMotionModelParameters(const double srr, const double srt,
                                const double str, const double stt);

  /**
   * @brief Set the distance threshold used for filter update
   *
   * @param linear             - Linear distance
   * @param angular            - Angular distance (angle)
   * @param resample_threshold - Elapsed time
   */
  void SetUpdateDistances(const double linear, const double angular,
                          const double resample_threshold);

  void SetUpdatePeriod(const double p);

  void SetGenerateMap(const bool generate_map);

  void SetTranslationalSamplingRange(const double ll_sample_range);

  void SetAngularSamplingRange(const double la_sample_range);

  void SetTranslationalSamplingStep(const double ll_sample_step);

  void SetAngularSamplingStep(const double la_sample_step);

  /** Set minimum score for considering the scan matching outcome good */
  void SetMinimumScore(const double minimum_score);

  void SetMatchingParameters(const double urange, const double max_range,
                             const double sigma, const int kernel_size,
                             const double lopt, const double aopt,
                             const size_t iterations,
                             const double likelihhod_sigma = 1.,
                             size_t likelihood_skip = 0,
                             const double likelihood_gain = 1.);

  /** @brief Initialize map and particles */
  void Initialize(const double xmin, const double ymin, const double xmax,
                  const double ymax, const double delta,
                  const size_t num_particles,
                  const OrientedPoint2d& initial_pose);

  /**
   * @brief Main function for processing scan
   * @details 1) Update odometry with motion model
   *          2) Improve proposal with latest observation
   *          3) Compute particles' weights with proposal and laser
   *          4) Resampling
   *
   * @param reading
   * @param adapt_particles - Particles we want to keep after resampling
   */
  bool ProcessScan(const RangeReading& reading,
                   const size_t adapt_particles = 0);

 protected:
  /**
   * @brief Scan match every single particle
   * @details If the scan matching fails, the particle gets a default likelihood
   * and uses odom pose as its pose. Otherwise, the particle's pose and weight
   * will be updated.
   */
  void ScanMatch(const std::vector<double>& ranges);

  /**
   * @brief Update weights for each particle's trajectory
   */
  void UpdateTreeWeights(const bool weight_already_normalized);

  bool Resample(const std::vector<double>& ranges, const size_t adapt_size,
                const RangeReadingPtr& reading);

  /**
   * @brief Normalize particle weights and compute number of effective particles
   * @details The current weights of all particle should sum to 1.
   *          num_eff = 1 / sum(w*w)
   */
  void Normalize();

  /**
   * @brief Reset all particles' trajectory's acc_weight to 0
   */
  void ResetTree();

  /**
   * @brief Propagate all weights of nodes
   */
  void PropagateWeights();

  /**
   * @brief Propagate all weights of children to parent node
   *
   * @param acc_weight - Accumulated weights of node's children
   * @param node       - Parent node
   */
  void PropagateWeight(const double acc_weight, const NodePtr& node);

 protected:
  std::vector<Particle> particles_;

  size_t num_beams_;

  ScanMatcher scan_matcher_;

  MotionModel motion_model_;

  double threshold_linear_distance_;
  double threshold_angular_distance_;
  double threshold_resample_threshold_;

  double period_;

  /** Minimum score for considering the scan matching outcome good */
  double minimum_score_;

  /** Smoothing factor for the likelihood */
  double likelihood_gain_;

  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;
  double delta_;

  double num_eff_;

  /** Number of ProcessScan() called */
  int cnt_;

  int reading_cnt_;

  /** Accumulated linear distances (reset if we update filter) */
  double acc_linear_distance_;

  /** Accumulated angular distances (reset if we update filter) */
  double acc_angular_distance_;

  OrientedPoint2d last_odom_pose_;

  double last_update_time_;

  /** Weights of all current particles (internally used) */
  std::vector<double> particle_weights_;

  /** Particle indices after resampling (internally used) */
  std::vector<size_t> particle_indices_;
};

}  // namespace gmapping
