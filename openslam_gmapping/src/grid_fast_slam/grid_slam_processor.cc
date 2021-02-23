#include "grid_fast_slam/grid_slam_processor.h"

#include <limits>

#include <glog/logging.h>

#include "grid_fast_slam/uniform_sampler.h"

namespace gmapping {

GridSlamProcessor::GridSlamProcessor()
    : num_beams_(0),
      threshold_linear_distance_(0),
      threshold_angular_distance_(0),
      threshold_resample_threshold_(0),
      period_(0),
      minimum_score_(0),
      likelihood_gain_(0),
      xmin_(0),
      ymin_(0),
      xmax_(0),
      ymax_(0),
      delta_(0),
      num_eff_(0),
      reading_cnt_(0),
      acc_linear_distance_(0),
      acc_angular_distance_(0),
      last_update_time_(0) {}

const std::vector<Particle>& GridSlamProcessor::GetParticles() const {
  return particles_;
}

const size_t GridSlamProcessor::GetBestParticleIndex() const {
  CHECK(!particles_.empty());

  size_t best_idx = 0;
  double max_weight = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < particles_.size(); ++i) {
    if (max_weight < particles_.at(i).weight_sum) {
      max_weight = particles_.at(i).weight_sum;
      best_idx = i;
    }
  }
  return best_idx;
}

const Particle& GridSlamProcessor::GetBestParticle() const {
  return GetParticles().at(GetBestParticleIndex());
}

void GridSlamProcessor::SetSensorMap(const SensorMap& names_2_sensors) {
  auto it = names_2_sensors.find("FLASER");
  LOG_IF(FATAL, it == names_2_sensors.end())
      << "FORGOT SET LASER'S NAME TO [FLASER]?";

  auto range_sensor = std::dynamic_pointer_cast<RangeSensor>(it->second);
  CHECK(range_sensor);

  num_beams_ = range_sensor->CountBeams();
  scan_matcher_.SetLaserParamters(num_beams_, range_sensor->GetAngles(),
                                  range_sensor->GetPose());
}

void GridSlamProcessor::SetMotionModelParameters(const double srr,
                                                 const double srt,
                                                 const double str,
                                                 const double stt) {
  motion_model_.srr = srr;
  motion_model_.srt = srt;
  motion_model_.str = str;
  motion_model_.stt = stt;
}

void GridSlamProcessor::SetUpdateDistances(const double linear,
                                           const double angular,
                                           const double resample_threshold) {
  threshold_linear_distance_ = linear;
  threshold_angular_distance_ = angular;
  threshold_resample_threshold_ = resample_threshold;
}

void GridSlamProcessor::SetUpdatePeriod(const double p) { period_ = p; }

void GridSlamProcessor::SetGenerateMap(const bool generate_map) {
  scan_matcher_.SetGenerateMap(generate_map);
}

void GridSlamProcessor::SetTranslationalSamplingRange(
    const double ll_sample_range) {
  scan_matcher_.SetTranslationalSamplingRange(ll_sample_range);
}

void GridSlamProcessor::SetAngularSamplingRange(const double la_sample_range) {
  scan_matcher_.SetAngularSamplingRange(la_sample_range);
}

void GridSlamProcessor::SetTranslationalSamplingStep(
    const double ll_sample_step) {
  scan_matcher_.SetTranslationalSamplingStep(ll_sample_step);
}

void GridSlamProcessor::SetAngularSamplingStep(const double la_sample_step) {
  scan_matcher_.SetAngularSamplingStep(la_sample_step);
}

void GridSlamProcessor::SetMinimumScore(const double minimum_score) {
  minimum_score_ = minimum_score;
}

void GridSlamProcessor::SetMatchingParameters(
    const double urange, const double max_range, const double sigma,
    const int kernel_size, const double lopt, const double aopt,
    const size_t iterations, const double likelihhod_sigma,
    const size_t likelihood_skip, const double likelihood_gain) {
  scan_matcher_.SetMatchingParameters(urange, max_range, sigma, kernel_size,
                                      lopt, aopt, iterations, likelihhod_sigma,
                                      likelihood_skip);

  CHECK_GT(likelihood_gain, 1e-6);
  likelihood_gain_ = likelihood_gain;
}

void GridSlamProcessor::Initialize(const double xmin, const double ymin,
                                   const double xmax, const double ymax,
                                   const double delta,
                                   const size_t num_particles,
                                   const OrientedPoint2d& initial_pose) {
  xmin_ = xmin;
  ymin_ = ymin;
  xmax_ = xmax;
  ymax_ = ymax;
  delta_ = delta;

  particles_.clear();
  auto node = std::make_shared<Node>(initial_pose, 0, nullptr, 0);

  // Initialize each particle's maps (1. high-resolution, 2. low-resolution)
  // Low resolution is fixed (0.1m)
  // High resolution is customized
  const auto center = Point2d(xmin_ + xmax_, ymin_ + ymax_) * 0.5;
  const double world_size_x = xmax_ - xmin_;
  const double world_size_y = ymax_ - ymin_;
  ScanMatcherMap high_map(center, world_size_x, world_size_y, delta_);
  ScanMatcherMap low_map(center, world_size_x, world_size_y, 0.1);

  for (size_t i = 0; i < num_particles; ++i) {
    particles_.emplace_back(high_map, low_map);

    auto& particle = particles_.back();
    particle.pose = initial_pose;
    particle.prev_pose = initial_pose;
    particle.weight = 0.;
    particle.prev_idx = 0;
    particle.node = node;
    CHECK(particle.node->parent == nullptr);
  }

  num_eff_ = static_cast<double>(num_particles);
  cnt_ = 0;
  reading_cnt_ = 0;
  acc_linear_distance_ = 0.;
  acc_angular_distance_ = 0.;
}

bool GridSlamProcessor::ProcessScan(const RangeReading& reading,
                                    const size_t adapt_particles) {
  // Retrieve the current pose from the reading
  const auto cur_odom_pose = reading.GetPose();

  if (cnt_ == 0) {
    // Now is the first scan
    last_odom_pose_ = cur_odom_pose;
  }

  // Update particle's state with motion model
  for (size_t i = 0; i < particles_.size(); ++i) {
    particles_[i].pose = motion_model_.DrawFromMotion(
        particles_[i].pose, cur_odom_pose, last_odom_pose_);
  }

  // Accumulate translation and rotation
  {
    OrientedPoint2d move = cur_odom_pose - last_odom_pose_;
    move.theta = atan2(sin(move.theta), cos(move.theta));

    acc_linear_distance_ += sqrt(move * move);
    acc_angular_distance_ += fabs(move.theta);
  }

  // If the robot has moved a large distance, then give a warning, since this
  // could be caused by odometry or laser bug.
  if (acc_linear_distance_ > threshold_linear_distance_) {
    LOG(ERROR) << "WE HAVE MOVED DISTANCE=" << acc_linear_distance_
               << " (THRESHOLD: " << threshold_linear_distance_
               << ") WITHOUT UPDATE! ANYTHING WRONG WITH ODOMETRY OR LASER?";
  }

  last_odom_pose_ = cur_odom_pose;

  const bool process_scan =
      (cnt_ == 0) || (acc_linear_distance_ > threshold_linear_distance_) ||
      (acc_angular_distance_ > threshold_angular_distance_) ||
      (period_ >= 0. && reading.GetTime() - last_update_time_ > period_);

  if (process_scan) {
    last_update_time_ = reading.GetTime();

    LOG(INFO) << "Start to process scan...";

    const auto& ranges = reading.GetRanges();
    const auto reading_copy = std::make_shared<RangeReading>(reading);
    if (cnt_ == 0) {
      // This is the first scan
      for (auto& particle : particles_) {
        scan_matcher_.InvalidateActiveArea();
        scan_matcher_.ComputeActiveArea(particle.pose, ranges,
                                        &particle.high_map);
        scan_matcher_.RegisterScan(particle.pose, ranges, &particle.high_map);

        // Create the first node for each particle
        CHECK(particle.node->parent == nullptr);
        auto node = std::make_shared<Node>(particle.pose, 0., particle.node, 0);
        node->reading = reading_copy;
        particle.node = node;
      }
    } else {
      // This is not the first scan

      // For each particle, do scan matching and compute
      // 1) best pose
      // 2) score of best pose
      // 3) likelihood of best pose
      // 4) weight
      // 5) valid area
      ScanMatch(ranges);

      // Update weights along trajectory of each particle
      UpdateTreeWeights(false);

      // Resampling according to num_eff and update map
      Resample(ranges, adapt_particles, reading_copy);
    }

    UpdateTreeWeights(false);

    // Reset accumulated distances cuz we have updated
    acc_linear_distance_ = 0.;
    acc_angular_distance_ = 0.;

    ++cnt_;

    // Be ready for next process
    for (auto& particle : particles_) {
      particle.prev_pose = particle.pose;
    }

    LOG(INFO) << "Finished processing scan...";
  }

  ++reading_cnt_;

  return process_scan;
}

void GridSlamProcessor::ScanMatch(const std::vector<double>& ranges) {
  for (auto& particle : particles_) {
    // Update pose if possible
    {
      OrientedPoint2d optimized_pose;
      const double score = scan_matcher_.Optimize(
          particle.high_map, particle.pose, ranges, &optimized_pose);

      if (score > minimum_score_) {
        // Successfully optimized pose
        particle.pose = optimized_pose;
      }
    }

    // Recompute particle's weight, i.e., log-likelihood log(p(z|x,m))
    {
      double score = 0.;
      double likelihood = 0.;
      scan_matcher_.ComputeScoreAndLikelihood(particle.high_map, particle.pose,
                                              ranges, &score, &likelihood);

      // Cuz we use log-likelihood, we should use addition here instead of
      // multiplication as presented in paper
      particle.weight += likelihood;
      particle.weight_sum += likelihood;
    }

    // Set up the selective copy of the active area by detaching the areas which
    // will be updated
    scan_matcher_.InvalidateActiveArea();
    scan_matcher_.ComputeActiveArea(particle.pose, ranges, &particle.high_map);
  }
}

void GridSlamProcessor::UpdateTreeWeights(
    const bool weight_already_normalized) {
  if (!weight_already_normalized) {
    Normalize();
  }

  ResetTree();
  PropagateWeights();
}

bool GridSlamProcessor::Resample(const std::vector<double>& ranges,
                                 const size_t adapt_size,
                                 const RangeReadingPtr& reading) {
  bool has_resampled = false;

  // Back up old trajectories, which will be used for adding new nodes
  std::vector<NodePtr> old_nodes;
  old_nodes.reserve(particles_.size());
  for (const auto& particle : particles_) {
    old_nodes.emplace_back(particle.node);
  }

  CHECK_GT(threshold_resample_threshold_, 0.);
  CHECK_LE(threshold_resample_threshold_, 1.);
  if (num_eff_ < threshold_resample_threshold_ * particles_.size()) {
    // We need to resample

    // Indices of particles we want to keep
    particle_indices_ = UniformSampler<double, double>::ResampleIndices(
        particle_weights_, adapt_size);

    std::vector<Particle> resampled_particles;
    resampled_particles.reserve(particle_indices_.size());

    for (size_t i = 0; i < particle_indices_.size(); ++i) {
      const auto idx = particle_indices_[i];

      auto& particle = particles_.at(idx);

      const auto& old_node = old_nodes.at(idx);
      auto new_node = std::make_shared<Node>(particle.pose, 0., old_node, 0);
      new_node->reading = reading;
      particle.node = new_node;
      particle.prev_idx = idx;

      resampled_particles.emplace_back(particle);
    }

    // Update every particle's weight and map
    particles_.clear();
    particles_.reserve(resampled_particles.size());
    for (auto& particle : resampled_particles) {
      particle.weight = 0;
      scan_matcher_.RegisterScan(particle.pose, ranges, &particle.high_map);

      particles_.emplace_back(particle);
    }

    has_resampled = true;
  } else {
    // No need to resample.
    // We can keep the weights the same and create a new node for trajectories

    for (size_t i = 0; i < particles_.size(); ++i) {
      auto& particle = particles_.at(i);
      auto node_new =
          std::make_shared<Node>(particle.pose, 0., old_nodes.at(i), 0);
      node_new->reading = reading;
      particle.node = node_new;

      // Update particle's map
      scan_matcher_.InvalidateActiveArea();
      scan_matcher_.RegisterScan(particle.pose, ranges, &particle.high_map);

      particle.prev_idx = i;
    }
  }

  return has_resampled;
}

void GridSlamProcessor::Normalize() {
  CHECK_GT(particles_.size(), 0);

  // max weight is namely max likelihood
  double max_weight = std::numeric_limits<double>::lowest();
  for (const auto& particle : particles_) {
    if (particle.weight > max_weight) {
      max_weight = particle.weight;
    }
  }

  // Assume that particle weights follow a Gauss with mean=max_weight
  particle_weights_.clear();
  double sum_weight = 0.;

  CHECK_GT(likelihood_gain_, 1e-6);
  const double gain = 1. / (likelihood_gain_ * particles_.size());
  for (const auto& particle : particles_) {
    // Likelihood
    const double weight = exp(gain * (particle.weight - max_weight));
    particle_weights_.emplace_back(weight);
    sum_weight += weight;
  }
  CHECK_GT(sum_weight, 1e-6);

  // Compute number of effective particles and normalize weights
  double tmp = 0.;  // accumulate w*w
  for (auto& weight : particle_weights_) {
    weight /= sum_weight;
    tmp += weight * weight;
  }
  CHECK_GT(tmp, 1e-6);
  num_eff_ = 1. / tmp;
}

void GridSlamProcessor::ResetTree() {
  for (auto& particle : particles_) {
    auto node = particle.node;
    while (node) {
      node->acc_weight = 0.;
      node->visit_counter = 0;
      node = node->parent;
    }
  }
}

void GridSlamProcessor::PropagateWeights() {
  CHECK_EQ(particle_weights_.size(), particles_.size());
  for (size_t i = 0; i < particles_.size(); ++i) {
    const auto& particle = particles_.at(i);
    const auto& weight = particle_weights_.at(i);

    auto node = particle.node;
    node->acc_weight = weight;
    PropagateWeight(node->acc_weight, node->parent);
  }
}

void GridSlamProcessor::PropagateWeight(const double acc_weight,
                                        const NodePtr& node) {
  if (!node) {
    return;
  }

  ++node->visit_counter;
  node->acc_weight += acc_weight;

  if (node->visit_counter == node->visit_counter) {
    // Once we have accumulated all children's acc_weight, we will propagate to
    // the parent node.
    PropagateWeight(node->acc_weight, node->parent);
  }
}

}  // namespace gmapping
