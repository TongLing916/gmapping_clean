#include "scan_matcher/scan_matcher.h"

#include <limits>

#include <glog/logging.h>

#include "scan_matcher/grid_line_traversal.h"

namespace gmapping {

ScanMatcher::ScanMatcher()
    : num_beams_(0),
      generate_map_(false),
      ll_sample_range_(0),
      la_sample_range_(0),
      ll_sample_step_(0),
      la_sample_step_(0),
      usable_range_(0),
      max_range_(0),
      kernel_size_(0),
      opt_linear_delta_(0),
      opt_angular_delta_(0),
      opt_recursive_iterations_(0),
      gauss_sigma_(0),
      likelihood_sigma_(0),
      likelihood_skip_(0),
      computed_active_area_(false),
      initial_beams_skip_(0),
      enlarge_step_(10),
      angular_odometry_reliability(0.),
      linear_odometry_reliability(0.),
      free_cell_ratio_(sqrt(2.)),
      fullness_threshold_(0.1),
      likelihood_miss_(-0.5) {}

void ScanMatcher::SetLaserParamters(const size_t num_beams,
                                    const std::vector<double>& angles,
                                    const OrientedPoint2d& pose) {
  num_beams_ = num_beams;
  angles_ = angles;
  laser_pose_ = pose;
}

void ScanMatcher::SetMaxRange(const double range) { max_range_ = range; }

void ScanMatcher::SetUsableRange(const double range) { usable_range_ = range; }

void ScanMatcher::SetGenerateMap(const bool generate_map) {
  generate_map_ = generate_map;
}

void ScanMatcher::SetTranslationalSamplingRange(const double ll_sample_range) {
  ll_sample_range_ = ll_sample_range;
}

void ScanMatcher::SetAngularSamplingRange(const double la_sample_range) {
  la_sample_range_ = la_sample_range;
}

void ScanMatcher::SetTranslationalSamplingStep(const double ll_sample_step) {
  ll_sample_step_ = ll_sample_step;
}

void ScanMatcher::SetAngularSamplingStep(const double la_sample_step) {
  la_sample_step_ = la_sample_step;
}

void ScanMatcher::SetMatchingParameters(
    const double urange, const double max_range, const double sigma,
    const int kernel_size, const double lopt, const double aopt,
    const size_t iterations, const double likelihhod_sigma,
    const size_t likelihood_skip) {
  usable_range_ = urange;
  max_range_ = max_range;
  gauss_sigma_ = sigma;
  kernel_size_ = kernel_size_;
  opt_linear_delta_ = lopt;
  opt_angular_delta_ = aopt;
  opt_recursive_iterations_ = iterations;
  likelihood_sigma_ = likelihhod_sigma;
  likelihood_skip_ = likelihood_skip;
}

double ScanMatcher::RegisterScan(const OrientedPoint2d& pose,
                                 const std::vector<double>& ranges,
                                 ScanMatcherMap* const scan_matcher_map) {
  CHECK_NOTNULL(scan_matcher_map);

  ComputeActiveArea(pose, ranges, scan_matcher_map);

  // This operation replicates the cells that will be changed in the
  // registration operation
  scan_matcher_map->GetStorage().AllocateActiveArea();

  // Convert laser pose from world to map
  OrientedPoint2d laser_in_world = pose;
  laser_in_world.x +=
      cos(pose.theta) * laser_pose_.x - sin(pose.theta) * laser_pose_.y;
  laser_in_world.y +=
      sin(pose.theta) * laser_pose_.x + cos(pose.theta) * laser_pose_.y;
  laser_in_world.theta += laser_pose_.theta;
  const double& theta = laser_in_world.theta;
  const Point2i laser_in_map = scan_matcher_map->World2Map(laser_in_world);

  double sum_entropy_change = 0.;

  for (size_t i = initial_beams_skip_; i < num_beams_; ++i) {
    CHECK_LT(i, ranges.size());
    double range = ranges.at(i);
    const double angle = angles_.at(i);

    if (isnan(range) || range == 0. || range > max_range_) {
      continue;
    }

    if (generate_map_) {
      // Remove invalid beams

      if (range > usable_range_) {
        range = usable_range_;
      }

      // The coordinate of point hit by the beam
      CHECK_LT(i, angles_.size());

      const Point2d p_hit_in_world =
          laser_in_world +
          Point2d(range * cos(theta + angle), range * sin(theta + angle));
      const Point2i p_hit_in_map = scan_matcher_map->World2Map(p_hit_in_world);

      const auto points =
          GridLineTraversal::GridLine(laser_in_map, p_hit_in_map);

      // Update free space
      for (size_t i = 1; i < points.size(); ++i) {
        const auto& pt = points.at(i);

        auto& cell = scan_matcher_map->GetCell(pt);

        // Change = new - old
        double entropy_change = -cell->GetEntropy();
        cell->Update(false, Point2d(0, 0));
        entropy_change += cell->GetEntropy();
        sum_entropy_change += entropy_change;
      }

      // Update hit space
      if (range < usable_range_) {
        auto& cell = scan_matcher_map->GetCell(p_hit_in_map);
        // Change = new - old
        double entropy_change = -cell->GetEntropy();
        cell->Update(true, p_hit_in_world);
        entropy_change += cell->GetEntropy();
        sum_entropy_change += entropy_change;
      }
    } else {
      const double& range = ranges.at(i);
      if (range > usable_range_) {
        continue;
      }

      const Point2d p_hit_in_world =
          laser_in_world +
          Point2d(range * cos(theta + angle), range * sin(theta + angle));
      const Point2i p_hit_in_map = scan_matcher_map->World2Map(p_hit_in_world);
      CHECK_GE(p_hit_in_map.x, 0);
      CHECK_GE(p_hit_in_map.y, 0);

      auto& cell = scan_matcher_map->GetCell(p_hit_in_map);
      CHECK(cell);
      cell->Update(true, p_hit_in_world);
    }
  }
  return sum_entropy_change;
}

void ScanMatcher::InvalidateActiveArea() { computed_active_area_ = false; }

void ScanMatcher::ComputeActiveArea(const OrientedPoint2d& pose,
                                    const std::vector<double>& ranges,
                                    ScanMatcherMap* const scan_matcher_map) {
  CHECK_NOTNULL(scan_matcher_map);

  if (computed_active_area_) {
    // Already computed
    return;
  }

  OrientedPoint2d laser_in_world = pose;
  laser_in_world.x +=
      cos(pose.theta) * laser_pose_.x - sin(pose.theta) * laser_pose_.y;
  laser_in_world.y +=
      sin(pose.theta) * laser_pose_.x + cos(pose.theta) * laser_pose_.y;
  laser_in_world.theta += laser_pose_.theta;
  const double& theta = laser_in_world.theta;
  const Point2i laser_in_map = scan_matcher_map->World2Map(laser_in_world);

  // Map range
  Point2d map_min(scan_matcher_map->Map2World(0, 0));
  Point2d map_max(
      scan_matcher_map->Map2World(scan_matcher_map->GetMapSizeX() - 1,
                                  scan_matcher_map->GetMapSizeY() - 1));
  map_min.x = std::min(map_min.x, laser_in_world.x);
  map_min.y = std::min(map_min.y, laser_in_world.y);
  map_max.x = std::max(map_max.x, laser_in_world.x);
  map_max.y = std::max(map_max.y, laser_in_world.y);

  // Determine the area size
  for (size_t i = initial_beams_skip_; i < num_beams_; ++i) {
    double range = ranges.at(i);
    const double angle = angles_.at(i);
    if (isnan(range) || range == 0. || range > max_range_) {
      continue;
    }

    if (range > usable_range_) {
      range = usable_range_;
    }

    const Point2d p_hit_in_world =
        laser_in_world +
        Point2d(range * cos(theta + angle), range * sin(theta + angle));

    // Extend ranges
    map_min.x = std::min(map_min.x, p_hit_in_world.x);
    map_min.y = std::min(map_min.y, p_hit_in_world.y);
    map_max.x = std::max(map_max.x, p_hit_in_world.x);
    map_max.y = std::max(map_max.y, p_hit_in_world.y);
  }

  // Extend map if necessary
  if (!scan_matcher_map->IsInside(map_min) ||
      !scan_matcher_map->IsInside(map_max)) {
    const Point2d cur_min(scan_matcher_map->Map2World(0, 0));
    const Point2d cur_max(
        scan_matcher_map->Map2World(scan_matcher_map->GetMapSizeX() - 1,
                                    scan_matcher_map->GetMapSizeY() - 1));

    if (map_min.x >= cur_min.x) {
      map_min.x = cur_min.x;
    } else {
      map_min.x -= enlarge_step_;
    }

    if (map_min.y >= cur_min.y) {
      map_min.y = cur_min.y;
    } else {
      map_min.y -= enlarge_step_;
    }

    if (map_max.x <= cur_max.x) {
      map_max.x = cur_max.x;
    } else {
      map_max.x += enlarge_step_;
    }

    if (map_max.y <= cur_max.y) {
      map_max.y = cur_max.y;
    } else {
      map_max.y += enlarge_step_;
    }

    scan_matcher_map->Resize(map_min.x, map_min.y, map_max.x, map_max.y);
  }

  auto& storage = scan_matcher_map->GetStorage();
  HierarchicalArray2D<PointAccumulator>::PointSet active_area;
  for (size_t i = initial_beams_skip_; i < num_beams_; ++i) {
    CHECK_LT(i, ranges.size());
    double range = ranges.at(i);
    const double angle = angles_.at(i);

    if (isnan(range) || range == 0. || range > max_range_) {
      continue;
    }

    if (generate_map_) {
      // Remove invalid beams

      if (range > usable_range_) {
        range = usable_range_;
      }

      // The coordinate of point hit by the beam
      CHECK_LT(i, angles_.size());

      const Point2d p_hit_in_world =
          laser_in_world +
          Point2d(range * cos(theta + angle), range * sin(theta + angle));
      const Point2i p_hit_in_map = scan_matcher_map->World2Map(p_hit_in_world);

      const auto points =
          GridLineTraversal::GridLine(laser_in_map, p_hit_in_map);

      // Update free space
      for (const auto& pt : points) {
        active_area.emplace(storage.GetPatchIndices(pt));
      }

      // Update hit space
      if (range < usable_range_) {
        active_area.emplace(storage.GetPatchIndices(p_hit_in_map));
      }
    } else {
      const double& range = ranges.at(i);
      if (range > usable_range_) {
        continue;
      }

      const Point2d p_hit_in_world =
          laser_in_world +
          Point2d(range * cos(theta + angle), range * sin(theta + angle));
      const Point2i p_hit_in_map = scan_matcher_map->World2Map(p_hit_in_world);

      const Point2i p_hit_patch = storage.GetPatchIndices(p_hit_in_map);

      CHECK_GE(p_hit_patch.x, 0);
      CHECK_GE(p_hit_patch.y, 0);
      active_area.emplace(p_hit_patch);
    }
  }

  storage.SetActiveArea(active_area);
  computed_active_area_ = true;
}

double ScanMatcher::Optimize(const ScanMatcherMap& scan_matcher_map,
                             const OrientedPoint2d& init_pose,
                             const std::vector<double>& ranges,
                             OrientedPoint2d* const optimized_pose) {
  CHECK_NOTNULL(optimized_pose);

  double best_score = -1.;

  OrientedPoint2d cur_pose = init_pose;
  double cur_score = ComputeScore(scan_matcher_map, cur_pose, ranges);

  double angular_delta = opt_angular_delta_;
  double linear_delta = opt_linear_delta_;

  size_t num_refinements = 0;

  do {
    // If current score is not better than last best score, we will decrease
    // step size
    if (cur_score <= best_score) {
      ++num_refinements;
      angular_delta *= 0.5;
      linear_delta *= 0.5;
    }

    best_score = cur_score;
    auto best_local_pose = cur_pose;

    Move move = kFront;
    while (move != kDone) {
      auto local_pose = cur_pose;
      switch (move) {
        case kFront: {
          cur_pose.x += linear_delta;
          move = kBack;
          break;
        }

        case kBack: {
          cur_pose.x -= linear_delta;
          move = kLeft;
          break;
        }
        case kLeft: {
          cur_pose.y -= linear_delta;
          move = kRight;
          break;
        }

        case kRight: {
          cur_pose.y += linear_delta;
          move = kTurnLeft;
          break;
        }

        case kTurnLeft: {
          cur_pose.theta += angular_delta;
          move = kTurnRight;
          break;
        }

        case kTurnRight: {
          cur_pose.theta -= angular_delta;
          move = kDone;
          break;
        }

        default:
          break;
      }

      // The larger the difference between current pose and initial pose is, the
      // less the gain is.
      double odom_gain = 1.;

      if (angular_odometry_reliability > 0.) {
        double diff_theta = init_pose.theta - local_pose.theta;
        diff_theta = atan2(sin(diff_theta), cos(diff_theta));
        diff_theta *= diff_theta;
        odom_gain *= exp(-angular_odometry_reliability * diff_theta);
      }

      if (linear_odometry_reliability > 0.) {
        const double dx = init_pose.x - local_pose.x;
        const double dy = init_pose.y - local_pose.y;
        const double drho = dx * dx + dy * dy;
        odom_gain *= exp(-linear_odometry_reliability * drho);
      }

      const double local_score =
          odom_gain * ComputeScore(scan_matcher_map, local_pose, ranges);

      if (local_score > cur_score) {
        cur_score = local_score;
        best_local_pose = local_pose;
      }
    }

    cur_pose = best_local_pose;
  } while (cur_score > best_score ||
           num_refinements < opt_recursive_iterations_);

  *optimized_pose = cur_pose;
  return best_score;
}

size_t ScanMatcher::ComputeScoreAndLikelihood(
    const ScanMatcherMap& scan_matcher_map, const OrientedPoint2d& pose,
    const std::vector<double>& ranges, double* const score,
    double* const log_likelihood) {
  CHECK_NOTNULL(score);
  CHECK_NOTNULL(log_likelihood);

  *score = 0.;
  *log_likelihood = 0.;

  OrientedPoint2d laser_in_world = pose;
  laser_in_world.x +=
      cos(pose.theta) * laser_pose_.x - sin(pose.theta) * laser_pose_.y;
  laser_in_world.y +=
      sin(pose.theta) * laser_pose_.x + cos(pose.theta) * laser_pose_.y;
  laser_in_world.theta += laser_pose_.theta;
  const double& theta = laser_in_world.theta;

  CHECK_GT(likelihood_sigma_, 1e-6);
  const double likelihood_no_hit = likelihood_miss_ / likelihood_sigma_;

  size_t num_valid_beams = 0;
  size_t skip = 0;
  const double free_delta = scan_matcher_map.GetResolution() * free_cell_ratio_;
  for (size_t i = initial_beams_skip_; i < num_beams_; ++i) {
    ++skip;
    if (skip > likelihood_skip_) {
      skip = 0;
    }
    if (skip) {
      continue;
    }

    const double& range = ranges.at(i);
    if (range > usable_range_) {
      continue;
    }

    const double& angle = angles_.at(i);

    const Point2d p_hit_in_world =
        laser_in_world +
        Point2d(range * cos(theta + angle), range * sin(theta + angle));
    const Point2i p_hit_in_map = scan_matcher_map.World2Map(p_hit_in_world);

    // The last free point along the beam
    const double range_free = range - free_delta;
    const Point2d p_free_in_world =
        laser_in_world + Point2d(range_free * cos(theta + angle),
                                 range_free * sin(theta + angle));

    const Point2d delta_p_in_world = p_free_in_world - p_hit_in_world;
    const Point2i delta_p_in_map = scan_matcher_map.World2Map(delta_p_in_world);

    // Search the most probably hit point inside the search window
    bool found = false;
    double min_rel_dist = std::numeric_limits<double>::max();
    for (int dx = -kernel_size_; dx <= kernel_size_; ++dx) {
      for (int dy = -kernel_size_; dy <= kernel_size_; ++dy) {
        const Point2i p_hit_tmp = p_hit_in_map + Point2i(dx, dy);
        const Point2i p_free_tmp = p_hit_tmp + delta_p_in_map;

        const auto& cell_hit = scan_matcher_map.GetCell(p_hit_tmp);
        const auto& cell_free = scan_matcher_map.GetCell(p_free_tmp);
        if (cell_hit->GetOccupancyProb() > fullness_threshold_ &&
            cell_free->GetOccupancyProb() < fullness_threshold_) {
          const Point2d rel_pos = p_hit_in_world - cell_hit->Mean();
          const double rel_dist = rel_pos * rel_pos;
          if (!found) {
            min_rel_dist = rel_dist;
            found = true;
          } else {
            if (rel_dist < min_rel_dist) {
              min_rel_dist = rel_dist;
            }
          }
        }
      }
    }
    if (found) {
      CHECK_GT(gauss_sigma_, 1e-6);
      *score += exp(-min_rel_dist / gauss_sigma_);
      ++num_valid_beams;

      *log_likelihood += -min_rel_dist / likelihood_sigma_;
    } else {
      // No hit (miss)
      *log_likelihood += likelihood_no_hit;
    }
  }

  return num_valid_beams;
}

double ScanMatcher::ComputeScore(const ScanMatcherMap& scan_matcher_map,
                                 const OrientedPoint2d& pose,
                                 const std::vector<double>& ranges) const {
  double score = 0.;

  OrientedPoint2d laser_in_world = pose;
  laser_in_world.x +=
      cos(pose.theta) * laser_pose_.x - sin(pose.theta) * laser_pose_.y;
  laser_in_world.y +=
      sin(pose.theta) * laser_pose_.x + cos(pose.theta) * laser_pose_.y;
  laser_in_world.theta += laser_pose_.theta;
  const double& theta = laser_in_world.theta;

  size_t skip = 0;
  const double free_delta = scan_matcher_map.GetResolution() * free_cell_ratio_;
  for (size_t i = initial_beams_skip_; i < num_beams_; ++i) {
    ++skip;
    if (skip > likelihood_skip_) {
      skip = 0;
    }
    if (skip) {
      continue;
    }

    const double& range = ranges.at(i);
    if (range > usable_range_) {
      continue;
    }

    const double& angle = angles_.at(i);

    const Point2d p_hit_in_world =
        laser_in_world +
        Point2d(range * cos(theta + angle), range * sin(theta + angle));
    const Point2i p_hit_in_map = scan_matcher_map.World2Map(p_hit_in_world);

    // The last free point along the beam
    const double range_free = range - free_delta;
    const Point2d p_free_in_world =
        laser_in_world + Point2d(range_free * cos(theta + angle),
                                 range_free * sin(theta + angle));

    const Point2d delta_p_in_world = p_free_in_world - p_hit_in_world;
    const Point2i delta_p_in_map = scan_matcher_map.World2Map(delta_p_in_world);

    // Search the most probably hit point inside the search window
    bool found = false;
    double min_rel_dist = std::numeric_limits<double>::max();
    for (int dx = -kernel_size_; dx <= kernel_size_; ++dx) {
      for (int dy = -kernel_size_; dy <= kernel_size_; ++dy) {
        const Point2i p_hit_tmp = p_hit_in_map + Point2i(dx, dy);
        const Point2i p_free_tmp = p_hit_tmp + delta_p_in_map;

        const auto& cell_hit = scan_matcher_map.GetCell(p_hit_tmp);
        const auto& cell_free = scan_matcher_map.GetCell(p_free_tmp);
        if (cell_hit->GetOccupancyProb() > fullness_threshold_ &&
            cell_free->GetOccupancyProb() < fullness_threshold_) {
          const Point2d rel_pos = p_hit_in_world - cell_hit->Mean();
          const double rel_dist = rel_pos * rel_pos;
          if (!found) {
            min_rel_dist = rel_dist;
            found = true;
          } else {
            if (rel_dist < min_rel_dist) {
              min_rel_dist = rel_dist;
            }
          }
        }
      }
    }
    if (found) {
      CHECK_GT(gauss_sigma_, 1e-6);
      score += exp(-min_rel_dist / gauss_sigma_);
    }
  }

  return score;
}

}  // namespace gmapping
