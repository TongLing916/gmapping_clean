#pragma once

#include "grid/hierarchical_array_2d.h"
#include "grid_fast_slam/node.h"

namespace gmapping {

struct Particle {
 public:
  Particle(const ScanMatcherMap& _high_map, const ScanMatcherMap& _low_map)
      : high_map(_high_map),
        low_map(_low_map),
        pose(0, 0, 0),
        prev_pose(0, 0, 0),
        weight(0),
        weight_sum(0),
        gweight(0),
        prev_idx(0),
        node(nullptr) {}

 public:
  /** High resolution map */
  ScanMatcherMap high_map;

  /** Low resolution map for Correlation Scan Match */
  ScanMatcherMap low_map;

  /** Robot's pose (Odometry) */
  OrientedPoint2d pose;

  /** Previous robot's pose used for computing odometry displacement */
  OrientedPoint2d prev_pose;

  /** The weight (log-likelihood) of the particle */
  double weight;

  /** The cumulative weight (log-likelihood) of the particle */
  double weight_sum;

  double gweight;

  /** The index of the previous particle in the trajectory tree */
  size_t prev_idx;

  /** Entry to the trajectory tree (last pose) */
  NodePtr node;
};

}  // namespace gmapping
