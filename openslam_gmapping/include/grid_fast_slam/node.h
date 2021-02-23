#pragma once

#include <memory>

#include "sensor/range_reading.h"
#include "type/oriented_point.h"

namespace gmapping {

/** @brief A node of the trajectory tree */
struct Node {
 public:
  Node(const OrientedPoint2d& _pose, const double _weight,
       const std::shared_ptr<Node>& _parent = nullptr,
       const size_t _children = 0)
      : pose(_pose),
        weight(_weight),
        acc_weight(0),
        gweight(0),
        reading(nullptr),
        parent(_parent),
        children(_children),
        flag(false) {
    if (parent) {
      ++parent->children;
    }
  }

 public:
  /** Robot's pose */
  OrientedPoint2d pose;

  /** The weight of the particle */
  double weight;

  /** The sum of particle weights in the children nodes */
  double acc_weight;

  double gweight;

  /** The range reading to which this node is associated */
  std::shared_ptr<RangeReading> reading;

  /** The node's parent */
  std::shared_ptr<Node> parent;

  /** The number of children */
  size_t children;

  /** The counter in visiting the node (internally used) */
  mutable size_t visit_counter;

  /** Visit flag (internally used) */
  mutable bool flag;
};

using NodePtr = std::shared_ptr<Node>;

}  // namespace gmapping
