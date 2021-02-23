#pragma once

#include "type/point.h"

namespace gmapping {

/** Class representing a cell in the map */
class PointAccumulator {
 public:
  PointAccumulator();

  double GetOccupancyProb() const;

  /**
   * @brief Update the status of the cell
   *
   * @param is_hit - Is this cell hit?
   * @param p      - The hit position in world system
   */
  void Update(const bool is_hit, const Point2d& p = Point2d(0, 0));

  double GetEntropy() const;

  Point2d Mean() const { return 1. / num_hits_ * Point2d(pt_.x, pt_.y); }

 private:
  /** Accumulated cell position */
  Point2f pt_;

  int num_hits_;
  int visits_;

  static constexpr int kSightInc_ = 1;
};

}  // namespace gmapping
