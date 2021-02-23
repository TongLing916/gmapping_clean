#include "grid/point_accumulator.h"

#include <math.h>

namespace gmapping {

PointAccumulator::PointAccumulator() : pt_(0, 0), num_hits_(0), visits_(0) {}

double PointAccumulator::GetOccupancyProb() const {
  if (visits_ == 0) {
    return -1;
  }
  return static_cast<double>(num_hits_) * kSightInc_ / visits_;
}

void PointAccumulator::Update(const bool is_hit, const Point2d& p) {
  if (!is_hit) {
    ++visits_;
    return;
  }

  pt_.x += static_cast<float>(p.x);
  pt_.y += static_cast<float>(p.y);
  ++num_hits_;
  visits_ += kSightInc_;
}

double PointAccumulator::GetEntropy() const {
  if (visits_ == 0) {
    return -log(0.5);
  }

  if (num_hits_ == visits_ || num_hits_ == 0) {
    return 0;
  }

  const double x = 1. * num_hits_ * kSightInc_ / visits_;
  return -(x * log(x) + (1. - x) * log(1. - x));
}

}  // namespace gmapping
