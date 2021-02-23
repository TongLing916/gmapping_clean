#include "sensor/range_sensor.h"

#include <math.h>

namespace gmapping {

RangeSensor::RangeSensor(const std::string& name, const size_t num_beams,
                         const double angle_inc, const OrientedPoint2d& pose,
                         const double span, const double max_range)
    : Sensor(name), beams_(num_beams), pose_(pose) {
  double angle = -0.5 * angle_inc * num_beams;
  for (size_t i = 0; i < num_beams; ++i) {
    auto& beam = beams_.at(i);

    beam.span = span;
    beam.pose.x = 0;
    beam.pose.y = 0;
    beam.pose.theta = angle;
    beam.max_range = max_range;

    angle += angle_inc;
  }

  UpdateLookUpTable();
}

void RangeSensor::UpdateLookUpTable() {
  for (size_t i = 0; i < beams_.size(); ++i) {
    auto& beam = beams_.at(i);
    beam.s = sin(beam.pose.theta);
    beam.c = cos(beam.pose.theta);
  }
}

OrientedPoint2d RangeSensor::GetPose() const { return pose_; }

size_t RangeSensor::CountBeams() const { return beams_.size(); }

std::vector<double> RangeSensor::GetAngles() const {
  std::vector<double> angles(beams_.size());
  for (size_t i = 0; i < beams_.size(); ++i) {
    angles.at(i) = beams_.at(i).pose.theta;
  }
  return angles;
}

}  // namespace gmapping
