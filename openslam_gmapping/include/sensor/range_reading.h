#pragma once

#include "sensor_reading.h"

#include <memory>

#include "sensor/range_sensor.h"

namespace gmapping {

class RangeReading : public SensorReading {
 public:
  RangeReading(const RangeSensorConstPtr& sensor, const double time = 0);

  RangeReading(const RangeSensorConstPtr& sensor,
               const std::vector<double>& ranges, const double time = 0);

  const std::vector<double>& GetRanges() const;

  OrientedPoint2d GetPose() const;
  void SetPose(const OrientedPoint2d& pose);

 private:
  std::vector<double> ranges_;
  std::vector<double> angles_;

  OrientedPoint2d pose_;
};

using RangeReadingPtr = std::shared_ptr<RangeReading>;

}  // namespace gmapping
