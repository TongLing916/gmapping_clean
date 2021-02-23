#include "sensor/range_reading.h"

namespace gmapping {

RangeReading::RangeReading(const RangeSensorConstPtr& sensor, const double time)
    : SensorReading(sensor, time) {}

RangeReading::RangeReading(const RangeSensorConstPtr& sensor,
                           const std::vector<double>& ranges, const double time)
    : SensorReading(sensor, time), ranges_(ranges) {}

const std::vector<double>& RangeReading::GetRanges() const { return ranges_; }

OrientedPoint2d RangeReading::GetPose() const { return pose_; }

void RangeReading::SetPose(const OrientedPoint2d& pose) { pose_ = pose; }

}  // namespace gmapping
