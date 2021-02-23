#pragma once

#include <memory>

#include "sensor/sensor.h"

namespace gmapping {

class SensorReading {
 public:
  SensorReading(const SensorConstPtr& sensor = nullptr, const double time = 0.)
      : sensor_(sensor), time_(time) {}

  virtual ~SensorReading() = default;

  double GetTime() const { return time_; }

  void SetTime(const double time) { time_ = time; }

  const SensorConstPtr GetSensor() const { return sensor_; }

 private:
  const SensorConstPtr sensor_;
  double time_;
};
}  // namespace gmapping
