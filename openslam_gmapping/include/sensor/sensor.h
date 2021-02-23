#pragma once

#include <map>
#include <memory>
#include <string>

namespace gmapping {

class Sensor {
 public:
  Sensor(const std::string& name = "") : name_(name) {}
  virtual ~Sensor() = default;
  const std::string GetName() const { return name_; }
  void SetName(const std::string& name) { name_ = name; }

 private:
  std::string name_;
};

using SensorConstPtr = std::shared_ptr<const Sensor>;
using SensorMap = std::map<std::string, std::shared_ptr<Sensor>>;

}  // namespace gmapping
