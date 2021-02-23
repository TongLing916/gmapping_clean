#pragma once

#include "sensor/sensor.h"

namespace gmapping {

class OdometrySensor : public Sensor {
 public:
  OdometrySensor(const std::string& name, bool ideal = false);

  bool IsIdeal() const;

 private:
  bool ideal_;
};

}  // namespace gmapping
