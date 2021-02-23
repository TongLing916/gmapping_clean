#include "sensor/odometry_sensor.h"

namespace gmapping {

OdometrySensor::OdometrySensor(const std::string& name, bool ideal)
    : Sensor(name), ideal_(ideal) {}

bool OdometrySensor::IsIdeal() const { return ideal_; }

}  // namespace gmapping
