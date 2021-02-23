#pragma once

#include "sensor/sensor.h"

#include <vector>

#include "type/oriented_point.h"

namespace gmapping {

class RangeSensor : public Sensor {
 public:
  struct Beam {
    /** Pose relative to the center of sensor */
    OrientedPoint2d pose;

    /** 0: Line-like beam */
    double span;

    /** Maximum range of the sensor */
    double max_range;

    /** Sinus of the beam */
    double s;

    /** Cosinus of the beam */
    double c;
  };

 public:
  /**
   * @brief Construct a new Range Sensor object
   *
   * @param name      - Sensor name
   * @param num_beams - Number of beams
   * @param angle_inc - Beam's angle increment (resolution)
   * @param pose      - Laser's pose
   * @param span      - Diverging angle (0 for laser, i.e. line-like)
   * @param max_range - Max range of laser beam
   */
  RangeSensor(const std::string& name, const size_t num_beams,
              const double angle_inc,
              const OrientedPoint2d& pose = OrientedPoint2d(0, 0, 0),
              const double span = 0, const double max_range = 89.);

  void UpdateLookUpTable();

  OrientedPoint2d GetPose() const;

  std::vector<double> GetAngles() const;

  size_t CountBeams() const;

 private:
  std::vector<Beam> beams_;

  /** Range sensor's pose wrt. base link */
  OrientedPoint2d pose_;
};

using RangeSensorConstPtr = std::shared_ptr<const RangeSensor>;

}  // namespace gmapping
