#pragma once

#include <math.h>
#include <memory>

#include "type/point.h"

namespace gmapping {

template <typename Cell, typename Storage, const bool is_class = true>
class Map {
 public:
  using CellPtr = std::shared_ptr<Cell>;

 public:
  /**
   * @brief Construct a new Map object
   *
   * @param map_size_x - Map size
   * @param map_size_y - Map size
   * @param resolution - Map resolution
   */
  Map(const int map_size_x, const int map_size_y, const double resolution)
      : world_size_x_(map_size_x * resolution),
        world_size_y_(map_size_y * resolution),
        storage_(map_size_x, map_size_y_),
        resolution_(resolution),
        center_(Point2d(0.5 * world_size_x_, 0.5 * world_size_y_)),
        map_size_x_(map_size_x),
        map_size_y_(map_size_y),
        map_size_x_half_(map_size_x >> 1),
        map_size_y_half_(map_size_y >> 1) {}

  /**
   * @brief Construct a new Map object
   *
   * @param center       - Map center
   * @param world_size_x - World size
   * @param world_size_y - World size
   * @param resolution   - Map resolution
   */
  Map(const Point2d& center, const double world_size_x,
      const double world_size_y, const double resolution)
      : world_size_x_(world_size_x),
        world_size_y_(world_size_y),
        storage_(ceil(world_size_x_ / resolution),
                 ceil(world_size_y_ / resolution)),
        resolution_(resolution),
        center_(center),
        map_size_x_(storage_.GetSizeX() << storage_.GetPatchMagnitude()),
        map_size_y_(storage_.GetSizeY() << storage_.GetPatchMagnitude()),
        map_size_x_half_(map_size_x_ >> 1),
        map_size_y_half_(map_size_y_ >> 1) {}

  /** @brief Construct a new Map object */
  Map(const Point2d& center, const double world_xmin, const double world_ymin,
      const double world_xmax, const double world_ymax, const double resolution)
      : world_size_x_(world_xmax - world_xmin),
        world_size_y_(world_ymax - world_ymin),
        storage_(ceil(world_size_x_ / resolution),
                 ceil(world_size_y_ / resolution)),
        resolution_(resolution),
        center_(center),
        map_size_x_(storage_.GetSizeX() << storage_.GetPatchMagnitude()),
        map_size_y_(storage_.GetSizeY() << storage_.GetPatchMagnitude()),
        map_size_x_half_(map_size_x_ >> 1),
        map_size_y_half_(map_size_y_ >> 1) {}

  int GetMapSizeX() const { return map_size_x_; }
  int GetMapSizeY() const { return map_size_y_; }
  double GetResolution() const { return resolution_; }

  Point2d Map2World(const Point2i& p) const { return Map2World(p.x, p.y); }

  Point2d Map2World(const int x, const int y) const {
    return center_ + Point2d((x - map_size_x_half_) * resolution_,
                             (y - map_size_y_half_) * resolution_);
  }

  Point2i World2Map(const Point2d& p) const { return World2Map(p.x, p.y); }

  Point2i World2Map(const double x, const double y) const {
    return Point2i(round((x - center_.x) / resolution_) + map_size_x_half_,
                   round((y - center_.y) / resolution_) + map_size_y_half_);
  }

  CellPtr& GetCell(const Point2d& p) { return storage_.GetCell(p); }

  const CellPtr& GetCell(const Point2d& p) const {
    const Point2i pi = World2Map(p);
    const auto state = storage_.CellState(pi);
    if (state & kAllocated) {
      return storage_.GetCell(pi);
    }

    return kUnknown_;
  }

  CellPtr& GetCell(const Point2i& p) { return storage_.GetCell(p); }

  const CellPtr& GetCell(const Point2i& p) const {
    const auto state = storage_.CellState(p);
    if (state & kAllocated) {
      return storage_.GetCell(p);
    }

    return kUnknown_;
  }

  Storage& GetStorage() { return storage_; }
  const Storage& GetStorage() const { return storage_; }

  bool IsInside(const double xw, const double yw) const {
    return storage_.CellState(World2Map(xw, yw)) & kInside;
  }

  bool IsInside(const Point2d& p) const { return IsInside(p.x, p.y); }

  void Resize(const double xmin, const double ymin, const double xmax,
              const double ymax) {
    const Point2i map_min = World2Map(xmin, ymin);
    const Point2i map_max = World2Map(xmax, ymax);

    const int magnitude = storage_.GetPatchMagnitude();
    const int pxmin = floor(1. * map_min.x / (1 << magnitude));
    const int pxmax = ceil(1. * map_max.x / (1 << magnitude));
    const int pymin = floor(1. * map_min.y / (1 << magnitude));
    const int pymax = ceil(1. * map_max.y / (1 << magnitude));

    storage_.Resize(pxmin, pymin, pxmax, pymax);

    world_size_x_ = xmax - xmin;
    world_size_y_ = ymax - ymin;

    map_size_x_ = storage_.GetSizeX() << magnitude;
    map_size_y_ = storage_.GetSizeY() << magnitude;
    map_size_x_half_ -= pxmin * (1 << magnitude);
    map_size_y_half_ -= pymin * (1 << magnitude);
  }

  void PrintSize() {
    std::cout << "world_size_x_: " << world_size_x_ << std::endl;
    std::cout << "world_size_y_: " << world_size_y_ << std::endl;
    std::cout << "resolution_: " << resolution_ << std::endl;
    std::cout << "map_size_x_: " << map_size_x_ << std::endl;
    std::cout << "map_size_y_: " << map_size_y_ << std::endl;
    std::cout << "map_size_x_half_: " << map_size_x_half_ << std::endl;
    std::cout << "map_size_y_half_: " << map_size_y_half_ << std::endl;

    std::cout << "storage_.size_x_: " << storage_.size_x_ << std::endl;
    std::cout << "storage_.size_y_: " << storage_.size_y_ << std::endl;
  }

 private:
  /** World size */
  double world_size_x_;

  /** World size */
  double world_size_y_;

  /** Map container */
  Storage storage_;

  /** Map resolution */
  double resolution_;

  /** Map center */
  Point2d center_;

  /** Map size */
  int map_size_x_;

  /** Map size */
  int map_size_y_;

  /** Half map size */
  int map_size_x_half_;

  /** Half map size */
  int map_size_y_half_;

  static const std::shared_ptr<Cell> kUnknown_;
};

template <typename Cell, typename Storage, const bool is_class>
const std::shared_ptr<Cell> Map<Cell, Storage, is_class>::kUnknown_ =
    std::make_shared<Cell>();

}  // namespace gmapping
