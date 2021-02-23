#pragma once

#include "grid/array_2d.h"

#include <iostream>
#include <memory>
#include <set>

#include <glog/logging.h>

#include "grid/map.h"
#include "grid/point_accumulator.h"

namespace gmapping {

template <typename Cell>
class HierarchicalArray2D : public Array2D<Array2D<Cell>> {
 public:
  using PointSet = std::set<Point2i, PointComparator<int>>;
  using CellPtr = std::shared_ptr<Cell>;

 public:
  HierarchicalArray2D(const int size_x, const int size_y,
                      const int patch_magnitude = 5)
      : Array2D<Array2D<Cell>>::Array2D(size_x >> patch_magnitude,
                                        size_y >> patch_magnitude),
        patch_magnitude_(patch_magnitude) {
    for (size_t x = 0; x < this->cells_.size(); ++x) {
      for (size_t y = 0; y < this->cells_.at(x).size(); ++y) {
        this->cells_.at(x).at(y) = std::make_shared<Array2D<Cell>>(
            1 << patch_magnitude_, 1 << patch_magnitude_);
      }
    }
  }

  virtual ~HierarchicalArray2D() = default;

  int GetPatchSize() const { return patch_size_; }
  int GetPatchMagnitude() const { return patch_magnitude_; }

  const CellPtr& GetCell(const int x, const int y) const {
    CHECK(IsAllocated(x, y));
    const Point2i p1 = GetPatchIndices(x, y);
    const std::shared_ptr<Array2D<Cell>>& ptr = this->cells_.at(p1.x).at(p1.y);

    const Point2i p2(x - (p1.x << patch_magnitude_),
                     y - (p1.y << patch_magnitude_));
    return ptr->GetCell(p2);
  }

  CellPtr& GetCell(const int x, const int y) {
    const Point2i p1 = GetPatchIndices(x, y);

    CHECK(this->IsInside(p1.x, p1.y))
        << "x: " << x << " y: " << y << " p.x: " << p1.x << " p.y: " << p1.y
        << " size_x: " << this->size_x_ << " size_y: " << this->size_y_;

    if (!this->cells_.at(p1.x).at(p1.y)) {
      this->cells_.at(p1.x).at(p1.y) = std::make_shared<Array2D<Cell>>(
          1 << patch_magnitude_, 1 << patch_magnitude_);
    }

    const auto& ptr = this->cells_.at(p1.x).at(p1.y);
    CHECK(ptr);

    const Point2i p2(x - (p1.x << patch_magnitude_),
                     y - (p1.y << patch_magnitude_));

    return ptr->GetCell(p2);
  }

  const CellPtr& GetCell(const Point2i& p) const { return GetCell(p.x, p.y); }

  CellPtr& GetCell(const Point2i& p) { return GetCell(p.x, p.y); }

  void AllocateActiveArea() {
    for (auto it = active_area_.begin(); it != active_area_.end(); ++it) {
      auto& cell = this->cells_.at(it->x).at(it->y);
      if (!cell) {
        cell.reset();
        cell = std::make_shared<Array2D<Cell>>(1 << patch_magnitude_,
                                               1 << patch_magnitude_);
      }
    }
  }

  void Resize(const double xmin, const double ymin, const double xmax,
              const double ymax) {
    const int size_x = xmax - xmin;
    const int size_y = ymax - ymin;

    std::vector<std::vector<std::shared_ptr<Array2D<Cell>>>> new_cells(size_x);
    for (int x = 0; x < size_x; ++x) {
      new_cells.at(x).resize(size_y, nullptr);
    }

    // Copy previous data into the new map
    const int beg_x = xmin < 0 ? 0 : xmin;
    const int beg_y = ymin < 0 ? 0 : ymin;
    const int end_x = xmax < this->size_x_ ? xmax : this->size_x_;
    const int end_y = ymax < this->size_y_ ? ymax : this->size_y_;

    for (int x = beg_x; x < end_x; ++x) {
      for (int y = beg_y; y < end_y; ++y) {
        new_cells.at(x - xmin).at(y - ymin) = this->cells_.at(x).at(y);
      }
    }
    this->cells_ = new_cells;
    this->size_x_ = size_x;
    this->size_y_ = size_y;
  }

  void SetActiveArea(const PointSet& point_set,
                     const bool patch_coord = false) {
    active_area_.clear();
    for (const auto& p : point_set) {
      const Point2i pi = patch_coord ? p : GetPatchIndices(p);
      active_area_.emplace(pi);
    }
  }

  bool IsAllocated(const int x, const int y) const {
    const Point2i p = GetPatchIndices(x, y);
    return this->cells_.at(p.x).at(p.y) != nullptr;
  }

  Point2i GetPatchIndices(const int x, const int y) const {
    if (x >= 0 && y >= 0) {
      return Point2i(x >> patch_magnitude_, y >> patch_magnitude_);
    }
    return Point2i(-1, -1);
  }

  Point2i GetPatchIndices(const Point2i& p) const {
    return GetPatchIndices(p.x, p.y);
  }

 protected:
  int patch_magnitude_;

  /**
   * @brief Number of cells in one patch
   * @details patch size = 1 << patch_magnitude
   */
  int patch_size_;

  /** Cells' coordinates for map storage */
  PointSet active_area_;
};

using ScanMatcherMap =
    Map<PointAccumulator, HierarchicalArray2D<PointAccumulator>>;

}  // namespace gmapping
