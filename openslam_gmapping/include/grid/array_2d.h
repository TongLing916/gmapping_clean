#pragma once

#include <memory>
#include <vector>

#include <glog/logging.h>

#include "grid/access_state.h"
#include "type/point.h"

namespace gmapping {

template <typename Cell, const bool debug = false>
class Array2D {
 public:
  using CellPtr = std::shared_ptr<Cell>;

 public:
  Array2D(const int size_x = 0, const int size_y = 0)
      : size_x_(size_x), size_y_(size_y) {
    if (size_x_ > 0 && size_y_ > 0) {
      cells_.resize(size_x);
      for (int i = 0; i < size_x; ++i) {
        for (int j = 0; j < size_y; ++j) {
          cells_[i].emplace_back(std::make_shared<Cell>());
        }
      }
    } else {
      size_x_ = size_y_ = 0;
    }
  }

  virtual ~Array2D() = default;

  int GetSizeX() const { return size_x_; }
  int GetSizeY() const { return size_y_; }

  bool IsInside(const int x, const int y) const {
    return x >= 0 && y >= 0 && x < size_x_ && y < size_y_;
  }

  AccessState CellState(const int x, const int y) const {
    if (IsInside(x, y)) {
      return static_cast<AccessState>(kInside | kAllocated);
    }
    return kOutside;
  }

  AccessState CellState(const Point2i& p) const { return CellState(p.x, p.y); }

  const CellPtr& GetCell(const int x, const int y) const {
    CHECK(IsInside(x, y)) << "x: " << x << ", y: " << y
                          << ", size_x_: " << size_x_
                          << ", size_y_: " << size_y_;
    return cells_.at(x).at(y);
  }

  CellPtr& GetCell(const int x, const int y) {
    CHECK(IsInside(x, y)) << "x: " << x << ", y: " << y
                          << ", size_x_: " << size_x_
                          << ", size_y_: " << size_y_;
    return cells_.at(x).at(y);
  }

  const CellPtr& GetCell(const Point2i& p) const { return GetCell(p.x, p.y); }

  CellPtr& GetCell(const Point2i& p) { return GetCell(p.x, p.y); }

 public:
  int size_x_;
  int size_y_;
  std::vector<std::vector<CellPtr>> cells_;
};

}  // namespace gmapping
