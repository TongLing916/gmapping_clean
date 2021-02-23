#pragma once

#include <vector>

#include "type/point.h"

namespace gmapping {

/** Compute points on the line bewteen two points with Bresenham's algorithm */
class GridLineTraversal {
 public:
  static std::vector<Point2i> GridLine(const Point2i& start,
                                       const Point2i& end);
};

}  // namespace gmapping
