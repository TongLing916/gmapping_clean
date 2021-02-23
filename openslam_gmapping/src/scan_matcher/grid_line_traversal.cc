#include "scan_matcher/grid_line_traversal.h"

#include <cmath>

namespace gmapping {

std::vector<Point2i> GridLineTraversal::GridLine(const Point2i& start,
                                                 const Point2i& end) {
  std::vector<Point2i> points;

  int incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

  const int dx = std::abs(end.x - start.x);
  const int dy = std::abs(end.y - start.y);

  if (dy <= dx) {
    d = 2 * dy - dx;
    incr1 = 2 * dy;
    incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x;
      y = end.y;
      ydirflag = (-1);
      xend = start.x;
    } else {
      x = start.x;
      y = start.y;
      ydirflag = 1;
      xend = end.x;
    }

    points.emplace_back(x, y);
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
        x++;
        if (d < 0) {
          d += incr1;
        } else {
          y++;
          d += incr2;
        }
        points.emplace_back(x, y);
      }
    } else {
      while (x < xend) {
        x++;
        if (d < 0) {
          d += incr1;
        } else {
          y--;
          d += incr2;
        }
        points.emplace_back(x, y);
      }
    }
  } else {
    d = 2 * dx - dy;
    incr1 = 2 * dx;
    incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y;
      x = end.x;
      yend = start.y;
      xdirflag = (-1);
    } else {
      y = start.y;
      x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    points.emplace_back(x, y);
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
        y++;
        if (d < 0) {
          d += incr1;
        } else {
          x++;
          d += incr2;
        }
        points.emplace_back(x, y);
      }
    } else {
      while (y < yend) {
        y++;
        if (d < 0) {
          d += incr1;
        } else {
          x--;
          d += incr2;
        }
        points.emplace_back(x, y);
      }
    }
  }

  return points;
}

}  // namespace gmapping
