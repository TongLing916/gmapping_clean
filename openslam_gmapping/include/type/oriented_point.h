#pragma once

#include <math.h>

#include "type/point.h"

namespace gmapping {
template <typename T, typename A>
struct OrientedPoint : public Point<T> {
 public:
  OrientedPoint() : Point<T>(0, 0), theta(0) {}

  OrientedPoint(const Point<T>& p) {
    this->x = p.x;
    this->y = p.y;
    theta = 0;
  }

  OrientedPoint(const T _x, const T _y, const A _theta)
      : Point<T>(_x, _y), theta(_theta) {}

  /** Ensure theta in [-pi, pi) */
  void Normalize() {
    if (theta >= -M_PI && theta < M_PI) {
      return;
    }

    const int multiplier = static_cast<int>(0.5 * theta / M_PI);
    theta -= multiplier * 2 * M_PI;

    if (theta >= M_PI) {
      theta -= 2. * M_PI;
    } else if (theta < -M_PI) {
      theta += 2. * M_PI;
    }
  }

 public:
  A theta;
};

template <class T, class A>
OrientedPoint<T, A> operator+(const OrientedPoint<T, A>& p1,
                              const OrientedPoint<T, A>& p2) {
  return OrientedPoint<T, A>(p1.x + p2.x, p1.y + p2.y, p1.theta + p2.theta);
}

template <class T, class A>
OrientedPoint<T, A> operator-(const OrientedPoint<T, A>& p1,
                              const OrientedPoint<T, A>& p2) {
  return OrientedPoint<T, A>(p1.x - p2.x, p1.y - p2.y, p1.theta - p2.theta);
}

template <class T, class A>
OrientedPoint<T, A> operator*(const OrientedPoint<T, A>& p, const T& v) {
  return OrientedPoint<T, A>(p.x * v, p.y * v, p.theta * v);
}

template <class T, class A>
OrientedPoint<T, A> operator*(const T& v, const OrientedPoint<T, A>& p) {
  return OrientedPoint<T, A>(p.x * v, p.y * v, p.theta * v);
}

template <class T, class A>
OrientedPoint<T, A> AbsoluteDifference(const OrientedPoint<T, A>& p1,
                                       const OrientedPoint<T, A>& p2) {
  OrientedPoint<T, A> delta = p1 - p2;
  delta.theta = atan2(sin(delta.theta), cos(delta.theta));
  const double s = sin(p2.theta), c = cos(p2.theta);
  return OrientedPoint<T, A>(c * delta.x + s * delta.y,
                             -s * delta.x + c * delta.y, delta.theta);
}

template <class T, class A>
OrientedPoint<T, A> AbsoluteSum(const OrientedPoint<T, A>& p1,
                                const OrientedPoint<T, A>& p2) {
  const double s = sin(p1.theta), c = cos(p1.theta);
  return OrientedPoint<T, A>(c * p2.x - s * p2.y, s * p2.x + c * p2.y,
                             p2.theta) +
         p1;
}

template <class T, class A>
Point<T> AbsoluteSum(const OrientedPoint<T, A>& p1, const Point<T>& p2) {
  const double s = sin(p1.theta), c = cos(p1.theta);
  return Point<T>(c * p2.x - s * p2.y, s * p2.x + c * p2.y) + (Point<T>)p1;
}

template <class T, class A, class F>
inline OrientedPoint<T, A> Interpolate(const OrientedPoint<T, A>& p1,
                                       const F& t1,
                                       const OrientedPoint<T, A>& p2,
                                       const F& t2, const F& t3) {
  const F gain = (t3 - t1) / (t2 - t1);
  OrientedPoint<T, A> p;
  p.x = p1.x + (p2.x - p1.x) * gain;
  p.y = p1.y + (p2.y - p1.y) * gain;
  const double s = sin(p1.theta) + sin(p2.theta) * gain,
               c = cos(p1.theta) + cos(p2.theta) * gain;
  p.theta = atan2(s, c);
  return p;
}

template <class T, class A>
inline double EuclideanDist(const OrientedPoint<T, A>& p1,
                            const OrientedPoint<T, A>& p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

template <class T, class A>
inline double SquareDist(const OrientedPoint<T, A>& p1,
                         const OrientedPoint<T, A>& p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

template <class T, class A>
inline double EuclideanDist(const OrientedPoint<T, A>& p1, const Point<T>& p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

template <class T, class A>
inline double EuclideanDist(const Point<T>& p1, const OrientedPoint<T, A>& p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

using OrientedPoint2d = OrientedPoint<double, double>;

}  // namespace gmapping
