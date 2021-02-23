#pragma once

namespace gmapping {

template <typename T>
struct Point {
 public:
  Point() : x(0), y(0) {}
  Point(const T _x, const T _y) : x(_x), y(_y) {}

 public:
  T x;
  T y;
};

template <class T>
inline Point<T> operator+(const Point<T>& p1, const Point<T>& p2) {
  return Point<T>(p1.x + p2.x, p1.y + p2.y);
}

template <class T>
inline Point<T> operator-(const Point<T>& p1, const Point<T>& p2) {
  return Point<T>(p1.x - p2.x, p1.y - p2.y);
}

template <class T>
inline Point<T> operator*(const Point<T>& p, const T& v) {
  return Point<T>(p.x * v, p.y * v);
}

template <class T>
inline Point<T> operator*(const T& v, const Point<T>& p) {
  return Point<T>(p.x * v, p.y * v);
}

template <class T>
inline T operator*(const Point<T>& p1, const Point<T>& p2) {
  return p1.x * p2.x + p1.y * p2.y;
}

template <class T>
struct PointComparator {
  bool operator()(const Point<T>& a, const Point<T>& b) const {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  }
};

template <class T>
struct PointRadialComparator {
  Point<T> origin;
  bool operator()(const Point<T>& a, const Point<T>& b) const {
    const Point<T> delta1 = a - origin;
    const Point<T> delta2 = b - origin;
    return (atan2(delta1.y, delta1.x) < atan2(delta2.y, delta2.x));
  }
};

template <class T>
inline Point<T> Max(const Point<T>& p1, const Point<T>& p2) {
  Point<T> p = p1;
  p.x = p.x > p2.x ? p.x : p2.x;
  p.y = p.y > p2.y ? p.y : p2.y;
  return p;
}

template <class T>
inline Point<T> Min(const Point<T>& p1, const Point<T>& p2) {
  Point<T> p = p1;
  p.x = p.x < p2.x ? p.x : p2.x;
  p.y = p.y < p2.y ? p.y : p2.y;
  return p;
}

template <class T, class F>
inline Point<T> Interpolate(const Point<T>& p1, const F& t1, const Point<T>& p2,
                            const F& t2, const F& t3) {
  const F gain = (t3 - t1) / (t2 - t1);
  Point<T> p = p1 + (p2 - p1) * gain;
  return p;
}

template <class T>
inline double EuclideanDist(const Point<T>& p1, const Point<T>& p2) {
  return hypot(p1.x - p2.x, p1.y - p2.y);
}

using Point2i = Point<int>;
using Point2f = Point<float>;
using Point2d = Point<double>;

}  // namespace gmapping
