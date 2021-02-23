#include "utility/random_helper.h"

#include <math.h>

namespace gmapping {

double RandomHelper::SampleGaussian(const double sigma, const size_t seed) {
  if (seed != 0) {
    srand(seed);
  }
  if (sigma == 0) {
    return 0;
  }

  return PfRanGaussian(sigma);
}

double RandomHelper::PfRanGaussian(const double sigma) {
  double x1, x2, w;
  double r;

  do {
    do {
      r = drand48();
    } while (r == 0.0);
    x1 = 2.0 * r - 1.0;
    do {
      r = drand48();
    } while (r == 0.0);
    x2 = 2.0 * drand48() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w > 1.0 || w == 0.0);

  return (sigma * x2 * sqrt(-2.0 * log(w) / w));
}

}  // namespace gmapping
