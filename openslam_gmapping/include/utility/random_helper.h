#pragma once

#include <random>

namespace gmapping {

class RandomHelper {
 public:
  static double SampleGaussian(const double sigma, const size_t seed = 0);

 private:
  /**
   * @brief Draw randomly from a Gauss distribution with zero-mean and sigma
   * @details We use the polar form of the Box-Muller transformation, explained
   * here: http://www.taygeta.com/random/gaussian.html
   */
  static double PfRanGaussian(const double sigma);
};

}  // namespace gmapping
