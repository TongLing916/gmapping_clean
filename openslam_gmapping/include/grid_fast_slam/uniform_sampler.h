#pragma once

#include <vector>

#include <glog/logging.h>

namespace gmapping {

template <class Particle, class Numeric>
class UniformSampler {
 public:
  /**
   * @brief Resample according to weights
   *
   * @param particles             - Particles (Weights)
   * @param num_particles_to_keep - Number we want to keep (0 means the same)
   */
  static std::vector<size_t> ResampleIndices(
      const std::vector<Particle>& particles,
      const size_t num_particles_to_keep = 0) {
    Numeric sum_weight = 0;

    // Compute cumulative weights
    for (const auto& particle : particles) {
      sum_weight += static_cast<Numeric>(particle);
    }

    size_t num_particles = particles.size();
    if (num_particles_to_keep > 0) {
      num_particles = num_particles_to_keep;
    }

    // Compute the interval
    CHECK_GT(num_particles, 0);
    const Numeric interval = sum_weight / num_particles;

    // Compute the initial target weight
    Numeric target = interval * drand48();

    // Sampling according to weights
    // Every time we increment one interval (Check <<Probabilistic Robotics>>)
    sum_weight = 0.;
    std::vector<size_t> indices(num_particles);
    size_t idx = 0;
    for (size_t i = 0; i < particles.size(); ++i) {
      sum_weight += static_cast<Numeric>(particles[i]);
      while (sum_weight > target) {
        CHECK_LT(idx, indices.size());
        indices.at(idx++) = i;
        target += interval;
      }
    }

    return indices;
  }
};

}  // namespace gmapping
