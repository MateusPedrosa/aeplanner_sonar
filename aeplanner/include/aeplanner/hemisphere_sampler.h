#ifndef AEPLANNER_HEMISPHERE_SAMPLER_H
#define AEPLANNER_HEMISPHERE_SAMPLER_H

#include <aeplanner/directed_sampler.h>

namespace aeplanner
{

// Hemisphere sampler for U-targets and occ/unk E-targets.
// Generates N_samples poses on a hemisphere of radius R_sample centred at
// target.position + d_standoff * target.normal, with yaw constrained to face
// the target. Collision-free candidates are returned sorted by score.
class HemisphereSampler : public DirectedSampler
{
public:
  std::vector<CandidatePose> sampleCandidates(
      const ScoredTarget&             target,
      const Eigen::Vector4d&          robot_state,
      const std::shared_ptr<la3dm::BGKLOctoMap>& map,
      const Params&                   params) override;

private:
  // Returns true if position p is collision-free (not OCCUPIED in map).
  bool collisionFree(const Eigen::Vector3d& p,
                     const std::shared_ptr<la3dm::BGKLOctoMap>& map) const;

  // Convert (theta, phi) on a hemisphere with pole direction n_hat to world-frame point.
  Eigen::Vector3d sphericalToCart(double theta, double phi,
                                  const Eigen::Vector3d& n_hat) const;
};

}  // namespace aeplanner

#endif  // AEPLANNER_HEMISPHERE_SAMPLER_H
