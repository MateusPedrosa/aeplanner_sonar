#ifndef AEPLANNER_FRONTIER_SAMPLER_H
#define AEPLANNER_FRONTIER_SAMPLER_H

#include <aeplanner/directed_sampler.h>

namespace aeplanner
{

// Frontier sampler for free/unk E-targets.
// If w_normal >= w_normal_thresh: samples a strip of poses along the frontier
// tangent at centroid + d_standoff * normal.
// If w_normal < w_normal_thresh: depth-biased fallback — samples poses at
// current_depth - dz_step, distributed uniformly in XY within local bounding box.
class FrontierSampler : public DirectedSampler
{
public:
  std::vector<CandidatePose> sampleCandidates(
      const ScoredTarget&             target,
      const Eigen::Vector4d&          robot_state,
      const std::shared_ptr<la3dm::BGKLOctoMap>& map,
      const Params&                   params) override;

private:
  bool collisionFree(const Eigen::Vector3d& p,
                     const std::shared_ptr<la3dm::BGKLOctoMap>& map) const;
};

}  // namespace aeplanner

#endif  // AEPLANNER_FRONTIER_SAMPLER_H
