#ifndef AEPLANNER_DIRECTED_SAMPLER_H
#define AEPLANNER_DIRECTED_SAMPLER_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <bgkloctomap/bgkloctomap.h>
#include <aeplanner/priority_scorer.h>
#include <aeplanner/param.h>

namespace aeplanner
{

struct CandidatePose
{
  Eigen::Vector4d state;  // (x, y, z, yaw)
  double          score;
};

class DirectedSampler
{
public:
  virtual ~DirectedSampler() = default;

  // Generate scored candidate poses aimed at the given target.
  // The caller picks the best via score and feeds to try_candidate().
  virtual std::vector<CandidatePose> sampleCandidates(
      const ScoredTarget&             target,
      const Eigen::Vector4d&          robot_state,
      const std::shared_ptr<la3dm::BGKLOctoMap>& map,
      const Params&                   params) = 0;
};

}  // namespace aeplanner

#endif  // AEPLANNER_DIRECTED_SAMPLER_H
