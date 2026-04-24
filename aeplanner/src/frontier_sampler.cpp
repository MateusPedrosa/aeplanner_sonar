#include <aeplanner/frontier_sampler.h>
#include <cmath>
#include <cstdlib>
#include <algorithm>

namespace aeplanner
{

bool FrontierSampler::collisionFree(
    const Eigen::Vector3d& p,
    const std::shared_ptr<la3dm::BGKLOctoMap>& map) const
{
  la3dm::OcTreeNode node = map->search((float)p.x(), (float)p.y(), (float)p.z());
  return node.get_state() != la3dm::State::OCCUPIED;
}

std::vector<CandidatePose> FrontierSampler::sampleCandidates(
    const ScoredTarget&             target,
    const Eigen::Vector4d&          robot_state,
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Params&                   params)
{
  const int    N          = params.N_samples;
  const double d_standoff = params.d_standoff;
  const double lambda_d   = params.lambda_dist;
  const double dz_step    = 1.0;  // depth step for fallback (m)

  std::vector<CandidatePose> candidates;
  candidates.reserve(N);

  bool use_normal = (target.w_normal >= params.w_normal_thresh);

  if (use_normal)
  {
    // Sample strip along frontier tangent
    Eigen::Vector3d normal = target.normal;
    if (normal.squaredNorm() < 1e-8) normal = Eigen::Vector3d::UnitX();

    Eigen::Vector3d p_center = target.pos + d_standoff * normal;

    // Tangent direction: perpendicular to normal in the horizontal plane
    Eigen::Vector3d tangent(-normal.y(), normal.x(), 0.0);
    if (tangent.squaredNorm() < 1e-8) tangent = Eigen::Vector3d::UnitX();
    else tangent.normalize();

    double strip_half = d_standoff;  // strip ±d_standoff along tangent
    for (int i = 0; i < N; ++i)
    {
      double t = strip_half * (2.0 * ((double)rand() / (double)RAND_MAX) - 1.0);
      Eigen::Vector3d p_cand = p_center + t * tangent;

      if (!collisionFree(p_cand, map)) continue;

      double yaw = std::atan2(target.pos.y() - p_cand.y(),
                              target.pos.x() - p_cand.x());
      double dist = (p_cand - robot_state.head<3>()).norm();
      candidates.push_back(
          {{p_cand.x(), p_cand.y(), p_cand.z(), yaw},
           target.priority * std::exp(-lambda_d * dist)});
    }
  }
  else
  {
    // Depth-biased fallback: sample at current_depth - dz_step in local XY box
    double z_target = robot_state.z() - dz_step;
    double half_box = params.local_radius * 0.5;

    for (int i = 0; i < N; ++i)
    {
      double dx = half_box * (2.0 * ((double)rand() / (double)RAND_MAX) - 1.0);
      double dy = half_box * (2.0 * ((double)rand() / (double)RAND_MAX) - 1.0);
      Eigen::Vector3d p_cand(robot_state.x() + dx, robot_state.y() + dy, z_target);

      if (!collisionFree(p_cand, map)) continue;

      double yaw = 2.0 * M_PI * ((double)rand() / (double)RAND_MAX) - M_PI;
      double dist = (p_cand - robot_state.head<3>()).norm();
      candidates.push_back(
          {{p_cand.x(), p_cand.y(), p_cand.z(), yaw},
           target.priority * std::exp(-lambda_d * dist)});
    }
  }

  std::sort(candidates.begin(), candidates.end(),
    [](const CandidatePose& a, const CandidatePose& b) {
      return a.score > b.score;
    });

  return candidates;
}

}  // namespace aeplanner
