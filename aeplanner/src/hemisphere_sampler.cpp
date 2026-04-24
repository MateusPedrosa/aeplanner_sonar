#include <aeplanner/hemisphere_sampler.h>
#include <cmath>
#include <cstdlib>
#include <algorithm>

namespace aeplanner
{

Eigen::Vector3d HemisphereSampler::sphericalToCart(
    double theta, double phi, const Eigen::Vector3d& n_hat) const
{
  // Build an orthonormal basis {u, v, n_hat}
  Eigen::Vector3d u = n_hat.unitOrthogonal();
  Eigen::Vector3d v = n_hat.cross(u);
  double st = std::sin(theta);
  return st * std::cos(phi) * u + st * std::sin(phi) * v + std::cos(theta) * n_hat;
}

bool HemisphereSampler::collisionFree(
    const Eigen::Vector3d& p,
    const std::shared_ptr<la3dm::BGKLOctoMap>& map) const
{
  la3dm::OcTreeNode node = map->search((float)p.x(), (float)p.y(), (float)p.z());
  return node.get_state() != la3dm::State::OCCUPIED;
}

std::vector<CandidatePose> HemisphereSampler::sampleCandidates(
    const ScoredTarget&             target,
    const Eigen::Vector4d&          robot_state,
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Params&                   params)
{
  const int N = params.N_samples;
  const double d_standoff = params.d_standoff;
  const double R_sample   = params.R_sample;
  const double lambda_d   = params.lambda_dist;

  // Normal: use target.normal; fall back to LOS from robot if near-zero
  Eigen::Vector3d normal = target.normal;
  if (normal.squaredNorm() < 1e-8)
  {
    normal = (target.pos - robot_state.head<3>()).normalized();
    if (normal.squaredNorm() < 1e-8) normal = Eigen::Vector3d::UnitX();
  }

  // Centre of hemisphere at standoff distance from target surface
  Eigen::Vector3d p_center = target.pos + d_standoff * normal;

  std::vector<CandidatePose> candidates;
  candidates.reserve(N);

  for (int i = 0; i < N; ++i)
  {
    // Cosine-weighted sampling: theta = acos(uniform(0,1)), phi = uniform(0,2π)
    double theta = std::acos(((double)rand() / (double)RAND_MAX));
    double phi   = 2.0 * M_PI * ((double)rand() / (double)RAND_MAX);

    Eigen::Vector3d p_cand = p_center + R_sample * sphericalToCart(theta, phi, normal);

    if (!collisionFree(p_cand, map)) continue;

    double yaw = std::atan2(target.pos.y() - p_cand.y(),
                            target.pos.x() - p_cand.x());

    Eigen::Vector4d state(p_cand.x(), p_cand.y(), p_cand.z(), yaw);
    double dist = (p_cand - robot_state.head<3>()).norm();
    // Score uses only distance penalty here; gain scoring happens in gainCubature
    // called from expandRRT after try_candidate accepts the pose.
    double score = target.priority * std::exp(-lambda_d * dist);

    candidates.push_back({state, score});
  }

  // Fallback: if fewer than 5 collision-free candidates, add a nearest projection
  if (candidates.size() < 5)
  {
    Eigen::Vector3d p_proj = p_center;
    if (collisionFree(p_proj, map))
    {
      double yaw = std::atan2(target.pos.y() - p_proj.y(),
                              target.pos.x() - p_proj.x());
      double dist = (p_proj - robot_state.head<3>()).norm();
      candidates.push_back({{p_proj.x(), p_proj.y(), p_proj.z(), yaw},
                             target.priority * std::exp(-lambda_d * dist)});
    }
  }

  // Sort descending by score
  std::sort(candidates.begin(), candidates.end(),
    [](const CandidatePose& a, const CandidatePose& b) {
      return a.score > b.score;
    });

  return candidates;
}

}  // namespace aeplanner
