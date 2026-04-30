#include <aeplanner/planner_state_machine.h>
#include <cmath>
#include <algorithm>

namespace aeplanner
{

const char* plannerStateToString(PlannerState s)
{
  switch (s)
  {
    case PlannerState::EXPLORE: return "EXPLORE";
    case PlannerState::RESOLVE: return "RESOLVE";
    case PlannerState::DWELL:   return "DWELL";
    case PlannerState::DONE:    return "DONE";
    default:                    return "UNKNOWN";
  }
}

PlannerState PlannerStateMachine::tick(
    const std::vector<ScoredTarget>& targets,
    const Eigen::Vector4d&           robot_state,
    const Params&                    params)
{
  if (in_dwell_)              return PlannerState::DWELL;
  if (in_resolve_commitment_) return PlannerState::RESOLVE;

  if (targets.empty()) return PlannerState::DONE;

  // Consider ALL targets globally — EXPLORE/RESOLVE handle any distance
  // via planPathToGoal. No local_radius filtering; no REPOSITION state.
  float best_u = -1.0f;
  float best_e = -1.0f;

  for (const ScoredTarget& t : targets)
  {
    if (t.type == TargetType::U_TARGET)
      best_u = std::max(best_u, t.priority);
    else
      best_e = std::max(best_e, t.priority);
  }

  if (best_u > 0.0f && best_u > best_e * params.alpha_bias)
    return PlannerState::RESOLVE;

  if (best_e > 0.0f)
    return PlannerState::EXPLORE;

  // Only U_TARGETs with lower priority than E-targets, or only U_TARGETs
  if (best_u > 0.0f)
    return PlannerState::RESOLVE;

  return PlannerState::DONE;
}

void PlannerStateMachine::enterDwell(const ScoredTarget& target)
{
  dwell_target_ = target;
  in_dwell_     = true;
}

void PlannerStateMachine::exitDwell()
{
  in_dwell_ = false;
}

}  // namespace aeplanner
