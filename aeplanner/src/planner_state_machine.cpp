#include <aeplanner/planner_state_machine.h>
#include <cmath>
#include <algorithm>

namespace aeplanner
{

const char* plannerStateToString(PlannerState s)
{
  switch (s)
  {
    case PlannerState::EXPLORE:    return "EXPLORE";
    case PlannerState::RESOLVE:    return "RESOLVE";
    case PlannerState::DWELL:      return "DWELL";
    case PlannerState::REPOSITION: return "REPOSITION";
    case PlannerState::DONE:       return "DONE";
    default:                       return "UNKNOWN";
  }
}

PlannerState PlannerStateMachine::tick(
    const std::vector<ScoredTarget>& targets,
    const Eigen::Vector4d&           robot_state,
    const Params&                    params)
{
  if (in_dwell_) return PlannerState::DWELL;

  if (targets.empty()) return PlannerState::DONE;

  const double local_r2 = params.local_radius * params.local_radius;
  Eigen::Vector3d robot_pos = robot_state.head<3>();

  // Find best U-target and best E-target in local volume
  float best_u_priority = -1.0f;
  float best_e_priority = -1.0f;

  for (const ScoredTarget& t : targets)
  {
    bool local = (t.pos - robot_pos).squaredNorm() <= local_r2;
    if (!local) continue;
    if (t.type == TargetType::U_TARGET)
      best_u_priority = std::max(best_u_priority, t.priority);
    else
      best_e_priority = std::max(best_e_priority, t.priority);
  }

  if (best_u_priority > 0.0f &&
      best_u_priority > best_e_priority * params.alpha_bias)
    return PlannerState::RESOLVE;

  if (best_e_priority > 0.0f)
    return PlannerState::EXPLORE;

  // No local targets — check if any global targets exist
  if (!targets.empty())
    return PlannerState::REPOSITION;

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
