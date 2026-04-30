#ifndef AEPLANNER_PLANNER_STATE_MACHINE_H
#define AEPLANNER_PLANNER_STATE_MACHINE_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <aeplanner/priority_scorer.h>
#include <aeplanner/param.h>

namespace aeplanner
{

enum class PlannerState
{
  EXPLORE,
  RESOLVE,
  DWELL,
  DONE
};

const char* plannerStateToString(PlannerState s);

class PlannerStateMachine
{
public:
  PlannerStateMachine() = default;

  // Determine the next planner state given the current target list and robot pose.
  // alpha_bias: U_target.priority must exceed E_target.priority * alpha_bias
  // to trigger RESOLVE instead of EXPLORE.
  PlannerState tick(const std::vector<ScoredTarget>& targets,
                    const Eigen::Vector4d&           robot_state,
                    const Params&                    params);

  // Force DWELL state (called by the planner on arrival at a RESOLVE viewpoint).
  void enterDwell(const ScoredTarget& target);

  // Exit DWELL (called by DwellController when resolved, timed-out, or FoV lost).
  void exitDwell();

  bool inDwell() const { return in_dwell_; }

  const ScoredTarget& dwellTarget() const { return dwell_target_; }

  // Protect an in-progress RESOLVE navigation from being preempted by EXPLORE.
  // Called when a viewpoint is committed; cleared on arrival, path failure, or
  // stale-commitment invalidation.
  void enterResolveCommitment() { in_resolve_commitment_ = true;  }
  void exitResolveCommitment()  { in_resolve_commitment_ = false; }

  bool inResolveCommitment() const { return in_resolve_commitment_; }

private:
  bool         in_dwell_              = false;
  bool         in_resolve_commitment_ = false;
  ScoredTarget dwell_target_;
};

}  // namespace aeplanner

#endif  // AEPLANNER_PLANNER_STATE_MACHINE_H
