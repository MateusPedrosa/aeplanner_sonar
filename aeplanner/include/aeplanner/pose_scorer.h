#ifndef AEPLANNER_POSE_SCORER_H
#define AEPLANNER_POSE_SCORER_H

// pose_scorer.h — stub header.
// Full scoring is handled by AEPlanner::gainCubature() which is called from
// expandRRT(). The PoseScorer concept is realised inline in expandRRT() using
// getGain() + exponential distance penalty (lambda_dist).
// This header is provided for forward-compatibility if scoring is later
// extracted into a stand-alone class.

#endif  // AEPLANNER_POSE_SCORER_H
