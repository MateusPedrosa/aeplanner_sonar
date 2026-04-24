#ifndef AEPLANNER_FRONTIER_DETECTOR_H
#define AEPLANNER_FRONTIER_DETECTOR_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <aeplanner/voxel_classifier.h>

namespace aeplanner
{

struct FrontierCluster
{
  Eigen::Vector3d centroid;
  Eigen::Vector3d normal;   // principal axis (pointing away from occupied side)
  float           w_normal; // PCA anisotropy = (λ1 - λ2) / λ1; < 0.4 → unreliable
  float           density;  // number of voxels in cluster / normalisation constant
  TargetType      type;     // E_OCC if any member is adj-to-occupied, else E_FREE
  std::vector<size_t> member_indices; // indices into the input voxel vector
};

// Clusters frontier voxels (E_OCC or E_FREE) within R_cluster and runs PCA
// to estimate surface normals. Returns one FrontierCluster per group.
// Input voxels must all have type E_OCC or E_FREE.
std::vector<FrontierCluster> detectFrontierClusters(
    const std::vector<ClassifiedVoxel>& frontier_voxels,
    float R_cluster);

}  // namespace aeplanner

#endif  // AEPLANNER_FRONTIER_DETECTOR_H
