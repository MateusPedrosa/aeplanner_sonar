#ifndef READ_PARAMS_H
#define READ_PARAMS_H

#include <vector>
#include <string>

namespace aeplanner
{
  struct Params
  {
    double hfov;
    double vfov;
    double r_max;
    double r_min;

    double dr;
    double dphi;
    double dtheta;

    double lambda;
    double zero_gain;
    double extension_range;
    double max_sampling_radius;
    double sigma_thresh;

    double d_overshoot_;
    double bounding_radius;

    int init_iterations;
    int cutoff_iterations;

    std::vector<double> boundary_min;
    std::vector<double> boundary_max;

    std::string robot_frame;
    std::string world_frame;
    std::string sensor_frame;

    bool visualize_tree;
    bool visualize_rays;
    bool visualize_exploration_area;

    // BGKLOctoMap parameters
    double resolution;
    int block_depth;
    double sf2;
    double ell;
    double free_thresh;
    double occupied_thresh;
    float var_thresh;
    float prior_A;
    float prior_B;
    float theta_bw;
    float phi_bw;
    bool  free_ray_range_weight;
    float tau_var;
    float tau_info;
    float uncertain_threshold;
    float free_resolution;
    float ds_resolution;
    double min_z;
    double max_z;
  };

  Params readParams();
}

#endif