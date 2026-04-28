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
    int max_sampling_attempts;

    double viz_rate;
    double max_vis_radius;

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

    int nbv_k;  // top-K voxels used in info-matrix gain scoring (NBV planner)

    // --- Directed View Planner Parameters ---
    float  sigma2_thresh;       // min Var_beta for U-target (default 0.05)
    float  w_frontier;          // frontier bonus weight in priority (default 0.6)
    float  cluster_norm;        // density normalisation for frontier priority (default 100.0)
    float  R_cluster;           // frontier clustering radius in metres (default 2.0)
    float  var_resolved_thresh; // Var_beta below which DWELL exits (default 0.01)
    float  alpha_bias;          // U vs E priority bias in state machine (default 1.0)
    double lambda_dist;         // exponential distance penalty in pose scorer (default 0.2)
    double d_standoff;          // nominal sonar standoff distance in metres (default 4.0)
    double R_sample;            // hemisphere sampling radius (default 8.0)
    int    N_samples;           // hemisphere samples per target (default 50)
    double T_dwell;             // DWELL timeout seconds (default 15.0)
    int    N_fail;              // consecutive failures before blacklisting (default 3)
    double T_cooldown;          // blacklist cooldown seconds (default 30.0)
    double local_radius;        // radius defining "local volume" (default 10.0)
    float  w_normal_thresh;     // min PCA anisotropy for reliable frontier normal (default 0.4)
    double tpm_rate;            // TPM update rate in Hz (default 2.0)
    double dwell_arrival_thresh; // dist to committed viewpoint that triggers DWELL entry (default 1.0)

    // --- Exploration gain parameters ---
    int    n_explore_rays_az;     // azimuth ray count for gainExploration (default 15)
    int    n_explore_rays_el;     // elevation ray count for gainExploration (default 4)
    double w_unknown;             // weight for UNKNOWN voxel count in explore score (default 1.0)
    double w_cubature_explore;    // weight for gainCubature component in explore score (default 0.1)
    double min_explore_gain;      // min explore score to commit; fallback to RRT if lower (default 5.0)
  };

  Params readParams();
}

#endif