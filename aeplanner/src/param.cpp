#include <ros/ros.h>
#include <aeplanner/param.h>
#include <cmath>

namespace aeplanner
{
  Params readParams()
  {
    // FIXME namespaces
    Params params;
    std::string ns = ros::this_node::getNamespace();
    params.hfov = 60;
    if (!ros::param::get(ns + "/camera/horizontal_fov", params.hfov)) {
      ROS_WARN_STREAM("No horizontal fov specified. Default: " << params.hfov);
    }
    params.vfov = 45;
    if (!ros::param::get(ns + "/camera/vertical_fov", params.vfov)) {
      ROS_WARN_STREAM("No vertical fov specified. Default: " << params.vfov);
    }
    // params.dr = 0.1;
    // if (!ros::param::get("/octomap_server/resolution", params.dr)) {
    //   ROS_WARN_STREAM("Could not read octomap resolution. Looking for /octomap_server/resolution.");
    //   ROS_WARN_STREAM("Using resolution specified by param file instead");
    // }
    // else if (!ros::param::get(ns + "/raycast/dr", params.dr)) {
    //   ROS_WARN_STREAM("No dr specified. Default: " << params.dr);
    // }
    params.dphi = 10;
    if (!ros::param::get(ns + "/raycast/dphi", params.dphi)) {
      ROS_WARN_STREAM("No dphi specified. Default: " << params.dphi);
    }
    params.dtheta = 10;
    if (!ros::param::get(ns + "/raycast/dtheta", params.dtheta)) {
      ROS_WARN_STREAM("No dtheta specified. Default: " << params.dtheta);
    }
    params.lambda = 0.5;
    if (!ros::param::get(ns + "/aep/gain/lambda", params.lambda)) {
      ROS_WARN_STREAM("No lambda specified. Default: " << params.lambda);
    }
    params.extension_range = 1.0;
    if (!ros::param::get(ns + "/aep/tree/extension_range", params.extension_range)) {
      ROS_WARN_STREAM("No extension_range specified. Default: " << params.extension_range);
    }
    params.max_sampling_radius = 10.0;
    if (!ros::param::get(ns + "/aep/tree/max_sampling_radius", params.max_sampling_radius)) {
      ROS_WARN_STREAM("No max_sampling_radius specified. Default: " << params.max_sampling_radius);
    }
    params.sigma_thresh = 0.2;
    if (!ros::param::get(ns + "/aep/gain/sigma_thresh", params.sigma_thresh)) {
      ROS_WARN_STREAM("No sigma_thresh specified. Default: " << params.sigma_thresh);
    }
    params.init_iterations = 15;
    if (!ros::param::get(ns + "/aep/tree/initial_iterations", params.init_iterations)) {
      ROS_WARN_STREAM("No init_iterations specified. Default: " << params.init_iterations);
    }
    params.r_max = 4.0;
    if (!ros::param::get(ns + "/aep/gain/r_max", params.r_max)) {
      ROS_WARN_STREAM("No r_max specified. Default: " << params.r_max);
    }
    params.r_min = 0.5;
    if (!ros::param::get(ns + "/aep/gain/r_min", params.r_min)) {
      ROS_WARN_STREAM("No r_min specified. Default: " << params.r_min);
    }
    if (!ros::param::get(ns + "/boundary/min", params.boundary_min)) {
      ROS_WARN_STREAM("No boundary/min specified.");
    }
    if (!ros::param::get(ns + "/boundary/max", params.boundary_max)) {
      ROS_WARN_STREAM("No boundary/max specified.");
    }
    params.bounding_radius = 0.5;
    if (!ros::param::get(ns + "/system/bbx/r", params.bounding_radius)) {
      ROS_WARN_STREAM("No /system/bbx/r specified. Default: " << params.bounding_radius);
    }
    params.cutoff_iterations = 200;
    if (!ros::param::get(ns + "/aep/tree/cutoff_iterations", params.cutoff_iterations)) {
      ROS_WARN_STREAM("No /aep/tree/cutoff_iterations specified. Default: " << params.cutoff_iterations);
    }
    params.max_sampling_attempts = 1000;
    if (!ros::param::get(ns + "/aep/tree/max_sampling_attempts", params.max_sampling_attempts)) {
      ROS_WARN_STREAM("No /aep/tree/max_sampling_attempts specified. Default: " << params.max_sampling_attempts);
    }
    params.viz_rate = 0.2;
    if (!ros::param::get(ns + "/viz_rate", params.viz_rate)) {
      ROS_WARN_STREAM("No /viz_rate specified. Default: " << params.viz_rate);
    }
    params.max_vis_radius = 20.0;
    if (!ros::param::get(ns + "/max_vis_radius", params.max_vis_radius)) {
      ROS_WARN_STREAM("No /max_vis_radius specified. Default: " << params.max_vis_radius);
    }
    params.zero_gain = 0.0;
    if (!ros::param::get(ns + "/aep/gain/zero", params.zero_gain)) {
      ROS_WARN_STREAM("No /aep/gain/zero specified. Default: " << params.zero_gain);
    }
    params.d_overshoot_ = 0.5;
    if (!ros::param::get(ns + "/system/bbx/overshoot", params.d_overshoot_)) {
      ROS_WARN_STREAM("No /system/bbx/overshoot specified. Default: " << params.d_overshoot_);
    }
    params.world_frame = "map";
    if (!ros::param::get(ns + "/world_frame", params.world_frame)) {
      ROS_WARN_STREAM("No /world_frame specified. Default: " << params.world_frame);
    }
    params.robot_frame = "base_link";
    if (!ros::param::get(ns + "/robot_frame", params.robot_frame)) {
      ROS_WARN_STREAM("No /robot_frame specified. Default: " << params.robot_frame);
    }
    params.sensor_frame = "sonar_link";
    if (!ros::param::get(ns + "/sensor_frame", params.sensor_frame)) {
      ROS_WARN_STREAM("No /sensor_frame specified. Default: " << params.sensor_frame);
    }
    params.visualize_tree = false;
    if (!ros::param::get(ns + "/visualize_tree", params.visualize_tree)) {
      ROS_WARN_STREAM("No /visualize_tree specified. Default: " << params.visualize_tree);
    }
    params.visualize_rays = false;
    if (!ros::param::get(ns + "/visualize_rays", params.visualize_rays)) {
      ROS_WARN_STREAM("No /visualize_rays specified. Default: " << params.visualize_rays);
    }
    params.visualize_exploration_area = false;
    if (!ros::param::get(ns + "/visualize_exploration_area", params.visualize_exploration_area)) {
      ROS_WARN_STREAM("No /visualize_exploration_area specified. Default: " << params.visualize_exploration_area);
    }

    // --- BGKLOctoMap Parameters ---
    params.resolution = 0.05;
    if (!ros::param::get("resolution", params.resolution)) {
      ROS_WARN_STREAM("No map resolution specified. Default: " << params.resolution);
    }
    params.block_depth = 5;
    if (!ros::param::get("block_depth", params.block_depth)) {
      ROS_WARN_STREAM("No block_depth specified. Default: " << params.block_depth);
    }
    params.sf2 = 1.0;
    if (!ros::param::get("sf2", params.sf2)) {
      ROS_WARN_STREAM("No sf2 specified. Default: " << params.sf2);
    }
    params.ell = 0.6;
    if (!ros::param::get("ell", params.ell)) {
      ROS_WARN_STREAM("No ell specified. Default: " << params.ell);
    }
    params.free_thresh = 0.3;
    if (!ros::param::get("free_thresh", params.free_thresh)) {
      ROS_WARN_STREAM("No free_thresh specified. Default: " << params.free_thresh);
    }
    params.occupied_thresh = 0.7;
    if (!ros::param::get("occupied_thresh", params.occupied_thresh)) {
      ROS_WARN_STREAM("No occupied_thresh specified. Default: " << params.occupied_thresh);
    }
    params.var_thresh = 100.0f;
    if (!ros::param::get("var_thresh", params.var_thresh)) {
      ROS_WARN_STREAM("No var_thresh specified. Default: " << params.var_thresh);
    }
    params.prior_A = 0.001f;
    if (!ros::param::get("prior_A", params.prior_A)) {
      ROS_WARN_STREAM("No prior_A specified. Default: " << params.prior_A);
    }
    params.prior_B = 0.001f;
    if (!ros::param::get("prior_B", params.prior_B)) {
      ROS_WARN_STREAM("No prior_B specified. Default: " << params.prior_B);
    }
    params.theta_bw = 0.6f * M_PI / 180.0f;
    if (!ros::param::get("theta_bw", params.theta_bw)) {
      ROS_WARN_STREAM("No theta_bw specified. Default: " << params.theta_bw);
    }
    params.phi_bw = 20.0f * M_PI / 180.0f;
    if (!ros::param::get("phi_bw", params.phi_bw)) {
      ROS_WARN_STREAM("No phi_bw specified. Default: " << params.phi_bw);
    }
    params.free_ray_range_weight = false;
    if (!ros::param::get("free_ray_range_weight", params.free_ray_range_weight)) {
      ROS_WARN_STREAM("No free_ray_range_weight specified. Default: " << params.free_ray_range_weight);
    }
    params.tau_var = 0.01f;
    if (!ros::param::get("tau_var", params.tau_var)) {
      ROS_WARN_STREAM("No tau_var specified. Default: " << params.tau_var);
    }
    params.tau_info = 0.5f;
    if (!ros::param::get("tau_info", params.tau_info)) {
      ROS_WARN_STREAM("No tau_info specified. Default: " << params.tau_info);
    }
    params.uncertain_threshold = 4.0f;
    if (!ros::param::get("uncertain_threshold", params.uncertain_threshold)) {
      ROS_WARN_STREAM("No uncertain_threshold specified. Default: " << params.uncertain_threshold);
    }
    params.ds_resolution = -1.0f;
    if (!ros::param::get("ds_resolution", params.ds_resolution)) {
      ROS_WARN_STREAM("No ds_resolution specified. Default: " << params.ds_resolution);
    }
    params.free_resolution = 0.4f;
    if (!ros::param::get("free_resolution", params.free_resolution)) {
      ROS_WARN_STREAM("No free_resolution specified. Default: " << params.free_resolution);
    }
    params.min_z = 0.0;
    if (!ros::param::get("min_z", params.min_z)) {
      ROS_WARN_STREAM("No min_z specified. Default: " << params.min_z);
    }
    params.max_z = 0.0;
    if (!ros::param::get("max_z", params.max_z)) {
      ROS_WARN_STREAM("No max_z specified. Default: " << params.max_z);
    }
    params.nbv_k = 1000;
    if (!ros::param::get(ns + "/aep/gain/nbv_k", params.nbv_k)) {
      ROS_WARN_STREAM("No nbv_k specified. Default: " << params.nbv_k);
    }

    // --- Directed View Planner Parameters ---
    params.sigma2_thresh = 0.05f;
    if (!ros::param::get(ns + "/viewplanner/sigma2_thresh", params.sigma2_thresh)) {
      ROS_WARN_STREAM("No viewplanner/sigma2_thresh specified. Default: " << params.sigma2_thresh);
    }
    params.w_frontier = 0.6f;
    if (!ros::param::get(ns + "/viewplanner/w_frontier", params.w_frontier)) {
      ROS_WARN_STREAM("No viewplanner/w_frontier specified. Default: " << params.w_frontier);
    }
    params.cluster_norm = 100.0f;
    if (!ros::param::get(ns + "/viewplanner/cluster_norm", params.cluster_norm)) {
      ROS_WARN_STREAM("No viewplanner/cluster_norm specified. Default: " << params.cluster_norm);
    }
    params.R_cluster = 2.0f;
    if (!ros::param::get(ns + "/viewplanner/R_cluster", params.R_cluster)) {
      ROS_WARN_STREAM("No viewplanner/R_cluster specified. Default: " << params.R_cluster);
    }
    params.var_resolved_thresh = 0.01f;
    if (!ros::param::get(ns + "/viewplanner/var_resolved_thresh", params.var_resolved_thresh)) {
      ROS_WARN_STREAM("No viewplanner/var_resolved_thresh specified. Default: " << params.var_resolved_thresh);
    }
    params.alpha_bias = 1.0f;
    if (!ros::param::get(ns + "/viewplanner/alpha_bias", params.alpha_bias)) {
      ROS_WARN_STREAM("No viewplanner/alpha_bias specified. Default: " << params.alpha_bias);
    }
    params.lambda_dist = 0.4;
    if (!ros::param::get(ns + "/viewplanner/lambda_dist", params.lambda_dist)) {
      ROS_WARN_STREAM("No viewplanner/lambda_dist specified. Default: " << params.lambda_dist);
    }
    params.d_standoff = 4.0;
    if (!ros::param::get(ns + "/viewplanner/d_standoff", params.d_standoff)) {
      ROS_WARN_STREAM("No viewplanner/d_standoff specified. Default: " << params.d_standoff);
    }
    params.R_sample = 8.0;
    if (!ros::param::get(ns + "/viewplanner/R_sample", params.R_sample)) {
      ROS_WARN_STREAM("No viewplanner/R_sample specified. Default: " << params.R_sample);
    }
    params.N_samples = 50;
    if (!ros::param::get(ns + "/viewplanner/N_samples", params.N_samples)) {
      ROS_WARN_STREAM("No viewplanner/N_samples specified. Default: " << params.N_samples);
    }
    params.T_dwell = 15.0;
    if (!ros::param::get(ns + "/viewplanner/T_dwell", params.T_dwell)) {
      ROS_WARN_STREAM("No viewplanner/T_dwell specified. Default: " << params.T_dwell);
    }
    params.N_fail = 3;
    if (!ros::param::get(ns + "/viewplanner/N_fail", params.N_fail)) {
      ROS_WARN_STREAM("No viewplanner/N_fail specified. Default: " << params.N_fail);
    }
    params.T_cooldown = 30.0;
    if (!ros::param::get(ns + "/viewplanner/T_cooldown", params.T_cooldown)) {
      ROS_WARN_STREAM("No viewplanner/T_cooldown specified. Default: " << params.T_cooldown);
    }
    params.local_radius = 10.0;
    if (!ros::param::get(ns + "/viewplanner/local_radius", params.local_radius)) {
      ROS_WARN_STREAM("No viewplanner/local_radius specified. Default: " << params.local_radius);
    }
    params.w_normal_thresh = 0.4f;
    if (!ros::param::get(ns + "/viewplanner/w_normal_thresh", params.w_normal_thresh)) {
      ROS_WARN_STREAM("No viewplanner/w_normal_thresh specified. Default: " << params.w_normal_thresh);
    }
    params.tpm_rate = 0.5;
    if (!ros::param::get(ns + "/viewplanner/tpm_rate", params.tpm_rate)) {
      ROS_WARN_STREAM("No viewplanner/tpm_rate specified. Default: " << params.tpm_rate);
    }
    params.dwell_arrival_thresh = 1.0;
    if (!ros::param::get(ns + "/viewplanner/dwell_arrival_thresh", params.dwell_arrival_thresh)) {
      ROS_WARN_STREAM("No viewplanner/dwell_arrival_thresh specified. Default: " << params.dwell_arrival_thresh);
    }

    return params;
  }
}