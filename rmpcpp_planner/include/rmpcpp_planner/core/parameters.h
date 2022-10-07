#ifndef RMPCPP_PLANNER_PARAMETERS_H
#define RMPCPP_PLANNER_PARAMETERS_H

#include "Eigen/Dense"

/**
 * Most of the default values here all get overridden by the parser class.
 */

enum PolicyType { SIMPLE_ESDF, RAYCASTING_CUDA };

struct WorldPolicyParameters {
  WorldPolicyParameters() = default;
  virtual ~WorldPolicyParameters() = default;
};

/** TODO: Fix the redundancy between some of these */
struct EsdfPolicyParameters : WorldPolicyParameters {
  /** Simple ESDF policy parameters. */
  double eta_rep = 22.0;   // Gets multiplied by a gain factor from the parser
  double eta_damp = 35.0;  // Gets multiplied by a gain factor from the parser
  double v_rep = 2.0;
  double v_damp = 2.0;
  double epsilon_damp = 0.1;
  double c_softmax_obstacle = 0.2;
  double r = 5.0;
};

struct RaycastingCudaPolicyParameters : WorldPolicyParameters {
  float eta_rep = 22.0;   // Gets multiplied by a gain factor from the parser
  float eta_damp = 35.0;  // Gets multiplied by a gain factor from the parser
  float v_rep = 2.0;
  float v_damp = 2.0;
  float epsilon_damp = 0.1;
  float c_softmax_obstacle = 0.2;
  float r = 5.0;
  bool metric = true;

  float lin_rep = 1.0;

  float alpha_goal = 10;
  float beta_goal = 20;
  float gamma_goal = 0.02;
  float metric_goal = 1.0;

  float a_fsp = 1e-4;
  float eta_fsp = 0;

  int N_sqrt = 32;  // square root of number of rays. Has to be divisible by
                    // blocksize TODO: deal with non divisibility
  float surface_distance_epsilon_vox = 0.1f;
  int max_steps = 100;
  double truncation_distance_vox = 1.0f;
};

struct ParametersRMP {
  ParametersRMP() : ParametersRMP(RAYCASTING_CUDA){};
  explicit ParametersRMP(PolicyType T) {
    policy_type = T;
    switch (T) {
      case SIMPLE_ESDF:
        worldPolicyParameters = new EsdfPolicyParameters();
        ((EsdfPolicyParameters*)worldPolicyParameters)->r = r;
        break;
      case RAYCASTING_CUDA:
        worldPolicyParameters = new RaycastingCudaPolicyParameters();
        ((RaycastingCudaPolicyParameters*)worldPolicyParameters)
            ->truncation_distance_vox = truncation_distance_vox;
        ((RaycastingCudaPolicyParameters*)worldPolicyParameters)->r = r;
        break;
    }
  };
  ~ParametersRMP() = default;

  PolicyType policy_type;
  double dt = 0.04;
  double r = 5.0;
  int max_length = 2000;
  double truncation_distance_vox = 1.0f;

  bool terminate_upon_goal_reached = true;

  /** Target policy parameters */
  double alpha_target = 10.0;
  double beta_target = 15.0;
  double c_softmax_target = 0.2;

  WorldPolicyParameters* worldPolicyParameters = nullptr;
};

#endif  // RMPCPP_PLANNER_PARAMETERS_H
