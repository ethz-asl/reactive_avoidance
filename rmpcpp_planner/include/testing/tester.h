
#ifndef RMPCPP_PLANNER_TESTER_H
#define RMPCPP_PLANNER_TESTER_H

#include <string>

#include "rmpcpp_planner/core/parameters.h"
#include "rmpcpp_planner/core/planner_rmp.h"
#include "testing/settings.h"
#include "testing/worldgen.h"

struct ParametersWrapper {
  ~ParametersWrapper() = default;

  explicit ParametersWrapper(const ParametersRMP &params)
      : parametersRMP(params){};

  const ParametersRMP parametersRMP;
};

/**
 * The tester class makes it easy to generate worlds, run the planner and export
 * the results to files that can be evaluated in python for e.g. plotting.
 */
class Tester {
  static const int dim = 3;
  using Space = rmpcpp::Space<dim>;

 public:
  Tester(const ParametersWrapper &parameters,
         const rmpcpp::TestSettings &settings);

  void run();

 private:
  rmpcpp::WorldGen worldgen;
  struct SuccessStats {
    int successfulcount = 0;
    std::vector<double> timings;
    std::vector<int> total_integration_steps;
    std::vector<double> integrations_per_sec;
    std::vector<double> shortest_lengths;
    std::vector<int> shortest_discrete_lengths;
    std::vector<int> success;
    std::vector<int> indices;
    std::vector<double> smoothness;
  } stats;

  /** World densities */
  std::vector<double> densities;

  void run_single();
  void update_success_stats();
  ParametersWrapper parameters;
  rmpcpp::TestSettings settings;
  std::unique_ptr<rmpcpp::PlannerBase<Space>> planner;

  void exportTrajectories(std::string path, const int i);
  void exportWorld(std::string path, const int i);
  void exportStats(std::string path);

  int run_index = 0;
};

#endif  // RMPCPP_PLANNER_TESTER_H
