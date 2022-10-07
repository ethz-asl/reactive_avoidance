
#ifndef RMPCPP_PLANNER_TESTER_H
#define RMPCPP_PLANNER_TESTER_H

#include <string>

#include "rmpcpp_planner/core/parameters.h"
#include "rmpcpp_planner/core/planner_rmp.h"
#include "testing/settings.h"
#include "testing/statistics.h"
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
  void runSingle(size_t run_index);
  void updateStats(int index, double map_density, double duration_s);
  void exportTrajectories(std::string path, const int i);
  void exportWorld(std::string path, const int i);
  void exportStats(std::string path);
  std::string getMapName();

  rmpcpp::WorldGen worldgen_;
  rmpcpp::RunStatistics statistics_;
  ParametersWrapper parameters_;
  rmpcpp::TestSettings settings_;
  std::unique_ptr<rmpcpp::PlannerBase<Space>> planner_;
};

#endif  // RMPCPP_PLANNER_TESTER_H
