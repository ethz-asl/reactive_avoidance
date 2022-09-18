//
// Created by isar on 09/04/2022.
//

#ifndef RMPCPP_PLANNER_DATA_EXPORTER_H
#define RMPCPP_PLANNER_DATA_EXPORTER_H

#include "rmpcpp_planner/core/planner_rmp.h"

template <class Space>
class DataExporter {
 public:
  virtual void exportTrajectories(std::ofstream& file) = 0;
};

template <class Space>
class DataExporterRMP : public DataExporter<Space> {
  using Vector = Eigen::Matrix<double, Space::dim, 1>;

 public:
  explicit DataExporterRMP(rmpcpp::PlannerRMP<Space>* planner)
      : planner(planner){};

  void exportTrajectories(std::ofstream& file) override;

 private:
  rmpcpp::PlannerRMP<Space>* planner;

  void exportTrajectory(rmpcpp::TrajectoryRMP<Space>* trajectory,
                        std::ofstream& file);
};

#endif  // RMPCPP_PLANNER_DATA_EXPORTER_H
