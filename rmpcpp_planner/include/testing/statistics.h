//
// Created by mpantic on 04.10.22.
//

#ifndef RMPCPP_PLANNER_STATISTICS_H
#define RMPCPP_PLANNER_STATISTICS_H
#include <cmath>
#include <fstream>
#include <numeric>
#include <utility>
#include <vector>

namespace rmpcpp {

class RunStatistics {
 public:
  typedef struct {
    int index;              // index of entry
    int success;            // 1 if run was successfull, 0 otherwise
    double time_sec;        // time it took in seconds
    int integration_steps;  // number of steps the integration lasted
    double length;          // final length of trajectory
    double smoothness;      // final smoothness of trajectory
    double integration_steps_per_sec;
    double world_density;
  } Line;

  void reset(const std::string& map_type) { map_type_ = map_type; }

  void clear() { lines_.clear(); }

  void add(Line line) { lines_.push_back(line); }

  const std::vector<Line>& getLines() const { return lines_; };

  template <typename t>
  double getSum(t Line::*ref) const {
    if (lines_.size() == 0) {
      return 0;
    }

    t sum = std::accumulate(
        lines_.begin(), lines_.end(),
        (t)0.0,  // important: cast to (t) must stay here, otherwise this might
                 // be evaluated as int, even if t is double. Which then causes
                 // a wrong evaluation.
        [&](const t init, const Line& line) { return (t)init + line.*ref; });

    return sum;
  }

  template <typename t>
  double getMean(t Line::*ref) const {
    if (lines_.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    return getSum(ref) / lines_.size();
  }

  template <typename t>
  double getStd(t Line::*ref) const {
    if (lines_.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    double mean = getMean(ref);

    t sum_squared_error = std::accumulate(lines_.begin(), lines_.end(), (t)0.0,
                                          [&](const t init, const Line& line) {
                                            double err = line.*ref - mean;
                                            return (t)(init) + (err * err);
                                          });

    return std::sqrt(sum_squared_error / lines_.size());
  }

  void writeSummary(std::ofstream& file) const {
    file << "obstacles success success_rate "
            "success_time_mean success_time_std "
            "length_discrete_mean length_discrete_std "
            "length_mean length_std "
            "integration_steps_mean integration_steps_std "
            "integrations_per_sec_mean integrations_per_sec_std "
            "smoothness_mean smoothness_std "
            "world_density_mean world_density_std "
         << std::endl;

    file << map_type_ << sep_;
    file << getSum(&Line::success) << sep_;
    file << getMean(&Line::success) << sep_;
    file << getMean(&Line::time_sec) << sep_;
    file << getStd(&Line::time_sec) << sep_;
    file << getMean(&Line::integration_steps) << sep_;
    file << getStd(&Line::integration_steps) << sep_;
    file << getMean(&Line::length) << sep_;
    file << getStd(&Line::length) << sep_;
    file << getMean(&Line::integration_steps) << sep_;
    file << getStd(&Line::integration_steps) << sep_;
    file << getMean(&Line::integration_steps_per_sec) << sep_;
    file << getStd(&Line::integration_steps_per_sec) << sep_;
    file << getMean(&Line::smoothness) << sep_;
    file << getStd(&Line::smoothness) << sep_;
    file << getMean(&Line::world_density) << sep_;
    file << getStd(&Line::world_density) << sep_;
    file << std::endl;

    file.flush();
  }

  void writeLines(std::ofstream& file) const {
    file << "map index "
            "success_time "
            "length_discrete "
            "length "
            "integration_steps "
            "integrations_per_sec "
            "world_density "
            "smoothness "
         << std::endl;
    for (const auto& line : lines_) {
      if (!line.success) {
        // skip failed runs
        continue;
      }

      file << map_type_ << sep_ << line.index << sep_ << line.time_sec << sep_
           << line.integration_steps << sep_ << line.length << sep_
           << line.integration_steps << sep_ << line.integration_steps_per_sec
           << sep_ << line.world_density << sep_ << line.smoothness
           << std::endl;
    }
    file.flush();
  }

 private:
  std::string map_type_;  // map name

  const char sep_ = '\t';
  std::vector<Line> lines_;
};
}  // namespace rmpcpp
#endif  // RMPCPP_PLANNER_STATISTICS_H
