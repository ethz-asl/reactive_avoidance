#include <chrono>
#include <iostream>
#include <numeric>
#include "testing/tester.h"
#include "testing/worldgen.h"
#include "testing/data_exporter.h"
#include <string>
#include "rmpcpp_planner/core/planner_rmp.h"
#include "testing/settings.h"
#include "boost/format.hpp"
#include "boost/variant.hpp"

Tester::Tester(const ParametersWrapper &parameters, const rmpcpp::TestSettings &settings) :
    worldgen(rmpcpp::WorldGen(rmpcpp::WorldGenSettings(settings))),
    parameters(parameters), settings(settings){
}

/**
 * Does a single planner run
 */
void Tester::run_single() {
    worldgen.seed(settings.seed);
    switch (settings.world_type) {
        case rmpcpp::SPHERES_ONLY_WORLD:
        case rmpcpp::SPHERES_BOX_WORLD:
            worldgen.generateRandomWorld(settings.obstacles);
            break;
    }

    switch (settings.planner_type) {
        case rmpcpp::RMP:
            planner = std::make_unique<rmpcpp::PlannerRMP<rmpcpp::Space<3>>>(parameters.parametersRMP);
            break;
        default:
            throw std::runtime_error("Not implemented");
    }

    /**
     * We set both ESDF and TSDF. Note that the RMP planner does not actually use the ESDF.
     */
    planner->setEsdf(worldgen.getEsdfLayer());
    planner->setTsdf(worldgen.getTsdfLayer());

    Eigen::Vector3d startv = worldgen.getStart();
    Eigen::Vector3d goalv = worldgen.getGoal();

    rmpcpp::State<dim> start(startv, {0.0, 0.0, 0.0});
    planner->setGoal(goalv);

    auto starttime = std::chrono::high_resolution_clock::now();
    planner->plan(start);
    auto endtime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endtime - starttime);


    if(planner->reachedGoal()){
        stats.successfulcount++;
        stats.timings.push_back(double(duration.count()) / 1000.0); // Timing in ms
        update_success_stats();
    }

    densities.push_back(worldgen.getDensity());
    std::string success = planner->reachedGoal() ? "Success: " : "Failure: ";
    std::cout << success << double(duration.count())/1000.0 << "ms" << std::endl;

}

void Tester::exportTrajectories(std::string path, const int i) {
    DataExporter<Space>* exporter;
    path.append("trajectory");

    switch (settings.planner_type) {
        case rmpcpp::RMP:
            exporter = new DataExporterRMP<Space>(dynamic_cast<rmpcpp::PlannerRMP<Space>*>(planner.get()));
            break;
    }

    boost::format index = boost::format("%03d") % std::to_string(i);
    path.append(index.str());
    std::string endpoints_path = path;
    path.append(".txt");
    std::ofstream file;
    file.open(path, std::ofstream::trunc); // clear file contents with trunc
    exporter->exportTrajectories(file);
    file.close();
    free(exporter);

    /** Export start and goal */
    endpoints_path.append("endpoints.txt");
    Eigen::IOFormat format(Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ", "", "", " ", "");
    file.open(endpoints_path, std::ofstream::trunc);
    file << "sx sy sz ex ey ez" << std::endl;
    file << worldgen.getStart().format(format) << worldgen.getGoal().format(format) << std::endl;
    file.close();
}

void Tester::exportWorld(std::string path, const int i) {
    path.append("world");

    switch (settings.world_type) {
        case rmpcpp::SPHERES_BOX_WORLD:
            path.append("sb");
            // fallthrough
        case rmpcpp::SPHERES_ONLY_WORLD:
            path.append((boost::format("%03d") % std::to_string(i)).str());
            break;
        case rmpcpp::CUSTOM_WORLD:
            /** Custom world stays the same, so only export the first time */
            if(i){
                return;
            }
            break;
    }

    path.append(".ply");
    worldgen.exportToPly(path);
}

void Tester::run() {
    for(run_index = 0; run_index < settings.n_runs; run_index++) {
        run_single();

        /** Export trajectories only if enabled */
        if(!settings.stats_only) {
            exportTrajectories(settings.data_path, run_index);
        }

        /** Export world only if enabled */
        if(!settings.stats_only and !settings.world_save_path.empty()){
            exportWorld(settings.world_save_path, run_index);
        }
        settings.seed++;
    }
    exportStats(settings.data_path);
}

/**
 * Export statistics to file
 * @param path
 */
void Tester::exportStats(std::string path) {
    std::string path_full(path);
    path.append("stats.txt");
    path_full.append("stats_full.txt");

    std::ofstream file;
    file.open(path, std::ofstream::trunc); // clear file contents with trunc

    /** For every statistic that needs a mean and std, add it to the end of the line, and to the vector below */
    file << "obstacles success success_rate "
            "success_time_mean success_time_std "
            "length_discrete_mean length_discrete_std "
            "length_mean length_std "
            "integration_steps_mean integration_steps_std "
            "integrations_per_sec_mean integrations_per_sec_std "
            "smoothness_mean smoothness_std "
            "world_density_mean world_density_std " << std::endl;

    std::stringstream line;
    std::stringstream obstacles;
    switch (settings.world_type) {
        case rmpcpp::SPHERES_ONLY_WORLD:
        case rmpcpp::SPHERES_BOX_WORLD:
            obstacles << settings.obstacles;
            break;
        case rmpcpp::CUSTOM_WORLD:
            /** In case of a custom world, 'obstacles' is just the world name */
            obstacles << settings.world_load_path.substr(settings.world_load_path.find_last_of("/\\") + 1);
            break;
    }
    line << obstacles.str() << " ";
    line << stats.successfulcount << " "
         << double(stats.successfulcount) / double(settings.n_runs);


    /** TODO: Write something nice to make this better */
    /** Mean and stddev get updated when update_mean_std is called! */

    double mean = 0.0, stddev = 0.0;
    auto update_mean_std = [&](std::vector<double> v){
        mean = 0.0; stddev=0.0;
        for(auto value :v){
          mean += value;
        }
        mean/=double(v.size());
        double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
        stddev = std::sqrt(sq_sum / double(v.size()) - mean * mean);
    };

    /** Add new mean and std parameters here, with a corresponding key in the string above */
    std::vector<std::vector<double>> vectorstats = {
            std::vector<double>(stats.timings.begin(), stats.timings.end()),
            std::vector<double>(stats.shortest_discrete_lengths.begin(), stats.shortest_discrete_lengths.end()),
            std::vector<double>(stats.shortest_lengths.begin(), stats.shortest_lengths.end()),
            std::vector<double>(stats.total_integration_steps.begin(), stats.total_integration_steps.end()),
            std::vector<double>(stats.integrations_per_sec.begin(), stats.integrations_per_sec.end()),
            std::vector<double>(stats.smoothness.begin(), stats.smoothness.end()),
            std::vector<double>(densities.begin(), densities.end()),
    };
    for(auto& v : vectorstats){
        update_mean_std(v);
        line << " " << mean << " " << stddev;
    }

    line << std::endl;
    std::string linestring = line.str();
    file << linestring;
    std::cout << linestring;

    file.close();

    /** Full stats */
    file.open(path_full, std::ofstream::trunc);

    file << "obstacles index "
            "success_time "
            "length_discrete "
            "length "
            "integration_steps "
            "integrations_per_sec "
            "world_density "
            "smoothness "<< std::endl;
    for(int i = 0; i < stats.timings.size(); i++){
        std::stringstream line;
        line << obstacles.str() << " "
             << stats.indices[i] << " "
             << stats.timings[i] << " "
             << stats.shortest_discrete_lengths[i] << " "
             << stats.shortest_lengths[i] << " "
             << stats.total_integration_steps[i] << " "
             << stats.integrations_per_sec[i] << " "
             << densities[i] << " "
            << stats.smoothness[i] << std::endl;
        linestring = line.str();
        file << linestring;
    }
    file.close();
}


void Tester::update_success_stats(){
    stats.total_integration_steps.push_back(planner->totalSteps());
    stats.integrations_per_sec.push_back(stats.total_integration_steps.back() / (stats.timings.back() / 1000.0));

    stats.success.push_back(int(planner->reachedGoal()));
    stats.indices.push_back(run_index);

    stats.shortest_lengths.push_back(planner->getShortestLengthToGoal());
    stats.shortest_discrete_lengths.push_back(planner->getShortestLengthToGoalDiscrete());

    stats.smoothness.push_back(planner->getSmoothnessToGoal());
}
