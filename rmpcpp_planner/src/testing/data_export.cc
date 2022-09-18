#include <iostream>

#include "testing/data_exporter.h"
#include "rmpcpp_planner/core/planner_rmp.h"

#include <fstream>

/**
 * Export trajectories to txt file
 * @tparam Space
 * @param path
 */
template<class Space>
void DataExporterRMP<Space>::exportTrajectories(std::ofstream& file) {
    file << "i " << rmpcpp::DataRMP<Space>::getHeaderFormat() << std::endl;
    for(auto& trajectory : *planner->getTrajectories()){
        exportTrajectory(trajectory.get(), file);
        if(trajectory.get()->getChildren()->size()){
            std::cout << "Branched\n";
        }
    }
}

template void DataExporterRMP<rmpcpp::Space<2>>::exportTrajectories(std::ofstream& file);
template void DataExporterRMP<rmpcpp::Space<3>>::exportTrajectories(std::ofstream& file);

template<class Space>
void DataExporterRMP<Space>::exportTrajectory(rmpcpp::TrajectoryRMP<Space>* trajectory, std::ofstream& file) {
    for(int i = 0; i < trajectory->stats.this_length_discrete; i++){
        file << i + trajectory->stats.start_length_discrete << (*trajectory)[i].format() << std::endl;
    }
    /** Recursively go over child trajectories */
    for(auto& child : *trajectory->getChildren()){
        exportTrajectory(child.get(), file);
    }
}

template void DataExporterRMP<rmpcpp::Space<2>>::exportTrajectory(rmpcpp::TrajectoryRMP<rmpcpp::Space<2>>* trajectory, std::ofstream& file);
template void DataExporterRMP<rmpcpp::Space<3>>::exportTrajectory(rmpcpp::TrajectoryRMP<rmpcpp::Space<3>>* trajectory, std::ofstream& file);


