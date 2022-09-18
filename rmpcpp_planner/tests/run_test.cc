#include <iostream>
#include <boost/program_options.hpp>
#include "testing/tester.h"
#include "testing/parser.h"

namespace po = boost::program_options;
/**
 * Binary used to generate random worlds, do a run and save the trajectory and possibly the world file.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char* argv[]){
    po::variables_map opts;

    Parser parser(argc, argv);
    if(!parser.parse()){
        return 1;
    }

    Tester tester(parser.getParameters(), parser.getSettings());
    tester.run();
}
