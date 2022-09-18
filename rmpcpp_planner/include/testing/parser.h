#ifndef RMPCPP_PLANNER_PARSER_H
#define RMPCPP_PLANNER_PARSER_H

#include <boost/program_options.hpp>

#include "testing/settings.h"
#include "testing/tester.h"

namespace po = boost::program_options;

class Parser {
 public:
  Parser(int argc, char* argv[]);

  bool parse();
  rmpcpp::TestSettings getSettings();
  ParametersWrapper getParameters();

 private:
  ParametersWrapper getRMPParameters();
  po::variables_map opts;
};

#endif  // RMPCPP_PLANNER_PARSER_H
