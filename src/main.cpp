/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <sstream>
#include <unistd.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include "./utils/version.h"
#include "./structures/typedefs.h"
#include "./heuristics/tsp_strategy.h"
#include "./loaders/problem_io.h"
#include "./loaders/tsplib_loader.h"
#include "./loaders/routed_loader.h"
#include "./loaders/json_loader.h"
#include "./utils/logger.h"

void display_usage(){
  std::string usage = "VROOM Copyright (C) 2015-2016, Julien Coupey\n";
  usage += "Version: " + get_version() + "\n";
  usage += "Usage:\n\tvroom [OPTION]... \"INPUT\"";
  usage += "\n\tvroom [OPTION]... -i FILE\n";
  usage += "Options:\n";
  usage += "\t-a ADDRESS\t OSRM server address (\"0.0.0.0\")\n";
  usage += "\t-p PORT,\t OSRM listening port (5000)\n";
  // usage += "\t-m MODE,\t OSRM profile name (car)\n";

  // The -m flag is only present as the profile name is part of the
  // OSRM v5 API. It is undocumented as OSRM doesn't implement
  // query-time profile selection (yet) so setting it will have no
  // effect for now.

  usage += "\t-g,\t\t get detailed route geometry for the solution\n";
  usage += "\t-i FILE,\t read input from FILE rather than from\n\t\t\t command-line\n";
  usage += "\t-j,\t\t use supplied matrix from JSON-object then from osrm-routed\n";
  usage += "\t-o OUTPUT,\t output file name\n";
  usage += "\t-t THREADS,\t number of threads to use\n";
  usage += "\t-v,\t\t turn on verbose output\n";
  usage += "\t-V,\t\t turn on verbose output with all details";
  std::cout << usage << std::endl;
  exit(0);
}

int main(int argc, char **argv){
  // Log formatting.
  boost::log::add_console_log(std::cout,
                              boost::log::keywords::format = "%Message%");

  // Load default command-line options.
  cl_args_t cl_args;

  // Parsing command-line arguments.
  const char* optString = "a:gi:lmj:o:p:t:vVh?";
  int opt = getopt(argc, argv, optString);

  std::string nb_threads_arg = std::to_string(cl_args.nb_threads);

  while(opt != -1) {
    switch(opt){
    case 'a':
      cl_args.osrm_address = optarg;
      break;
    case 'g':
      cl_args.geometry = true;
      break;
    case 'h':
      display_usage();
      break;
    case 'i':
      cl_args.input_file = optarg;
      break;
    case 'j':
      cl_args.use_json_loader = true;
      break;
    case 'm':
      cl_args.osrm_profile = optarg;
      break;
    case 'o':
      cl_args.output_file = optarg;
      break;
    case 'p':
      cl_args.osrm_port = optarg;
      break;
    case 't':
      nb_threads_arg = optarg;
      break;
    case 'v':
      cl_args.log_level = boost::log::trivial::info;
      break;
    case 'V':
      cl_args.log_level = boost::log::trivial::trace;
      break;
    default:
      break;
    }
    opt = getopt(argc, argv, optString);
  }

  try{
    // Needs to be done after previous switch to make sure the
    // appropriate output file is set.
    cl_args.nb_threads = std::stoul(nb_threads_arg);
  }
  catch(const std::exception& e){
    std::string message = "Wrong value for number of threads.";
    std::cerr << "[Error] " << message << std::endl;
    write_error(cl_args.output_file, message);
    exit(1);
  }

  if(cl_args.input_file.empty()){
    // Getting input from command-line.
    if(argc == optind){
      // Missing argument!
      display_usage();
    }
    cl_args.input = argv[optind];
  }
  else{
    // Getting input from provided file.
    std::ifstream ifs (cl_args.input_file);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    cl_args.input = buffer.str();
  }

  // Log level.
  boost::log::core::get()
    ->set_filter(boost::log::trivial::severity >= cl_args.log_level);

  try{
    timing_t computing_times;
    auto start_problem_loading = std::chrono::high_resolution_clock::now();
    BOOST_LOG_TRIVIAL(info)
      << "[Matrix] Start matrix computing and problem loading.";

    // Check, if input-file is in TSPLIB format
    cl_args.use_tsplib_loader = !(cl_args.input.find("DIMENSION") == std::string::npos);
    cl_args.use_osrm = !(cl_args.use_tsplib_loader || cl_args.use_json_loader);

    std::unique_ptr<problem_io<distance_t>> loader;
    if(cl_args.use_osrm){
      // Use osrm-routed.
      loader = std::make_unique<routed_loader>(cl_args.osrm_address,
                                               cl_args.osrm_port,
                                               cl_args.osrm_profile,
                                               cl_args.input);
    }
    else if(cl_args.use_tsplib_loader){
      loader = std::make_unique<tsplib_loader>(cl_args.input);
    }
    else if(cl_args.use_json_loader){
      loader = std::make_unique<json_loader>(cl_args.input);
    }
    else{   //Shall never happen!
      std::string message = "Internal unexpected indecision which loader to use.";
      std::cerr << "[Error] " << message << std::endl;
      write_error(cl_args.output_file, message);
      exit(1);
    }

    // Build problem.
    tsp asymmetric_tsp (loader->get_pbl_context(), loader->get_matrix());

    auto end_problem_loading = std::chrono::high_resolution_clock::now();
    computing_times.matrix_loading =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_problem_loading - start_problem_loading).count();

    BOOST_LOG_TRIVIAL(info) << "[Matrix] Done, took "
                            << computing_times.matrix_loading << " ms.";

    // Solve!
    // TODO: adapt return type.
    std::pair<std::list<index_t>, distance_t> solution
      = solve_atsp(asymmetric_tsp, cl_args.nb_threads, computing_times);

    // Write solution.
    write_solution(cl_args,
                   *loader,
                   solution.first,
                   solution.second,
                   computing_times);
  }
  catch(const custom_exception& e){
    std::cerr << "[Error] " << e.get_message() << std::endl;
    write_error(cl_args.output_file, e.get_message());
    exit(1);
  }

  return 0;
}
