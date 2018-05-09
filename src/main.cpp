/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <chrono>
#include <fstream>
#include <unistd.h>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>

#include "problems/vrp.h"
#include "structures/typedefs.h"
#include "structures/cl_args.h"
#include "structures/vroom/input/input.h"
#include "utils/exceptions.h"
#include "utils/input_parser.h"
#include "utils/output_json.h"
#include "utils/version.h"

void display_usage() {
  std::string usage = "VROOM Copyright (C) 2015-2018, Julien Coupey\n";
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
  usage +=
    "\t-i FILE,\t read input from FILE rather than from\n\t\t\t "
    "command-line\n";
  usage += "\t-l,\t\t use libosrm rather than osrm-routed\n";
  usage += "\t-o OUTPUT,\t output file name\n";
  usage += "\t-t THREADS,\t number of threads to use\n";
  usage += "\t-v,\t\t turn on verbose output\n";
  usage += "\t-V,\t\t turn on verbose output with all details";
  std::cout << usage << std::endl;
  exit(0);
}

int main(int argc, char** argv) {
  // Log formatting.
  boost::log::add_console_log(std::cout,
                              boost::log::keywords::format = "%Message%");

  // Load default command-line options.
  cl_args_t cl_args;

  // Parsing command-line arguments.
  const char* optString = "a:gi:lm:o:p:t:vVh?";
  int opt = getopt(argc, argv, optString);

  std::string nb_threads_arg = std::to_string(cl_args.nb_threads);

  while (opt != -1) {
    switch (opt) {
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
    case 'l':
      cl_args.use_libosrm = true;
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

  try {
    // Needs to be done after previous switch to make sure the
    // appropriate output file is set.
    cl_args.nb_threads = std::stoul(nb_threads_arg);
  } catch (const std::exception& e) {
    std::string message = "Wrong value for number of threads.";
    std::cerr << "[Error] " << message << std::endl;
    write_to_json({1, message}, false, cl_args.output_file);
    exit(1);
  }

  if (cl_args.input_file.empty()) {
    // Getting input from command-line.
    if (argc == optind) {
      // Missing argument!
      display_usage();
    }
    cl_args.input = argv[optind];
  } else {
    // Getting input from provided file.
    std::ifstream ifs(cl_args.input_file);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    cl_args.input = buffer.str();
  }

  // Log level.
  boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                      cl_args.log_level);

  try {
    // Build problem.
    input problem_instance = parse(cl_args);

    solution sol = problem_instance.solve(cl_args.nb_threads);

    // Write solution.
    write_to_json(sol, cl_args.geometry, cl_args.output_file);
  } catch (const custom_exception& e) {
    std::cerr << "[Error] " << e.get_message() << std::endl;
    write_to_json({1, e.get_message()}, false, cl_args.output_file);
    exit(1);
  }
#if LIBOSRM
  catch (const std::exception& e) {
    // Should only occur when trying to use libosrm without running
    // osrm-datastore. It would be good to be able to catch an
    // osrm::util::exception for this. See OSRM issue #2813.
    std::cerr << "[Error] " << e.what() << std::endl;
    write_to_json({1, e.what()}, false, cl_args.output_file);
    exit(1);
  }
#endif

  return 0;
}
