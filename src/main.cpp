/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <fstream>
#include <iostream>
#include <sstream>
#ifndef _WIN32
#include <unistd.h>
#endif

#if USE_LIBOSRM
#include "osrm/exception.hpp"
#endif

#include "../include/cxxopts.hpp"
#include "problems/vrp.h"
#include "structures/cl_args.h"
#include "utils/helpers.h"
#include "utils/input_parser.h"
#include "utils/output_json.h"
#include "utils/version.h"

int main(int argc, char** argv) {
  vroom::io::CLArgs cl_args;
  std::string host_arg;
  std::string port_arg;
  std::string router_arg;
  std::string limit_arg;
  std::string nb_threads_arg;
  std::vector<std::string> heuristic_params_arg;

  // clang-format off
  cxxopts::Options options(
    "vroom",
    "VROOM Copyright (C) 2015-2021, Julien Coupey\n"
    "Version: " + vroom::get_version() + "\n"
    "A command-line utility to solve complex vehicle routing problems."
    );

  options.add_options()
    ("h,help", "Print this help message.")
    ("v,version", "Print the version of this software.")
    ("a,host", "The host for the routing profile, e.g. '" + vroom::DEFAULT_PROFILE + ":0.0.0.0'", cxxopts::value<std::string>(host_arg)->default_value("car:0.0.0.0"))
    ("c,choose-eta", "Choose ETA for custom routes and report violations.", cxxopts::value<bool>(cl_args.check)->default_value("false"))
    ("g,geometry", "Add detailed route geometry and indicators", cxxopts::value<bool>(cl_args.geometry)->default_value("false"))
    ("i,input-file", "Read input from 'input-file' rather than from stdin", cxxopts::value<std::string>(cl_args.input_file))
    ("l,limit", "Stop solving process after 'limit' seconds", cxxopts::value<std::string>(limit_arg)->default_value(""))
    ("o,output", "Output file name", cxxopts::value<std::string>(cl_args.output_file))
    ("p,port", "The host port for the routing profile, e.g. '" + vroom::DEFAULT_PROFILE + ":5000'", cxxopts::value<std::string>(port_arg)->default_value("car:5000"))
    ("r,router", "osrm, libosrm, ors or valhalla", cxxopts::value<std::string>(router_arg)->default_value("osrm"))
    ("t,threads", "Number of threads to use", cxxopts::value<unsigned>(cl_args.nb_threads)->default_value("4"))
    ("x,explore", "Exploration level to use (0..5)", cxxopts::value<unsigned>(cl_args.exploration_level)->default_value("5"))
    ("e,heuristic-param", "Heuristic parameter, useful for debugging", cxxopts::value<std::vector<std::string>>(heuristic_params_arg));
  // clang-format on

  auto parsed_args = options.parse(argc, argv);

  if (parsed_args.count("help")) {
    std::cout << options.help() << "\n";
    exit(0);
  }

  if (parsed_args.count("version")) {
    std::cout << "vroom " << vroom::get_version() << "\n";
    exit(0);
  }

  // parse and update some params
  vroom::io::update_host(cl_args.servers, host_arg);
  vroom::io::update_port(cl_args.servers, port_arg);
  if (!limit_arg.empty()) {
    // Internally timeout is in milliseconds.
    cl_args.timeout = 1000 * std::stof(limit_arg);
  }
  cl_args.exploration_level =
    std::min(cl_args.exploration_level, cl_args.max_exploration_level);

  // Determine routing engine (defaults to ROUTER::OSRM).
  if (router_arg == "libosrm") {
    cl_args.router = vroom::ROUTER::LIBOSRM;
  } else if (router_arg == "ors") {
    cl_args.router = vroom::ROUTER::ORS;
  } else if (router_arg == "valhalla") {
    cl_args.router = vroom::ROUTER::VALHALLA;
  } else if (!router_arg.empty() and router_arg != "osrm") {
    auto error_code = vroom::utils::get_code(vroom::ERROR::INPUT);
    std::string message = "Invalid routing engine: " + router_arg + ".";
    std::cerr << "[Error] " << message << std::endl;
    vroom::io::write_to_json({error_code, message}, false, cl_args.output_file);
    exit(error_code);
  }

  try {
    // Force heuristic parameters from the command-line, useful for
    // debugging.
    std::transform(heuristic_params_arg.begin(),
                   heuristic_params_arg.end(),
                   std::back_inserter(cl_args.h_params),
                   [](const auto& str_param) {
                     return vroom::utils::str_to_heuristic_param(str_param);
                   });
  } catch (const vroom::Exception& e) {
    auto error_code = vroom::utils::get_code(e.error);
    std::cerr << "[Error] " << e.message << std::endl;
    vroom::io::write_to_json({error_code, e.message},
                             false,
                             cl_args.output_file);
    exit(error_code);
  }

  // Read input problem
  if (optind == argc) {
    std::stringstream buffer;
    if (cl_args.input_file.empty()) {
      // Getting input from stdin.
      buffer << std::cin.rdbuf();
    } else {
      // Getting input from provided file.
      std::ifstream ifs(cl_args.input_file);
      buffer << ifs.rdbuf();
    }
    cl_args.input = buffer.str();
  } else {
    // Getting input from command-line.
    cl_args.input = argv[optind];
  }

  try {
    // Build problem.
    vroom::Input problem_instance = vroom::io::parse(cl_args);

    vroom::Solution sol = (cl_args.check)
                            ? problem_instance.check(cl_args.nb_threads)
                            : problem_instance.solve(cl_args.exploration_level,
                                                     cl_args.nb_threads,
                                                     cl_args.timeout,
                                                     cl_args.h_params);

    // Write solution.
    vroom::io::write_to_json(sol, cl_args.geometry, cl_args.output_file);
  } catch (const vroom::Exception& e) {
    auto error_code = vroom::utils::get_code(e.error);
    std::cerr << "[Error] " << e.message << std::endl;
    vroom::io::write_to_json({error_code, e.message},
                             false,
                             cl_args.output_file);
    exit(error_code);
  }
#if USE_LIBOSRM
  catch (const osrm::exception& e) {
    // In case of an unhandled routing error.
    auto error_code = vroom::utils::get_code(vroom::ERROR::ROUTING);
    auto message = "Routing problem: " + std::string(e.what());
    std::cerr << "[Error] " << message << std::endl;
    vroom::io::write_to_json({error_code, message}, false, cl_args.output_file);
    exit(error_code);
  }
#endif
  catch (const std::exception& e) {
    // In case of an unhandled internal error.
    auto error_code = vroom::utils::get_code(vroom::ERROR::INTERNAL);
    std::cerr << "[Error] " << e.what() << std::endl;
    vroom::io::write_to_json({error_code, e.what()},
                             false,
                             cl_args.output_file);
    exit(error_code);
  }

  return 0;
}
