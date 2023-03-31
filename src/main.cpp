/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <fstream>
#include <iostream>
#include <sstream>

#if USE_LIBOSRM
#include "osrm/exception.hpp"
#endif

#include "../include/cxxopts/include/cxxopts.hpp"

#include "problems/vrp.h"
#include "structures/cl_args.h"
#include "utils/exception.h"
#include "utils/helpers.h"
#include "utils/input_parser.h"
#include "utils/output_json.h"
#include "utils/version.h"

int main(int argc, char** argv) {
  vroom::io::CLArgs cl_args;
  std::vector<std::string> host_args;
  std::vector<std::string> port_args;
  std::string router_arg;
  std::string extra_args;
  std::string limit_arg;
  std::vector<std::string> heuristic_params_arg;

  cxxopts::Options options("vroom",
                           "VROOM Copyright (C) 2015-2022, Julien Coupey\n"
                           "Version: " +
                             vroom::get_version() +
                             "\n\n"
                             "A command-line utility to solve complex vehicle "
                             "routing problems.\n");

  // clang-format off
  options
    .set_width(80)
    .set_tab_expansion()
    .add_options("Solving")
    ("a,host",
     "host for the routing profile",
     cxxopts::value<std::vector<std::string>>(host_args)->default_value({vroom::DEFAULT_PROFILE + ":0.0.0.0"}))
    ("c,choose-eta",
     "choose ETA for custom routes and report violations",
     cxxopts::value<bool>(cl_args.check)->default_value("false"))
    ("g,geometry",
     "add detailed route geometry and distance",
     cxxopts::value<bool>(cl_args.geometry)->default_value("false"))
    ("h,help", "display this help and exit")
    ("i,input",
     "read input from a file rather than from stdin",
     cxxopts::value<std::string>(cl_args.input_file))
    ("l,limit",
     "stop solving process after 'limit' seconds",
     cxxopts::value<std::string>(limit_arg))
    ("o,output",
     "write output to a file rather than stdout",
     cxxopts::value<std::string>(cl_args.output_file))
    ("p,port",
     "host port for the routing profile",
     cxxopts::value<std::vector<std::string>>(port_args)->default_value({vroom::DEFAULT_PROFILE + ":5000"}))
    ("r,router",
     "osrm, libosrm, ors or valhalla",
     cxxopts::value<std::string>(router_arg)->default_value("osrm"))
    ("E,extra_args",
     "extra args",
      cxxopts::value<std::string>(cl_args.extra_args))
    ("t,threads",
     "number of available threads",
     cxxopts::value<unsigned>(cl_args.nb_threads)->default_value(std::to_string(vroom::DEFAULT_THREADS_NUMBER)))
    ("v,version", "output version information and exit")
    ("x,explore",
     "exploration level to use (0..5)",
     cxxopts::value<unsigned>(cl_args.exploration_level)->default_value(std::to_string(vroom::DEFAULT_EXPLORATION_LEVEL)))
    ("stdin",
     "optional input positional arg",
     cxxopts::value<std::string>(cl_args.input));

  // clang-format on
  try {
    // we don't want to print debug args on --help
    options.add_options("debug_group")("e,heuristic-param",
                                       "Heuristic parameter",
                                       cxxopts::value<std::vector<std::string>>(
                                         heuristic_params_arg));

    options.parse_positional({"stdin"});
    options.positional_help("OPTIONAL INLINE JSON");
    auto parsed_args = options.parse(argc, argv);

    try {
      if (!limit_arg.empty()) {
        // Internally timeout is in milliseconds.
        cl_args.timeout =
          std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
            1000 * std::stof(limit_arg)));
      }
    } catch (const std::exception& e) {
      throw cxxopts::OptionException("Argument '" + limit_arg +
                                     "' failed to parse");
    }

    if (parsed_args.count("help")) {
      std::cout << options.help({"Solving"}) << "\n";
      exit(0);
    }

    if (parsed_args.count("version")) {
      std::cout << "vroom " << vroom::get_version() << "\n";
      exit(0);
    }
  } catch (const cxxopts::OptionException& e) {
    // cxxopts outputs the failed parameter but no other details, so we add some
    // (likely) context
    const auto exc = vroom::InputException(": invalid numerical value.");
    const auto msg = e.what() + exc.message;
    std::cerr << "[Error] " << msg << std::endl;
    vroom::io::write_to_json({exc.error_code, msg}, false, cl_args.output_file);
    exit(exc.error_code);
  }

  // parse and update some params
  for (const auto& host : host_args) {
    vroom::io::update_host(cl_args.servers, host);
  }
  for (const auto& port : port_args) {
    vroom::io::update_port(cl_args.servers, port);
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
    auto error_code = vroom::InputException("").error_code;
    std::string message = "Invalid routing engine: " + router_arg + ".";
    std::cerr << "[Error] " << message << std::endl;
    vroom::io::write_to_json({error_code, message}, false, cl_args.output_file);
    exit(error_code);
  } else {
    cl_args.router = vroom::ROUTER::OSRM;
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
    std::cerr << "[Error] " << e.message << std::endl;
    vroom::io::write_to_json({e.error_code, e.message},
                             false,
                             cl_args.output_file);
    exit(e.error_code);
  }

  // Read input problem
  if (cl_args.input.empty()) {
    std::stringstream buffer;
    if (!cl_args.input_file.empty()) {
      std::ifstream ifs(cl_args.input_file);
      buffer << ifs.rdbuf();
    } else {
      buffer << std::cin.rdbuf();
    }
    cl_args.input = buffer.str();
  }

  try {
    // Build problem.
    vroom::Input problem_instance(cl_args.servers, cl_args.router, cl_args.extra_args);
    vroom::io::parse(problem_instance, cl_args.input, cl_args.geometry);

    vroom::Solution sol = (cl_args.check)
                            ? problem_instance.check(cl_args.nb_threads)
                            : problem_instance.solve(cl_args.exploration_level,
                                                     cl_args.nb_threads,
                                                     cl_args.timeout,
                                                     cl_args.h_params);

    // Write solution.
    vroom::io::write_to_json(sol, cl_args.geometry, cl_args.output_file);
  } catch (const vroom::Exception& e) {
    std::cerr << "[Error] " << e.message << std::endl;
    vroom::io::write_to_json({e.error_code, e.message},
                             false,
                             cl_args.output_file);
    exit(e.error_code);
  }
#if USE_LIBOSRM
  catch (const osrm::exception& e) {
    // In case of an unhandled routing error.
    auto error_code = vroom::RoutingException("").error_code;
    auto message = "Routing problem: " + std::string(e.what());
    std::cerr << "[Error] " << message << std::endl;
    vroom::io::write_to_json({error_code, message}, false, cl_args.output_file);
    exit(error_code);
  }
#endif
  catch (const std::exception& e) {
    // In case of an unhandled internal error.
    auto error_code = vroom::InternalException("").error_code;
    std::cerr << "[Error] " << e.what() << std::endl;
    vroom::io::write_to_json({error_code, e.what()},
                             false,
                             cl_args.output_file);
    exit(error_code);
  }

  return 0;
}
