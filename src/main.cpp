/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <fstream>
#include <iostream>
#include <sstream>

#if USE_LIBOSRM
#include "osrm/exception.hpp"
#endif

#include "../include/cxxopts/include/cxxopts.hpp"

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
  std::string limit_arg;
  std::string output_file;
  unsigned exploration_level;

  cxxopts::Options options("vroom",
                           "VROOM Copyright (C) 2015-2025, Julien Coupey\n"
                           "Version: " +
                             vroom::get_version() +
                             "\n\n"
                             "A command-line utility to solve complex vehicle "
                             "routing problems.\n");

  // clang-format off
  constexpr unsigned cxxopts_width = 80;
  options
    .set_width(cxxopts_width)
    .set_tab_expansion()
    .add_options("Solving")
    ("a,host",
     "host for the routing profile, optionally with a URL path, e.g 'routing.openstreetmap.de/routed-car'",
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
     cxxopts::value<std::string>(output_file))
    ("p,port",
     "host port for the routing profile",
     cxxopts::value<std::vector<std::string>>(port_args)->default_value({vroom::DEFAULT_PROFILE + ":5000"}))
    ("r,router",
     "osrm, libosrm, ors or valhalla",
     cxxopts::value<std::string>(router_arg)->default_value("osrm"))
    ("t,threads",
     "number of available threads",
     cxxopts::value<unsigned>(cl_args.nb_threads)->default_value(std::to_string(vroom::DEFAULT_THREADS_NUMBER)))
    ("v,version", "output version information and exit")
    ("x,explore",
     "exploration level to use (0..5)",
     cxxopts::value<unsigned>(exploration_level)->default_value(std::to_string(vroom::DEFAULT_EXPLORATION_LEVEL)))
    ("stdin",
     "optional input positional arg",
     cxxopts::value<std::string>(cl_args.input));

  // we don't want to print debug args on --help
  options.add_options("debug_group")
    ("f,apply-tsp-fix",
     "apply experimental TSPFix local search operator",
     cxxopts::value<bool>(cl_args.apply_TSPFix)->default_value("false"));

  // clang-format on
  try {
    options.parse_positional({"stdin"});
    options.positional_help("OPTIONAL INLINE JSON");
    auto parsed_args = options.parse(argc, argv);

    if (!output_file.empty()) {
      // Check we're able to write to the output file.
      std::ofstream out_stream(output_file);
      if (!out_stream) {
        const auto exc =
          vroom::InputException("Can't write to file: " + output_file);
        std::cerr << "[Error] " << exc.message << std::endl;
        vroom::io::write_to_json(exc);
        exit(exc.error_code);
      }
      out_stream.close();

      cl_args.output_file = output_file;
    }

    try {
      if (!limit_arg.empty()) {
        // Internally timeout is in milliseconds.
        constexpr unsigned s_to_ms = 1000;
        cl_args.timeout =
          std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
            s_to_ms * std::stof(limit_arg)));
      }
    } catch (const std::exception&) {
      throw cxxopts::exceptions::exception("Argument '" + limit_arg +
                                           "' failed to parse");
    }

    if (parsed_args.count("help") != 0) {
      std::cout << options.help({"Solving"}) << "\n";
      exit(0);
    }

    if (parsed_args.count("version") != 0) {
      std::cout << "vroom " << vroom::get_version() << "\n";
      exit(0);
    }
  } catch (const cxxopts::exceptions::exception& e) {
    // cxxopts outputs the failed parameter but no other details, so we add some
    // (likely) context
    const auto exc = vroom::InputException(std::string(e.what()) +
                                           ": invalid numerical value.");
    std::cerr << "[Error] " << exc.message << std::endl;
    vroom::io::write_to_json(exc, cl_args.output_file);
    exit(exc.error_code);
  }

  // parse and update some params
  for (const auto& host : host_args) {
    vroom::io::update_host(cl_args.servers, host);
  }
  for (const auto& port : port_args) {
    vroom::io::update_port(cl_args.servers, port);
  }
  exploration_level = std::min(exploration_level, vroom::MAX_EXPLORATION_LEVEL);
  cl_args.set_exploration_level(exploration_level);

  // Determine routing engine (defaults to ROUTER::OSRM).
  if (router_arg == "libosrm") {
    cl_args.router = vroom::ROUTER::LIBOSRM;
  } else if (router_arg == "ors") {
    cl_args.router = vroom::ROUTER::ORS;
  } else if (router_arg == "valhalla") {
    cl_args.router = vroom::ROUTER::VALHALLA;
  } else if (!router_arg.empty() && router_arg != "osrm") {
    const auto e =
      vroom::InputException("Invalid routing engine: " + router_arg + ".");
    std::cerr << "[Error] " << e.message << std::endl;
    vroom::io::write_to_json(e, cl_args.output_file);
    exit(e.error_code);
  } else {
    cl_args.router = vroom::ROUTER::OSRM;
  }

  // Get input problem from first input file, then positional arg,
  // then stdin.
  if (!cl_args.input_file.empty()) {
    const std::ifstream ifs(cl_args.input_file);
    if (!ifs) {
      const auto exc =
        vroom::InputException("Can't read file: " + cl_args.input_file);
      std::cerr << "[Error] " << exc.message << std::endl;
      vroom::io::write_to_json(exc, cl_args.output_file);
      exit(exc.error_code);
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();

    cl_args.input = buffer.str();
  } else if (cl_args.input.empty()) {
    // No input file provided and no positional arg, check stdin.
    std::stringstream buffer;
    buffer << std::cin.rdbuf();
    cl_args.input = buffer.str();
  }

  try {
    // Build problem.
    vroom::Input problem_instance(cl_args.servers,
                                  cl_args.router,
                                  cl_args.apply_TSPFix);
    vroom::io::parse(problem_instance, cl_args.input, cl_args.geometry);

    const vroom::Solution sol = (cl_args.check)
                                  ? problem_instance.check(cl_args.nb_threads)
                                  : problem_instance.solve(cl_args.nb_searches,
                                                           cl_args.depth,
                                                           cl_args.nb_threads,
                                                           cl_args.timeout);

    // Write solution.
    vroom::io::write_to_json(sol,
                             cl_args.output_file,
                             problem_instance.report_distances());
  } catch (const vroom::Exception& e) {
    std::cerr << "[Error] " << e.message << std::endl;
    vroom::io::write_to_json(e, cl_args.output_file);
    exit(e.error_code);
  }
#if USE_LIBOSRM
  catch (const osrm::exception& e) {
    // In case of an unhandled routing error.
    const auto exc =
      vroom::RoutingException("Routing problem: " + std::string(e.what()));
    std::cerr << "[Error] " << exc.message << std::endl;
    vroom::io::write_to_json(exc, cl_args.output_file);
    exit(exc.error_code);
  }
#endif
  catch (const std::exception& e) {
    // In case of an unhandled internal error.
    const auto exc = vroom::InternalException(e.what());
    std::cerr << "[Error] " << exc.message << std::endl;
    vroom::io::write_to_json(exc, cl_args.output_file);
    exit(exc.error_code);
  }

  return 0;
}
