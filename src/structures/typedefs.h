#ifndef VROOM_TYPEDEFS_H
#define VROOM_TYPEDEFS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <limits>
#include <boost/log/trivial.hpp>

// To easily differentiate variable types.
using index_t = uint16_t;
using distance_t = uint32_t;
using duration_t = uint32_t;

// Setting max value would cause trouble with further additions.
constexpr distance_t INFINITE_DISTANCE = 3 * (std::numeric_limits<distance_t>::max() / 4);

struct timing_t{
  // Computing times in milliseconds.
  duration_t matrix_loading;
  duration_t heuristic;
  duration_t local_search;
  duration_t total_duration;
};

struct cl_args_t{
  // Listing command-line options.
  std::string osrm_address;                      // -a
  bool geometry;                                 // -g
  std::string input_file;                        // -i
  std::string output_file;                       // -o
  std::string osrm_port;                         // -p
  boost::log::trivial::severity_level log_level; // -v
  std::string input;                             // cl arg
  unsigned nb_threads;                           // -t
  std::string osrm_profile;                      // -m
  bool use_osrm;                                 // default
  bool use_tsplib_loader;                        // auto-detected
  bool use_json_loader;                          // -j
  // Default values.
  cl_args_t():
    osrm_address("0.0.0.0"),
    geometry(false),
    osrm_port("5000"),
    log_level(boost::log::trivial::error),
    nb_threads(2),
    osrm_profile("car"){}
};

struct pbl_context_t{
  bool force_start;
  index_t start;
  bool force_end;
  index_t end;
};

#endif
