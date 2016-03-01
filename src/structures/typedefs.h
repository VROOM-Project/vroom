#ifndef TYPEDEFS_H
#define TYPEDEFS_H

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

// Setting max value would cause trouble with further additions.
constexpr distance_t INFINITE_DISTANCE = 3 * (std::numeric_limits<distance_t>::max() / 4);

struct timing_t {
  // Computing times in milliseconds.
  unsigned long matrix_loading;
  unsigned long heuristic;
  unsigned long local_search;
  unsigned long total_duration;
};

struct cl_args_t {
  // Listing command-line options.
  std::string osrm_address;                      // -a
  bool geometry;                                 // -g
  std::string input_file;                        // -i
  std::string output_file;                       // -o
  std::string osrm_port;                         // -p
  boost::log::trivial::severity_level log_level; // -v
  std::string input;                             // cl arg
  bool force_start;                              // -s
  bool force_end;                                // -e
  unsigned nb_threads;                           // -t
  bool use_osrm;
  // Default values.
  cl_args_t():
    osrm_address("0.0.0.0"),
    geometry(false),
    osrm_port("5000"),
    log_level(boost::log::trivial::error),
    force_start(false),
    force_end(false),
    nb_threads(2) {}
};

#endif
