#ifndef VROOM_TYPEDEFS_H
#define VROOM_TYPEDEFS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <limits>
#include <string>

#include <boost/log/trivial.hpp>
#include <boost/optional.hpp>

// To easily differentiate variable types.
using ID_t = uint64_t;
using index_t = uint16_t;
using cost_t = uint32_t;
using distance_t = uint32_t;
using duration_t = uint32_t;
using coordinate_t = double;

// Type helpers.
using coords_t = std::array<coordinate_t, 2>;
using optional_coords_t = boost::optional<coords_t>;

// Setting max value would cause trouble with further additions.
constexpr cost_t INFINITE_COST = 3 * (std::numeric_limits<cost_t>::max() / 4);

struct cl_args_t {
  // Listing command-line options.
  std::string osrm_address;                      // -a
  bool geometry;                                 // -g
  std::string input_file;                        // -i
  std::string output_file;                       // -o
  std::string osrm_port;                         // -p
  bool use_libosrm;                              // -l
  boost::log::trivial::severity_level log_level; // -v
  std::string input;                             // cl arg
  unsigned nb_threads;                           // -t
  std::string osrm_profile;                      // -m
  // Default values.
  cl_args_t()
    : osrm_address("0.0.0.0"),
      geometry(false),
      osrm_port("5000"),
      use_libosrm(false),
      log_level(boost::log::trivial::error),
      nb_threads(2),
      osrm_profile("car") {
  }
};

// Problem types.
enum class PROBLEM_T { TSP };

// Available location status.
enum class TYPE { START, JOB, END };

#endif
