#ifndef VROOM_TYPEDEFS_H
#define VROOM_TYPEDEFS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <limits>
#include <string>

#include <boost/log/trivial.hpp>
#include <boost/optional.hpp>

// To easily differentiate variable types.
using index_t = uint16_t;
using distance_t = uint32_t;
using duration_t = uint32_t;
using coordinate_t = double;

// Type helper.
using optional_coords_t = boost::optional<std::array<coordinate_t, 2>>;

// Setting max value would cause trouble with further additions.
constexpr distance_t INFINITE_DISTANCE =
  3 * (std::numeric_limits<distance_t>::max() / 4);

struct cl_args_t{
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
      osrm_profile("car") {}
};

// Problem types.
enum class PROBLEM_T { TSP };

#endif
