#ifndef CLARGS_H
#define CLARGS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include <boost/log/trivial.hpp>

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
  unsigned exploration_level;                    // -x

  static const unsigned max_exploration_level = 5;

  // Default values.
  cl_args_t()
    : osrm_address("0.0.0.0"),
      geometry(false),
      osrm_port("5000"),
      use_libosrm(false),
      log_level(boost::log::trivial::error),
      nb_threads(4),
      osrm_profile("car"),
      exploration_level(1) {
  }
};

#endif
