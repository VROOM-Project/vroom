#ifndef CL_ARGS_H
#define CL_ARGS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

namespace vroom {

struct CLArgs {
  // Listing command-line options.
  std::string osrm_address;   // -a
  bool geometry;              // -g
  std::string input_file;     // -i
  std::string output_file;    // -o
  std::string osrm_port;      // -p
  bool use_libosrm;           // -l
  std::string input;          // cl arg
  unsigned nb_threads;        // -t
  std::string osrm_profile;   // -m
  unsigned exploration_level; // -x

  static const unsigned max_exploration_level;

  CLArgs();
};

} // namespace vroom

#endif
