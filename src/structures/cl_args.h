#ifndef CL_ARGS_H
#define CL_ARGS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/typedefs.h"

namespace vroom {
namespace io {

struct CLArgs {
  // Listing command-line options.
  Server server;              // -a and -p
  bool geometry;              // -g
  std::string input_file;     // -i
  std::string output_file;    // -o
  ROUTER router;              // -r
  std::string input;          // cl arg
  unsigned nb_threads;        // -t
  unsigned exploration_level; // -x

  static const unsigned max_exploration_level;

  CLArgs();
};

} // namespace io
} // namespace vroom

#endif
