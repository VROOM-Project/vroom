#ifndef CL_ARGS_H
#define CL_ARGS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <unordered_map>

#include "structures/typedefs.h"

namespace vroom::io {

// Profile name used as key.
using Servers =
  std::unordered_map<std::string, Server, StringHash, std::equal_to<>>;

struct CLArgs {
  // Listing command-line options.
  Servers servers;         // -a and -p
  bool check;              // -c
  bool apply_TSPFix;       // -f
  bool geometry;           // -g
  std::string input_file;  // -i
  Timeout timeout;         // -l
  std::string output_file; // -o
  ROUTER router;           // -r
  std::string input;       // cl arg
  unsigned nb_threads;     // -t
  unsigned nb_searches;    // derived from -x
  unsigned depth;          // derived from -x

  void set_exploration_level(unsigned exploration_level);
};

void update_host(Servers& servers, std::string_view value);

void update_port(Servers& servers, std::string_view value);

} // namespace vroom::io

#endif
