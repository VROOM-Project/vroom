/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015-2016, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TYPEDEFS_H
#define TYPEDEFS_H

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
  bool use_osrm;
  // Default values.
  cl_args_t():
    osrm_address("0.0.0.0"),
    geometry(false),
    osrm_port("5000"),
    log_level(boost::log::trivial::error),
    force_start(false),
    force_end(false) {}
};

#endif
