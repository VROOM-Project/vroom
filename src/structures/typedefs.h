#ifndef VROOM_TYPEDEFS_H
#define VROOM_TYPEDEFS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <limits>
#include <list>
#include <unordered_set>
#include <vector>

#include <boost/optional.hpp>

// To easily differentiate variable types.
using ID_t = uint64_t;
using index_t = uint16_t;
using cost_t = uint32_t;
using distance_t = uint32_t;
using duration_t = uint32_t;
using coordinate_t = double;
using capacity_t = int64_t;
using skill_t = uint32_t;

// Type helpers.
using coords_t = std::array<coordinate_t, 2>;
using optional_coords_t = boost::optional<coords_t>;
using skills_t = std::unordered_set<skill_t>;
using raw_solution = std::vector<std::list<index_t>>;

// Setting max value would cause trouble with further additions.
constexpr cost_t INFINITE_COST = 3 * (std::numeric_limits<cost_t>::max() / 4);

// Available location status.
enum class TYPE { START, JOB, END };

#endif
