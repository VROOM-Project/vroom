#ifndef HEURISTICS_H
#define HEURISTICS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"

namespace vroom {
namespace heuristics {

// Implementation of a variant of the Solomon I1 heuristic.
template <class T> T basic(const Input& input, INIT init, double lambda);

// Adjusting the above for situation with heterogeneous fleet.
template <class T>
T dynamic_vehicle_choice(const Input& input, INIT init, double lambda);

// Populate routes with user-defined vehicle steps.
template <class T> T initial_routes(const Input& input);

} // namespace heuristics
} // namespace vroom

#endif
