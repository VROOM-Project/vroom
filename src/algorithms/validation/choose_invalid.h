#ifndef CHOOSE_INVALID_H
#define CHOOSE_INVALID_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/solution.h"

namespace vroom {
namespace validation {

Route choose_invalid_route(const Input& input,
                           unsigned vehicle_rank,
                           const std::vector<InputStep>& steps,
                           std::unordered_set<Index>& unassigned_ranks);

} // namespace validation
} // namespace vroom

#endif
