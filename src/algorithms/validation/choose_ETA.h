#ifndef CHOOSE_ETA_H
#define CHOOSE_ETA_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/solution.h"

namespace vroom {
namespace validation {

Route choose_ETA(const Input& input,
                 unsigned vehicle_rank,
                 const std::vector<VehicleStep>& steps);

} // namespace validation
} // namespace vroom

#endif
