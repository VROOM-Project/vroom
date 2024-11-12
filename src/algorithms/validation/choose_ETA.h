#ifndef CHOOSE_ETA_H
#define CHOOSE_ETA_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/route.h"

namespace vroom::validation {

Route choose_ETA(const Input& input,
                 unsigned vehicle_rank,
                 const std::vector<VehicleStep>& steps);

} // namespace vroom::validation

#endif
