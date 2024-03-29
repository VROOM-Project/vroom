#ifndef CHECK_H
#define CHECK_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/solution.h"

namespace vroom::validation {

Solution check_and_set_ETA(const Input& input, unsigned nb_thread);

} // namespace vroom::validation

#endif
