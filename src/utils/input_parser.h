#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/input/vehicle_step.h"

namespace vroom {
namespace io {

struct CLArgs;

Input parse(std::string input, Servers& servers, ROUTER router, bool geometry);

} // namespace io
} // namespace vroom

#endif
