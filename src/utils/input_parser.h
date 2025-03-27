#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/input/vehicle_step.h"

namespace vroom::io {

void parse(Input& input, const std::string& input_str, bool geometry);

} // namespace vroom::io

#endif
