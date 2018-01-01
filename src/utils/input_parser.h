#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>
#include <vector>

#include "../../include/rapidjson/document.h"
#include "../../include/rapidjson/error/en.h"
#include "../structures/abstract/matrix.h"
#include "../structures/typedefs.h"
#include "../structures/vroom/input/input.h"
#include "../structures/vroom/job.h"
#include "../structures/vroom/vehicle.h"
#include "./exceptions.h"

input parse(const cl_args_t& cl_args);

#endif
