#ifndef VERSION_H
#define VERSION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#define MAJOR 1
#define MINOR 11
#define PATCH 0
#define DEV 0
#define RC 0

#include <string>

namespace vroom {

std::string get_version();

} // namespace vroom

#endif
