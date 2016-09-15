#ifndef VERSION_H
#define VERSION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#define MAJOR 1
#define MINOR 1
#define PATCH 0
#define DEV 1
#define RC 0

#include <string>

std::string get_version();

#endif
