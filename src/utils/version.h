#ifndef VERSION_H
#define VERSION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#define MAJOR 1
#define MINOR 1
#define PATCH 0
#define DEV 0
#define RC 1

#include <string>

std::string get_version();

#endif
