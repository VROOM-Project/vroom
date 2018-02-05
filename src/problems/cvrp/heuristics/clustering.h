#ifndef CLUSTERING_H
#define CLUSTERING_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <vector>

#include "../../../structures/vroom/amount.h"
#include "../../../structures/vroom/input/input.h"

std::vector<std::vector<index_t>> clustering(const input& input);

#endif
