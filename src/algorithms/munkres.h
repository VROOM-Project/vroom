#ifndef MUNKRES_H
#define MUNKRES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_map>
#include <limits>
#include <set>
#include <list>
#include <cassert>
#include "../structures/abstract/matrix.h"
#include "../structures/abstract/edge.h"

template <class T>
std::unordered_map<index_t, index_t> minimum_weight_perfect_matching(const matrix<T>& m);

template <class T>
std::unordered_map<index_t, index_t> greedy_symmetric_approx_mwpm(const matrix<T>& m);

#endif
