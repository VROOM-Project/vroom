#ifndef MUNKRES_H
#define MUNKRES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>
#include <limits>
#include <list>
#include <set>
#include <unordered_map>

#include "../structures/abstract/edge.h"
#include "../structures/abstract/matrix.h"

template <class T>
std::unordered_map<index_t, index_t>
minimum_weight_perfect_matching(const matrix<T>& m);

template <class T>
std::unordered_map<index_t, index_t>
greedy_symmetric_approx_mwpm(const matrix<T>& m);

#endif
