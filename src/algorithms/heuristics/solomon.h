#ifndef SOLOMON_H
#define SOLOMON_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"

// Implementation of a variant of the Solomon I1 heuristic.
template <class T>
T basic_heuristic(const Input& input, INIT init, float lambda);

// Adjusting the above for situation with heterogeneous fleet.
template <class T>
T dynamic_vehicle_choice_heuristic(const Input& input, INIT init, float lambda);

#endif
