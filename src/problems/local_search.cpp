/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>

#include "problems/local_search.h"

local_search::local_search(const input& input, unsigned max_nb_jobs_removal)
  : _input(input),
    _m(_input.get_matrix()),
    V(_input._vehicles.size()),
    _amount_lower_bound(_input.get_amount_lower_bound()),
    _double_amount_lower_bound(_amount_lower_bound + _amount_lower_bound),
    _max_nb_jobs_removal(max_nb_jobs_removal),
    _all_routes(V),
    _sol_state(input) {
  // Initialize all route indices.
  std::iota(_all_routes.begin(), _all_routes.end(), 0);
}

local_search::~local_search() {
}
