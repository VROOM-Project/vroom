/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/local_search.h"

local_search::local_search(const input& input, const raw_solution& sol)
  : _input(input),
    _m(_input.get_matrix()),
    V(_input._vehicles.size()),
    _amount_lower_bound(_input.get_amount_lower_bound()),
    _double_amount_lower_bound(_amount_lower_bound + _amount_lower_bound),
    _sol(sol),
    _sol_state(input, sol) {
}

local_search::~local_search() {
}
