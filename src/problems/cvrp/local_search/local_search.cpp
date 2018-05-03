/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "local_search.h"

cvrp_local_search::cvrp_local_search(const input& input, raw_solution& sol)
  : _input(input), _sol(sol) {
}

void cvrp_local_search::run() {
  std::cout << "Running CVRP local search." << std::endl;
}
