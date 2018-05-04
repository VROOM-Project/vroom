/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "exchange.h"
#include "local_search.h"
#include "relocate.h"

cvrp_local_search::cvrp_local_search(const input& input, raw_solution& sol)
  : _input(input),
    _sol(sol),
    _amounts(sol.size(), amount_t(input.amount_size())) {
  for (std::size_t i = 0; i < sol.size(); ++i) {
    for (const auto rank : sol[i]) {
      _amounts[i] += _input._jobs[rank].amount;
    }
  }
}

void cvrp_local_search::run() {
  std::cout << "Running CVRP local search." << std::endl;

  // Relocate stuff
  for (unsigned s_v = 0; s_v < _sol.size(); ++s_v) {
    for (unsigned s_rank = 0; s_rank < _sol[s_v].size(); ++s_rank) {
      for (unsigned t_v = 0; t_v < _sol.size(); ++t_v) {
        if (s_v == t_v) {
          continue;
        }
        for (unsigned t_rank = 0; t_rank <= _sol[t_v].size(); ++t_rank) {
          relocate r(_input, _sol, _amounts, s_v, s_rank, t_v, t_rank);
          if (r.gain > 0 and r.is_valid()) {
            r.log();
          }
        }
      }
    }
  }

  // Exchange stuff
  for (unsigned s_v = 0; s_v < _sol.size(); ++s_v) {
    for (unsigned s_rank = 0; s_rank < _sol[s_v].size(); ++s_rank) {
      for (unsigned t_v = s_v + 1; t_v < _sol.size(); ++t_v) {
        for (unsigned t_rank = 0; t_rank < _sol[t_v].size(); ++t_rank) {
          exchange r(_input, _sol, _amounts, s_v, s_rank, t_v, t_rank);
          if (r.gain > 0 and r.is_valid()) {
            r.log();
          }
        }
      }
    }
  }
}
