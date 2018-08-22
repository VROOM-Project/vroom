#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "utils/exceptions.h"

inline cost_t add_without_overflow(cost_t a, cost_t b) {
  if (a > std::numeric_limits<cost_t>::max() - b) {
    throw custom_exception(
      "Too high cost values, stopping to avoid overflowing.");
  }
  return a + b;
}

// Compute cost of adding job with index job_index in given route at
// given rank for vehicle v.
inline gain_t addition_cost(const input& input,
                            const matrix<cost_t>& m,
                            index_t job_rank,
                            const vehicle_t& v,
                            const std::vector<index_t>& route,
                            index_t rank) {
  index_t job_index = input._jobs[job_rank].index();
  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t old_edge_cost = 0;

  if (rank == route.size()) {
    if (route.size() == 0) {
      // Adding job to an empty route.
      if (v.has_start()) {
        previous_cost = m[v.start.get().index()][job_index];
      }
      if (v.has_end()) {
        next_cost = m[job_index][v.end.get().index()];
      }
    } else {
      // Adding job past the end after a real job.
      auto p_index = input._jobs[route[rank - 1]].index();
      previous_cost = m[p_index][job_index];
      if (v.has_end()) {
        auto n_index = v.end.get().index();
        old_edge_cost = m[p_index][n_index];
        next_cost = m[job_index][n_index];
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = input._jobs[route[rank]].index();
    next_cost = m[job_index][n_index];

    if (rank == 0) {
      if (v.has_start()) {
        auto p_index = v.start.get().index();
        previous_cost = m[p_index][job_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index = input._jobs[route[rank - 1]].index();
      previous_cost = m[p_index][job_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  return previous_cost + next_cost - old_edge_cost;
}

#endif
