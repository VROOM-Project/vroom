/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/inner_or_opt.h"

vrptw_inner_or_opt::vrptw_inner_or_opt(const input& input,
                                       const solution_state& sol_state,
                                       tw_solution& tw_sol,
                                       index_t s_vehicle,
                                       index_t s_rank,
                                       index_t t_rank)
  : cvrp_inner_or_opt(input,
                      sol_state,
                      tw_sol[s_vehicle].route,
                      s_vehicle,
                      s_rank,
                      t_rank),
    _tw_sol(tw_sol),
    _is_normal_valid(false),
    _is_reverse_valid(false) {
}

void vrptw_inner_or_opt::compute_gain() {
  cvrp_inner_or_opt::compute_gain();
  assert(_is_normal_valid or _is_reverse_valid);

  if (reverse_s_edge) {
    if (!_is_reverse_valid) {
      // Biggest potential gain is obtained when reversing edge, but
      // this does not match TW constraints, so update gain and edge
      // direction to not reverse.
      stored_gain = normal_stored_gain;
      reverse_s_edge = false;
    }
  } else {
    if (!_is_normal_valid) {
      // Biggest potential gain is obtained when not reversing edge,
      // but this does not match TW constraints, so update gain and
      // edge direction to reverse.
      stored_gain = reversed_stored_gain;
      reverse_s_edge = true;
    }
  }
}

bool vrptw_inner_or_opt::is_valid() {
  if (!_tw_sol[s_vehicle].is_valid_removal(_input, s_rank, 2)) {
    return false;
  }

  unsigned validity_test_size =
    (s_rank < t_rank) ? t_rank - s_rank + 2 : s_rank - t_rank + 2;

  std::vector<index_t> job_ranks(validity_test_size);
  index_t first_rank;
  index_t last_rank;
  index_t s_edge_first;
  index_t s_edge_last;

  if (t_rank < s_rank) {
    s_edge_first = 0;
    s_edge_last = 1;

    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              job_ranks.begin() + 2);

    first_rank = t_rank;
    last_rank = s_rank + 2;
  } else {
    s_edge_first = job_ranks.size() - 2;
    s_edge_last = job_ranks.size() - 1;

    std::copy(s_route.begin() + s_rank + 2,
              s_route.begin() + t_rank + 2,
              job_ranks.begin());

    first_rank = s_rank;
    last_rank = t_rank + 2;
  }

  job_ranks[s_edge_first] = s_route[s_rank];
  job_ranks[s_edge_last] = s_route[s_rank + 1];

  _is_normal_valid =
    _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                job_ranks.begin(),
                                                job_ranks.end(),
                                                first_rank,
                                                last_rank);

  std::swap(job_ranks[s_edge_first], job_ranks[s_edge_last]);
  _is_reverse_valid =
    _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                job_ranks.begin(),
                                                job_ranks.end(),
                                                first_rank,
                                                last_rank);
  return _is_normal_valid or _is_reverse_valid;
}

void vrptw_inner_or_opt::apply() const {
  std::vector<index_t> job_ranks({s_route[s_rank], s_route[s_rank + 1]});
  if (reverse_s_edge) {
    std::swap(job_ranks[0], job_ranks[1]);
  }

  _tw_sol[s_vehicle].remove(_input, s_rank, 2);

  _tw_sol[s_vehicle].replace(_input,
                             job_ranks.begin(),
                             job_ranks.end(),
                             t_rank,
                             t_rank);
}
