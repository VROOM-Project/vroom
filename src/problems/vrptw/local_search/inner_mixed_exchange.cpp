/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/inner_mixed_exchange.h"

vrptw_inner_mixed_exchange::vrptw_inner_mixed_exchange(
  const input& input,
  const solution_state& sol_state,
  tw_solution& tw_sol,
  index_t s_vehicle,
  index_t s_rank,
  index_t t_rank)
  : cvrp_inner_mixed_exchange(input,
                              sol_state,
                              tw_sol[s_vehicle].route,
                              s_vehicle,
                              s_rank,
                              t_rank),
    _tw_sol(tw_sol),
    _s_is_normal_valid(false),
    _s_is_reverse_valid(false) {
}

void vrptw_inner_mixed_exchange::compute_gain() {
  cvrp_inner_mixed_exchange::compute_gain();
  assert(_s_is_normal_valid or _s_is_reverse_valid);

  gain_t s_gain;
  if (reverse_t_edge) {
    s_gain = reversed_s_gain;
    if (!_s_is_reverse_valid) {
      // Biggest potential gain is obtained when reversing edge, but
      // this does not match TW constraints, so update gain and edge
      // direction to not reverse.
      s_gain = normal_s_gain;
      reverse_t_edge = false;
    }
  } else {
    s_gain = normal_s_gain;
    if (!_s_is_normal_valid) {
      // Biggest potential gain is obtained when not reversing edge,
      // but this does not match TW constraints, so update gain and
      // edge direction to reverse.
      s_gain = reversed_s_gain;
      reverse_t_edge = true;
    }
  }

  stored_gain = s_gain + t_gain;
}

bool vrptw_inner_mixed_exchange::is_valid() {
  unsigned validity_test_size =
    (s_rank < t_rank) ? t_rank - s_rank + 2 : s_rank - t_rank + 1;

  std::vector<index_t> job_ranks(validity_test_size);
  index_t first_rank;
  index_t last_rank;
  index_t t_edge_first;
  index_t t_edge_last;
  index_t s_node;

  if (t_rank < s_rank) {
    s_node = 0;
    t_edge_first = job_ranks.size() - 2;
    t_edge_last = job_ranks.size() - 1;

    std::copy(s_route.begin() + t_rank + 2,
              s_route.begin() + s_rank,
              job_ranks.begin() + 1);

    first_rank = t_rank;
    last_rank = s_rank + 1;
  } else {
    t_edge_first = 0;
    t_edge_last = 1;
    s_node = job_ranks.size() - 1;

    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank,
              job_ranks.begin() + 2);

    first_rank = s_rank;
    last_rank = t_rank + 2;
  }

  job_ranks[s_node] = s_route[s_rank];
  job_ranks[t_edge_first] = s_route[t_rank];
  job_ranks[t_edge_last] = s_route[t_rank + 1];

  _s_is_normal_valid =
    _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                job_ranks.begin(),
                                                job_ranks.end(),
                                                first_rank,
                                                last_rank);

  std::swap(job_ranks[t_edge_first], job_ranks[t_edge_last]);
  _s_is_reverse_valid =
    _tw_sol[s_vehicle].is_valid_addition_for_tw(_input,
                                                job_ranks.begin(),
                                                job_ranks.end(),
                                                first_rank,
                                                last_rank);

  return _s_is_normal_valid or _s_is_reverse_valid;
}

void vrptw_inner_mixed_exchange::apply() {
  unsigned validity_test_size =
    (s_rank < t_rank) ? t_rank - s_rank + 2 : s_rank - t_rank + 1;

  std::vector<index_t> job_ranks(validity_test_size);
  index_t first_rank;
  index_t last_rank;
  index_t t_edge_first;
  index_t t_edge_last;
  index_t s_node;

  if (t_rank < s_rank) {
    s_node = 0;
    t_edge_first = job_ranks.size() - 2;
    t_edge_last = job_ranks.size() - 1;

    std::copy(s_route.begin() + t_rank + 2,
              s_route.begin() + s_rank,
              job_ranks.begin() + 1);

    first_rank = t_rank;
    last_rank = s_rank + 1;
  } else {
    t_edge_first = 0;
    t_edge_last = 1;
    s_node = job_ranks.size() - 1;

    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank,
              job_ranks.begin() + 2);

    first_rank = s_rank;
    last_rank = t_rank + 2;
  }

  job_ranks[s_node] = s_route[s_rank];
  job_ranks[t_edge_first] = s_route[t_rank];
  job_ranks[t_edge_last] = s_route[t_rank + 1];

  if (reverse_t_edge) {
    std::swap(job_ranks[t_edge_first], job_ranks[t_edge_last]);
  }

  _tw_sol[s_vehicle].replace(_input,
                             job_ranks.begin(),
                             job_ranks.end(),
                             first_rank,
                             last_rank);
}
