/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/reverse_2_opt.h"

cvrp_reverse_two_opt::cvrp_reverse_two_opt(const input& input,
                                           raw_solution& sol,
                                           const solution_state& sol_state,
                                           index_t source_vehicle,
                                           index_t source_rank,
                                           index_t target_vehicle,
                                           index_t target_rank)
  : cvrp_ls_operator(input,
                     sol,
                     sol_state,
                     source_vehicle,
                     source_rank,
                     target_vehicle,
                     target_rank) {
  assert(source_vehicle != target_vehicle);
  assert(_sol[source_vehicle].size() >= 1);
  assert(_sol[target_vehicle].size() >= 1);
  assert(source_rank < _sol[source_vehicle].size());
  assert(target_rank < _sol[target_vehicle].size());
}

void cvrp_reverse_two_opt::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_source = _input._vehicles[source_vehicle];
  const auto& v_target = _input._vehicles[target_vehicle];

  index_t s_index = _input._jobs[_sol[source_vehicle][source_rank]].index();
  index_t t_index = _input._jobs[_sol[target_vehicle][target_rank]].index();
  index_t last_s = _input._jobs[_sol[source_vehicle].back()].index();
  index_t first_t = _input._jobs[_sol[target_vehicle].front()].index();
  stored_gain = 0;
  bool last_in_source = (source_rank == _sol[source_vehicle].size() - 1);
  bool last_in_target = (target_rank == _sol[target_vehicle].size() - 1);

  // Cost of swapping route for vehicle source_vehicle after step
  // source_rank with route for vehicle target_vehicle up to step
  // target_rank, but reversed.

  // Add new source -> target edge.
  stored_gain -= m[s_index][t_index];

  // Cost of reversing target route portion.
  stored_gain += _sol_state.fwd_costs[target_vehicle][target_rank];
  stored_gain -= _sol_state.bwd_costs[target_vehicle][target_rank];

  if (!last_in_target) {
    // Spare next edge in target route.
    index_t next_index =
      _input._jobs[_sol[target_vehicle][target_rank + 1]].index();
    stored_gain += m[t_index][next_index];
  }

  if (!last_in_source) {
    // Spare next edge in source route.
    index_t next_index =
      _input._jobs[_sol[source_vehicle][source_rank + 1]].index();
    stored_gain += m[s_index][next_index];

    // Part of source route is moved to target route.
    index_t next_s_index =
      _input._jobs[_sol[source_vehicle][source_rank + 1]].index();

    // Cost or reverting source route portion.
    stored_gain += _sol_state.fwd_costs[source_vehicle].back();
    stored_gain -= _sol_state.fwd_costs[source_vehicle][source_rank + 1];
    stored_gain -= _sol_state.bwd_costs[source_vehicle].back();
    stored_gain += _sol_state.bwd_costs[source_vehicle][source_rank + 1];

    if (last_in_target) {
      if (v_target.has_end()) {
        // Handle target route new end.
        auto end_t = v_target.end.get().index();
        stored_gain += m[t_index][end_t];
        stored_gain -= m[next_s_index][end_t];
      }
    } else {
      // Add new target -> source edge.
      index_t next_t_index =
        _input._jobs[_sol[target_vehicle][target_rank + 1]].index();
      stored_gain -= m[next_s_index][next_t_index];
    }
  }

  if (v_source.has_end()) {
    // Update cost to source end because last job changed.
    auto end_s = v_source.end.get().index();
    stored_gain += m[last_s][end_s];
    stored_gain -= m[first_t][end_s];
  }

  if (v_target.has_start()) {
    // Spare cost from target start because first job changed.
    auto start_t = v_target.start.get().index();
    stored_gain += m[start_t][first_t];
    if (!last_in_source) {
      stored_gain -= m[start_t][last_s];
    } else {
      // No job from source route actually swapped to target route.
      if (!last_in_target) {
        // Going straight from start to next job in target route.
        index_t next_index =
          _input._jobs[_sol[target_vehicle][target_rank + 1]].index();
        stored_gain -= m[start_t][next_index];
      } else {
        // Emptying the whole target route here, so also gaining cost
        // to end if it exists.
        if (v_target.has_end()) {
          auto end_t = v_target.end.get().index();
          stored_gain += m[t_index][end_t];
        }
      }
    }
  }

  gain_computed = true;
}

bool cvrp_reverse_two_opt::is_valid() const {
  bool valid = (_sol_state.bwd_skill_rank[source_vehicle][target_vehicle] <=
                source_rank + 1);

  valid &=
    (target_rank < _sol_state.fwd_skill_rank[target_vehicle][source_vehicle]);

  valid &= (_sol_state.fwd_amounts[source_vehicle][source_rank] +
              _sol_state.fwd_amounts[target_vehicle][target_rank] <=
            _input._vehicles[source_vehicle].capacity);
  valid &= (_sol_state.bwd_amounts[target_vehicle][target_rank] +
              _sol_state.bwd_amounts[source_vehicle][source_rank] <=
            _input._vehicles[target_vehicle].capacity);

  return valid;
}

void cvrp_reverse_two_opt::apply() const {
  auto nb_source = _sol[source_vehicle].size() - 1 - source_rank;

  _sol[target_vehicle].insert(_sol[target_vehicle].begin(),
                              _sol[source_vehicle].rbegin(),
                              _sol[source_vehicle].rbegin() + nb_source);
  _sol[source_vehicle].erase(_sol[source_vehicle].begin() + source_rank + 1,
                             _sol[source_vehicle].end());
  _sol[source_vehicle].insert(_sol[source_vehicle].end(),
                              _sol[target_vehicle].rend() - target_rank -
                                nb_source - 1,
                              _sol[target_vehicle].rend() - nb_source);
  _sol[target_vehicle].erase(_sol[target_vehicle].begin() + nb_source,
                             _sol[target_vehicle].begin() + nb_source +
                               target_rank + 1);
}

std::vector<index_t> cvrp_reverse_two_opt::addition_candidates() const {
  return {source_vehicle, target_vehicle};
}
