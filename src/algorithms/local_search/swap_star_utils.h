#ifndef SWAP_STAR_UTILS_H
#define SWAP_STAR_UTILS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

namespace vroom {
namespace ls {

struct InsertionOption {
  Gain cost;
  Index rank;
};

using ThreeInsertions = std::array<InsertionOption, 3>;

constexpr InsertionOption no_insert = {std::numeric_limits<Gain>::max(), 0};
constexpr ThreeInsertions
  empty_three_insertions({no_insert, no_insert, no_insert});

template <class Route>
ThreeInsertions find_top_3_insertions(const Input& input,
                                      Index j,
                                      const Route& r) {
  const auto& v = input.vehicles[r.vehicle_rank];

  auto best_insertions = empty_three_insertions;

  for (Index rank = 0; rank <= r.route.size(); ++rank) {
    const InsertionOption current_insert =
      {utils::addition_cost(input, j, v, r.route, rank), rank};

    if (current_insert.cost < best_insertions[2].cost) {
      if (current_insert.cost < best_insertions[1].cost) {
        if (current_insert.cost < best_insertions[0].cost) {
          best_insertions[2] = best_insertions[1];
          best_insertions[1] = best_insertions[0];
          best_insertions[0] = current_insert;
        } else {
          best_insertions[2] = best_insertions[1];
          best_insertions[1] = current_insert;
        }
      } else {
        best_insertions[2] = current_insert;
      }
    }
  }

  return best_insertions;
}

struct SwapChoice {
  Gain gain;
  Index source_rank;
  Index target_rank;
  Index insertion_in_source;
  Index insertion_in_target;
};

constexpr SwapChoice empty_choice = {0, 0, 0, 0, 0};

template <class Route>
SwapChoice compute_best_swap_star_choice(const Input& input,
                                         const utils::SolutionState& sol_state,
                                         const Route& source,
                                         const Route& target) {
  // Preprocessing phase.
  std::unordered_map<Index, ThreeInsertions> top_insertions_in_target;
  for (unsigned s_rank = 0; s_rank < source.route.size(); ++s_rank) {
    const auto source_job_rank = source.route[s_rank];

    if (input.jobs[source_job_rank].type == JOB_TYPE::SINGLE and
        input.vehicle_ok_with_job(target.vehicle_rank, source_job_rank)) {
      top_insertions_in_target[s_rank] =
        find_top_3_insertions(input, source_job_rank, target);
    }
  }

  std::unordered_map<Index, ThreeInsertions> top_insertions_in_source;
  for (unsigned t_rank = 0; t_rank < target.route.size(); ++t_rank) {
    const auto target_job_rank = target.route[t_rank];

    if (input.jobs[target_job_rank].type == JOB_TYPE::SINGLE and
        input.vehicle_ok_with_job(source.vehicle_rank, target_job_rank)) {
      top_insertions_in_source[t_rank] =
        find_top_3_insertions(input, target_job_rank, source);
    }
  }

  // Search phase.
  auto choice = empty_choice;
  Gain best_delta = 0;

  for (const auto& [s_rank, target_insertions] : top_insertions_in_target) {
    const auto source_remove_gain =
      sol_state.node_gains[source.vehicle_rank][s_rank];

    for (const auto& [t_rank, source_insertions] : top_insertions_in_source) {
      const auto target_remove_gain =
        sol_state.node_gains[target.vehicle_rank][t_rank];

      // Compute delta for target route.
      const auto target_in_place_delta =
        utils::in_place_delta_cost(input,
                                   source.route[s_rank],
                                   input.vehicles[target.vehicle_rank],
                                   target.route,
                                   t_rank);

      auto target_delta = target_in_place_delta;

      bool use_k_for_min = false;
      const auto k =
        std::find_if(target_insertions.begin(),
                     target_insertions.end(),
                     [&](const auto& option) {
                       return (option.rank != t_rank) and
                              (option.rank != t_rank + 1) and
                              (option.cost != std::numeric_limits<Gain>::max());
                     });

      if (k != target_insertions.end() and k->cost < target_in_place_delta) {
        target_delta = k->cost;
        use_k_for_min = true;
      }

      target_delta -= source_remove_gain;

      // Compute delta for source route.
      const auto source_in_place_delta =
        utils::in_place_delta_cost(input,
                                   target.route[t_rank],
                                   input.vehicles[source.vehicle_rank],
                                   source.route,
                                   s_rank);

      auto source_delta = source_in_place_delta;

      bool use_k_prime_for_min = false;
      const auto k_prime =
        std::find_if(source_insertions.begin(),
                     source_insertions.end(),
                     [&](const auto& option) {
                       return (option.rank != s_rank) and
                              (option.rank != s_rank + 1) and
                              (option.cost != std::numeric_limits<Gain>::max());
                     });

      if (k_prime != source_insertions.end() and
          k_prime->cost < source_in_place_delta) {
        source_delta = k_prime->cost;
        use_k_prime_for_min = true;
      }

      source_delta -= target_remove_gain;

      const auto current_delta = source_delta + target_delta;

      if (current_delta < best_delta) {
        best_delta = current_delta;
        choice = {-current_delta,
                  s_rank,
                  t_rank,
                  (use_k_prime_for_min) ? k_prime->rank : s_rank,
                  (use_k_for_min) ? k->rank : t_rank};
      }
    }
  }

  return choice;
}

} // namespace ls
} // namespace vroom

#endif
