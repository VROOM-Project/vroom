#ifndef SWAP_STAR_UTILS_H
#define SWAP_STAR_UTILS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <set>

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
  Index s_rank;
  Index t_rank;
  Index insertion_in_source;
  Index insertion_in_target;
};

auto SwapChoiceCmp = [](const SwapChoice& lhs, const SwapChoice& rhs) {
  return lhs.gain > rhs.gain;
};

constexpr SwapChoice empty_choice = {0, 0, 0, 0, 0};

struct InsertionRange {
  std::vector<Index> range;
  Index first_rank;
  Index last_rank;
};

// Compute insertion range in source route when removing job at s_rank
// and adding job at job_rank in source route at insertion_rank.
inline InsertionRange get_insert_range(const std::vector<Index>& s_route,
                                       Index s_rank,
                                       Index job_rank,
                                       Index insertion_rank) {
  InsertionRange insert;
  if (s_rank == insertion_rank) {
    insert.range.push_back(job_rank);
    insert.first_rank = s_rank;
    insert.last_rank = s_rank + 1;
  } else {
    if (s_rank < insertion_rank) {
      std::copy(s_route.begin() + s_rank + 1,
                s_route.begin() + insertion_rank,
                std::back_inserter(insert.range));
      insert.range.push_back(job_rank);
      insert.first_rank = s_rank;
      insert.last_rank = insertion_rank;
    } else {
      insert.range.push_back(job_rank);
      std::copy(s_route.begin() + insertion_rank,
                s_route.begin() + s_rank,
                std::back_inserter(insert.range));
      insert.first_rank = insertion_rank;
      insert.last_rank = s_rank + 1;
    }
  }

  return insert;
}

template <class Route>
SwapChoice compute_best_swap_star_choice(const Input& input,
                                         const utils::SolutionState& sol_state,
                                         const Route& source,
                                         const Route& target,
                                         Gain best_known_gain) {
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
  auto best_choice = empty_choice;
  Gain best_gain = best_known_gain;

  for (const auto& s_element : top_insertions_in_target) {
    const auto s_rank = s_element.first;
    const auto& target_insertions = s_element.second;

    const auto source_remove_gain =
      sol_state.node_gains[source.vehicle_rank][s_rank];

    for (const auto& t_element : top_insertions_in_source) {
      const auto t_rank = t_element.first;
      const auto& source_insertions = t_element.second;

      const auto target_remove_gain =
        sol_state.node_gains[target.vehicle_rank][t_rank];

      const auto target_in_place_delta =
        utils::in_place_delta_cost(input,
                                   source.route[s_rank],
                                   input.vehicles[target.vehicle_rank],
                                   target.route,
                                   t_rank);

      const auto source_in_place_delta =
        utils::in_place_delta_cost(input,
                                   target.route[t_rank],
                                   input.vehicles[source.vehicle_rank],
                                   source.route,
                                   s_rank);

      auto swap_choice_options =
        std::set<SwapChoice, decltype(SwapChoiceCmp)>(SwapChoiceCmp);

      // Options for in-place insertion in source route include
      // in-place insertion in target route and other relevant
      // positions from target_insertions.
      const Gain in_place_source_insertion_gain =
        target_remove_gain - source_in_place_delta;
      const Gain in_place_target_insertion_gain =
        source_remove_gain - target_in_place_delta;

      Gain current_gain =
        in_place_target_insertion_gain + in_place_source_insertion_gain;
      if (current_gain > best_gain) {
        swap_choice_options.insert(
          {current_gain, s_rank, t_rank, s_rank, t_rank});
      }

      for (const auto& ti : target_insertions) {
        if ((ti.rank != t_rank) and (ti.rank != t_rank + 1) and
            (ti.cost != std::numeric_limits<Gain>::max())) {
          const Gain t_gain = source_remove_gain - ti.cost;
          current_gain = in_place_source_insertion_gain + t_gain;
          if (current_gain > best_gain) {
            swap_choice_options.insert(
              {current_gain, s_rank, t_rank, s_rank, ti.rank});
          }
        }
      }

      // Options for other relevant positions for insertion in source
      // route (from source_insertions) include in-place insertion in
      // target route and other relevant positions from
      // target_insertions.
      for (const auto& si : source_insertions) {
        if ((si.rank != s_rank) and (si.rank != s_rank + 1) and
            (si.cost != std::numeric_limits<Gain>::max())) {
          const Gain s_gain = target_remove_gain - si.cost;

          current_gain = s_gain + in_place_target_insertion_gain;
          if (current_gain > best_gain) {
            swap_choice_options.insert(
              {current_gain, s_rank, t_rank, si.rank, t_rank});
          }

          for (const auto& ti : target_insertions) {
            if ((ti.rank != t_rank) and (ti.rank != t_rank + 1) and
                (ti.cost != std::numeric_limits<Gain>::max())) {
              const Gain t_gain = source_remove_gain - ti.cost;
              current_gain = s_gain + t_gain;
              if (current_gain > best_gain) {
                swap_choice_options.insert(
                  {current_gain, s_rank, t_rank, si.rank, ti.rank});
              }
            }
          }
        }
      }

      assert(swap_choice_options.size() <= 16);

      const auto& sc = *(swap_choice_options.begin());

      if (best_gain < sc.gain) {
        const auto s_insert = get_insert_range(source.route,
                                               s_rank,
                                               target.route[t_rank],
                                               sc.insertion_in_source);

        auto source_pickup =
          std::accumulate(s_insert.range.begin(),
                          s_insert.range.end(),
                          input.zero_amount(),
                          [&](auto sum, const auto i) {
                            return sum + input.jobs[i].pickup;
                          });
        auto source_delivery =
          std::accumulate(s_insert.range.begin(),
                          s_insert.range.end(),
                          input.zero_amount(),
                          [&](auto sum, const auto i) {
                            return sum + input.jobs[i].delivery;
                          });

        bool source_valid =
          source.is_valid_addition_for_capacity_margins(input,
                                                        source_pickup,
                                                        source_delivery,
                                                        s_insert.first_rank,
                                                        s_insert.last_rank);

        source_valid =
          source_valid &&
          source
            .is_valid_addition_for_capacity_inclusion(input,
                                                      source_delivery,
                                                      s_insert.range.begin(),
                                                      s_insert.range.end(),
                                                      s_insert.first_rank,
                                                      s_insert.last_rank);

        source_valid = source_valid &&
                       source.is_valid_addition_for_tw(input,
                                                       s_insert.range.begin(),
                                                       s_insert.range.end(),
                                                       s_insert.first_rank,
                                                       s_insert.last_rank);

        if (source_valid) {
          const auto t_insert = get_insert_range(target.route,
                                                 t_rank,
                                                 source.route[s_rank],
                                                 sc.insertion_in_target);

          auto target_pickup =
            std::accumulate(t_insert.range.begin(),
                            t_insert.range.end(),
                            input.zero_amount(),
                            [&](auto sum, const auto i) {
                              return sum + input.jobs[i].pickup;
                            });
          auto target_delivery =
            std::accumulate(t_insert.range.begin(),
                            t_insert.range.end(),
                            input.zero_amount(),
                            [&](auto sum, const auto i) {
                              return sum + input.jobs[i].delivery;
                            });

          bool target_valid =
            target.is_valid_addition_for_capacity_margins(input,
                                                          target_pickup,
                                                          target_delivery,
                                                          t_insert.first_rank,
                                                          t_insert.last_rank);

          target_valid =
            target_valid &&
            target
              .is_valid_addition_for_capacity_inclusion(input,
                                                        target_delivery,
                                                        t_insert.range.begin(),
                                                        t_insert.range.end(),
                                                        t_insert.first_rank,
                                                        t_insert.last_rank);

          target_valid = target_valid &&
                         target.is_valid_addition_for_tw(input,
                                                         t_insert.range.begin(),
                                                         t_insert.range.end(),
                                                         t_insert.first_rank,
                                                         t_insert.last_rank);

          if (target_valid) {
            best_gain = sc.gain;
            best_choice = sc;
          }
        }
      }
    }
  }

  return best_choice;
}

} // namespace ls
} // namespace vroom

#endif
