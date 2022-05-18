#ifndef SWAP_STAR_UTILS_H
#define SWAP_STAR_UTILS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

// This file implements an adjusted version of the SWAP* operator
// described in https://arxiv.org/abs/2012.10384, extended to support
// additional constraint checks (back-hauls and time windows).

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

const auto SwapChoiceCmp = [](const SwapChoice& lhs, const SwapChoice& rhs) {
  return lhs.gain > rhs.gain;
};

constexpr SwapChoice empty_choice = {0, 0, 0, 0, 0};

template <class Route>
bool valid_choice_for_insertion_ranks(const utils::SolutionState& sol_state,
                                      const Index s_vehicle,
                                      const Route& source,
                                      const Index t_vehicle,
                                      const Route& target,
                                      const SwapChoice& sc) {
  const auto source_job_rank = source.route[sc.s_rank];
  const auto target_job_rank = target.route[sc.t_rank];

  // Weak insertion rank begin in target route is still valid except
  // if we remove precisely the job in target that triggered this
  // value.
  bool valid =
    sol_state.weak_insertion_ranks_begin[t_vehicle][source_job_rank] ==
      sc.t_rank + 1 or
    sol_state.weak_insertion_ranks_begin[t_vehicle][source_job_rank] <=
      sc.insertion_in_target;

  // Weak insertion rank end in target route is still valid except if
  // we remove precisely the job in target that triggered this value.
  valid =
    valid && (sol_state.weak_insertion_ranks_end[t_vehicle][source_job_rank] ==
                sc.t_rank + 1 or
              sc.insertion_in_target <
                sol_state.weak_insertion_ranks_end[t_vehicle][source_job_rank]);

  // Weak insertion rank begin in source route is still valid except
  // if we remove precisely the job in source that triggered this
  // value.
  valid = valid &&
          (sol_state.weak_insertion_ranks_begin[s_vehicle][target_job_rank] ==
             sc.s_rank + 1 or
           sol_state.weak_insertion_ranks_begin[s_vehicle][target_job_rank] <=
             sc.insertion_in_source);

  // Weak insertion rank end in source route is still valid except if
  // we remove precisely the job in source that triggered this value.
  valid =
    valid && (sol_state.weak_insertion_ranks_end[s_vehicle][target_job_rank] ==
                sc.s_rank + 1 or
              sc.insertion_in_source <
                sol_state.weak_insertion_ranks_end[s_vehicle][target_job_rank]);

  // If t_rank is greater or equal to insertion_in_target, then strong
  // insertion rank end is still valid when removing in target.
  valid =
    valid && (sc.t_rank < sc.insertion_in_target or
              sc.insertion_in_target <
                sol_state.insertion_ranks_end[t_vehicle][source_job_rank]);

  // If s_rank is greater or equal to insertion_in_source, then strong
  // insertion rank end is still valid when removing in source.
  valid =
    valid && (sc.s_rank < sc.insertion_in_source or
              sc.insertion_in_source <
                sol_state.insertion_ranks_end[s_vehicle][target_job_rank]);

  // If t_rank is strictly lower than insertion_in_target, then strong
  // insertion rank begin is still valid when removing in target.
  valid =
    valid && (sc.t_rank >= sc.insertion_in_target or
              sol_state.insertion_ranks_begin[t_vehicle][source_job_rank] <=
                sc.insertion_in_target);

  // If s_rank is strictly lower than insertion_in_source, then strong
  // insertion rank begin is still valid when removing in source.
  valid =
    valid && (sc.s_rank >= sc.insertion_in_source or
              sol_state.insertion_ranks_begin[s_vehicle][target_job_rank] <=
                sc.insertion_in_source);

  return valid;
}

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
                                         const Index s_vehicle,
                                         const Route& source,
                                         const Index t_vehicle,
                                         const Route& target,
                                         const Gain best_known_gain) {
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

  const auto& v_source = input.vehicles[source.vehicle_rank];
  const auto& v_target = input.vehicles[target.vehicle_rank];

  const auto& s_delivery_margin = source.delivery_margin();
  const auto& s_pickup_margin = source.pickup_margin();
  const auto& t_delivery_margin = target.delivery_margin();
  const auto& t_pickup_margin = target.pickup_margin();

  for (const auto& s_element : top_insertions_in_target) {
    const auto s_rank = s_element.first;
    const auto& target_insertions = s_element.second;

    // sol_state.node_gains contains the Delta value we're looking for
    // except in the case of a single-step route with a start and end,
    // where the start->end cost is not accounted for.
    const auto source_start_end_cost =
      (source.size() == 1 and v_source.has_start() and v_source.has_end())
        ? v_source.cost(v_source.start.value().index(),
                        v_source.end.value().index())
        : 0;
    const auto source_delta =
      sol_state.node_gains[source.vehicle_rank][s_rank] - source_start_end_cost;

    for (const auto& t_element : top_insertions_in_source) {
      const auto t_rank = t_element.first;
      const auto& source_insertions = t_element.second;

      // Same as above.
      const auto target_start_end_cost =
        (target.size() == 1 and v_target.has_start() and v_target.has_end())
          ? v_target.cost(v_target.start.value().index(),
                          v_target.end.value().index())
          : 0;
      const auto target_delta =
        sol_state.node_gains[target.vehicle_rank][t_rank] -
        target_start_end_cost;

      const auto target_in_place_delta =
        utils::in_place_delta_cost(input,
                                   source.route[s_rank],
                                   v_target,
                                   target.route,
                                   t_rank);

      const auto source_in_place_delta =
        utils::in_place_delta_cost(input,
                                   target.route[t_rank],
                                   v_source,
                                   source.route,
                                   s_rank);

      std::vector<SwapChoice> swap_choice_options;

      // Options for in-place insertion in source route include
      // in-place insertion in target route and other relevant
      // positions from target_insertions.
      const Gain in_place_source_insertion_gain =
        target_delta - source_in_place_delta;
      const Gain in_place_target_insertion_gain =
        source_delta - target_in_place_delta;

      Gain current_gain =
        in_place_target_insertion_gain + in_place_source_insertion_gain;
      if (current_gain > best_gain) {
        SwapChoice sc({current_gain, s_rank, t_rank, s_rank, t_rank});
        if (valid_choice_for_insertion_ranks(sol_state,
                                             s_vehicle,
                                             source,
                                             t_vehicle,
                                             target,
                                             sc)) {
          swap_choice_options.push_back(std::move(sc));
        }
      }

      for (const auto& ti : target_insertions) {
        if ((ti.rank != t_rank) and (ti.rank != t_rank + 1) and
            (ti.cost != std::numeric_limits<Gain>::max())) {
          const Gain t_gain = source_delta - ti.cost;
          current_gain = in_place_source_insertion_gain + t_gain;
          if (current_gain > best_gain) {
            SwapChoice sc({current_gain, s_rank, t_rank, s_rank, ti.rank});
            if (valid_choice_for_insertion_ranks(sol_state,
                                                 s_vehicle,
                                                 source,
                                                 t_vehicle,
                                                 target,
                                                 sc)) {
              swap_choice_options.push_back(std::move(sc));
            }
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
          const Gain s_gain = target_delta - si.cost;

          current_gain = s_gain + in_place_target_insertion_gain;
          if (current_gain > best_gain) {
            SwapChoice sc({current_gain, s_rank, t_rank, si.rank, t_rank});
            if (valid_choice_for_insertion_ranks(sol_state,
                                                 s_vehicle,
                                                 source,
                                                 t_vehicle,
                                                 target,
                                                 sc)) {
              swap_choice_options.push_back(std::move(sc));
            }
          }

          for (const auto& ti : target_insertions) {
            if ((ti.rank != t_rank) and (ti.rank != t_rank + 1) and
                (ti.cost != std::numeric_limits<Gain>::max())) {
              const Gain t_gain = source_delta - ti.cost;
              current_gain = s_gain + t_gain;
              if (current_gain > best_gain) {
                SwapChoice sc({current_gain, s_rank, t_rank, si.rank, ti.rank});
                if (valid_choice_for_insertion_ranks(sol_state,
                                                     s_vehicle,
                                                     source,
                                                     t_vehicle,
                                                     target,
                                                     sc)) {
                  swap_choice_options.push_back(std::move(sc));
                }
              }
            }
          }
        }
      }

      std::sort(swap_choice_options.begin(),
                swap_choice_options.end(),
                SwapChoiceCmp);

      assert(swap_choice_options.size() <= 16);

      for (const auto& sc : swap_choice_options) {
        // Browse interesting options by decreasing gain and check for
        // validity.

        // Early abort on invalid capacity bounds.
        const auto s_index = source.route[sc.s_rank];
        const auto& s_delivery = input.jobs[s_index].delivery;
        const auto& s_pickup = input.jobs[s_index].pickup;
        const auto t_index = target.route[sc.t_rank];
        const auto& t_delivery = input.jobs[t_index].delivery;
        const auto& t_pickup = input.jobs[t_index].pickup;

        if (!(t_delivery <= s_delivery_margin + s_delivery) or
            !(t_pickup <= s_pickup_margin + s_pickup) or
            !(s_delivery <= t_delivery_margin + t_delivery) or
            !(s_pickup <= t_pickup_margin + t_pickup)) {
          continue;
        }

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
            // Options are ordered by decreasing gain so we stop at
            // the first valid one.
            break;
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
