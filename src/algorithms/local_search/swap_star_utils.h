#ifndef SWAP_STAR_UTILS_H
#define SWAP_STAR_UTILS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "algorithms/local_search/top_insertions.h"
#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

// This file implements an adjusted version of the SWAP* operator
// described in https://arxiv.org/abs/2012.10384, extended to support
// additional constraint checks (back-hauls and time windows).

namespace vroom::ls {

struct SwapChoice {
  Eval gain;
  Index s_rank{0};
  Index t_rank{0};
  Index insertion_in_source{0};
  Index insertion_in_target{0};
  Amount source_range_delivery{0};
  Amount target_range_delivery{0};

  SwapChoice() = default;

  SwapChoice(const Eval& gain,
             Index s_rank,
             Index t_rank,
             Index insertion_in_source,
             Index insertion_in_target)
    : gain(gain),
      s_rank(s_rank),
      t_rank(t_rank),
      insertion_in_source(insertion_in_source),
      insertion_in_target(insertion_in_target) {
  }
};

const auto SwapChoiceCmp = [](const SwapChoice& lhs, const SwapChoice& rhs) {
  return rhs.gain < lhs.gain;
};

const SwapChoice empty_swap_choice = {Eval(), 0, 0, 0, 0};

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
      sc.t_rank + 1 ||
    sol_state.weak_insertion_ranks_begin[t_vehicle][source_job_rank] <=
      sc.insertion_in_target;

  // Weak insertion rank end in target route is still valid except if
  // we remove precisely the job in target that triggered this value.
  valid =
    valid && (sol_state.weak_insertion_ranks_end[t_vehicle][source_job_rank] ==
                sc.t_rank + 1 ||
              sc.insertion_in_target <
                sol_state.weak_insertion_ranks_end[t_vehicle][source_job_rank]);

  // Weak insertion rank begin in source route is still valid except
  // if we remove precisely the job in source that triggered this
  // value.
  valid = valid &&
          (sol_state.weak_insertion_ranks_begin[s_vehicle][target_job_rank] ==
             sc.s_rank + 1 ||
           sol_state.weak_insertion_ranks_begin[s_vehicle][target_job_rank] <=
             sc.insertion_in_source);

  // Weak insertion rank end in source route is still valid except if
  // we remove precisely the job in source that triggered this value.
  valid =
    valid && (sol_state.weak_insertion_ranks_end[s_vehicle][target_job_rank] ==
                sc.s_rank + 1 ||
              sc.insertion_in_source <
                sol_state.weak_insertion_ranks_end[s_vehicle][target_job_rank]);

  // If t_rank is greater or equal to insertion_in_target, then strong
  // insertion rank end is still valid when removing in target.
  valid =
    valid && (sc.t_rank < sc.insertion_in_target ||
              sc.insertion_in_target <
                sol_state.insertion_ranks_end[t_vehicle][source_job_rank]);

  // If s_rank is greater or equal to insertion_in_source, then strong
  // insertion rank end is still valid when removing in source.
  valid =
    valid && (sc.s_rank < sc.insertion_in_source ||
              sc.insertion_in_source <
                sol_state.insertion_ranks_end[s_vehicle][target_job_rank]);

  // If t_rank is strictly lower than insertion_in_target, then strong
  // insertion rank begin is still valid when removing in target.
  valid =
    valid && (sc.t_rank >= sc.insertion_in_target ||
              sol_state.insertion_ranks_begin[t_vehicle][source_job_rank] <=
                sc.insertion_in_target);

  // If s_rank is strictly lower than insertion_in_source, then strong
  // insertion rank begin is still valid when removing in source.
  valid =
    valid && (sc.s_rank >= sc.insertion_in_source ||
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
      insert.range.reserve(insertion_rank - s_rank);
      std::copy(s_route.begin() + s_rank + 1,
                s_route.begin() + insertion_rank,
                std::back_inserter(insert.range));
      insert.range.push_back(job_rank);
      insert.first_rank = s_rank;
      insert.last_rank = insertion_rank;
    } else {
      insert.range.reserve(s_rank - insertion_rank + 1);
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
                                         const Eval& best_known_gain) {
  // Preprocessing phase.
  std::vector<ThreeInsertions> top_insertions_in_target(source.route.size(),
                                                        empty_three_insertions);
  for (unsigned s_rank = 0; s_rank < source.route.size(); ++s_rank) {
    const auto source_job_rank = source.route[s_rank];

    if (input.jobs[source_job_rank].type == JOB_TYPE::SINGLE &&
        input.vehicle_ok_with_job(t_vehicle, source_job_rank)) {
      top_insertions_in_target[s_rank] =
        find_top_3_insertions(input, source_job_rank, target);
    }
  }

  std::vector<ThreeInsertions> top_insertions_in_source(target.route.size(),
                                                        empty_three_insertions);
  for (unsigned t_rank = 0; t_rank < target.route.size(); ++t_rank) {
    const auto target_job_rank = target.route[t_rank];

    if (input.jobs[target_job_rank].type == JOB_TYPE::SINGLE &&
        input.vehicle_ok_with_job(s_vehicle, target_job_rank)) {
      top_insertions_in_source[t_rank] =
        find_top_3_insertions(input, target_job_rank, source);
    }
  }

  // Search phase.
  auto best_choice = empty_swap_choice;
  Eval best_gain = best_known_gain;

  const auto& s_v = input.vehicles[s_vehicle];
  const auto& t_v = input.vehicles[t_vehicle];

  const auto& s_eval = sol_state.route_evals[s_vehicle];
  const auto& t_eval = sol_state.route_evals[t_vehicle];

  const auto& s_delivery_margin = source.delivery_margin();
  const auto& s_pickup_margin = source.pickup_margin();
  const auto& t_delivery_margin = target.delivery_margin();
  const auto& t_pickup_margin = target.pickup_margin();

  for (unsigned s_rank = 0; s_rank < source.route.size(); ++s_rank) {
    const auto& target_insertions = top_insertions_in_target[s_rank];
    if (target_insertions[0].eval == NO_EVAL) {
      continue;
    }

    // sol_state.node_gains contains the Delta value we're looking for
    // except in the case of a single-step route with a start and end,
    // where the start->end cost is not accounted for.
    const auto source_start_end_cost =
      (source.size() == 1 && s_v.has_start() && s_v.has_end())
        ? s_v.eval(s_v.start.value().index(), s_v.end.value().index())
        : Eval();
    const auto source_delta =
      sol_state.node_gains[s_vehicle][s_rank] - source_start_end_cost;

    for (unsigned t_rank = 0; t_rank < target.route.size(); ++t_rank) {
      const auto& source_insertions = top_insertions_in_source[t_rank];
      if (source_insertions[0].eval == NO_EVAL) {
        continue;
      }

      // Same as above.
      const auto target_start_end_cost =
        (target.size() == 1 && t_v.has_start() && t_v.has_end())
          ? t_v.eval(t_v.start.value().index(), t_v.end.value().index())
          : Eval();
      const auto target_delta =
        sol_state.node_gains[t_vehicle][t_rank] - target_start_end_cost;

      if (source_delta + target_delta <= best_gain) {
        continue;
      }

      const auto target_in_place_delta =
        utils::in_place_delta_eval(input,
                                   source.route[s_rank],
                                   t_v,
                                   target.route,
                                   t_rank);

      const auto source_in_place_delta =
        utils::in_place_delta_eval(input,
                                   target.route[t_rank],
                                   s_v,
                                   source.route,
                                   s_rank);

      std::vector<SwapChoice> swap_choice_options;
      constexpr std::size_t MAX_SWAP_CHOICES = 16;
      swap_choice_options.reserve(MAX_SWAP_CHOICES);

      // Options for in-place insertion in source route include
      // in-place insertion in target route and other relevant
      // positions from target_insertions.
      const Eval in_place_s_gain = source_delta - source_in_place_delta;
      const Eval in_place_t_gain = target_delta - target_in_place_delta;

      Eval current_gain = in_place_s_gain + in_place_t_gain;

      if (s_v.ok_for_range_bounds(s_eval - in_place_s_gain)) {
        // Only bother further checking in-place insertion in source
        // route if max travel time constraint is OK.
        if (best_gain < current_gain &&
            t_v.ok_for_range_bounds(t_eval - in_place_t_gain)) {
          SwapChoice sc(current_gain, s_rank, t_rank, s_rank, t_rank);
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
          if ((ti.rank != t_rank) && (ti.rank != t_rank + 1) &&
              (ti.eval != NO_EVAL)) {
            const Eval t_gain = target_delta - ti.eval;
            current_gain = in_place_s_gain + t_gain;
            if (best_gain < current_gain &&
                t_v.ok_for_range_bounds(t_eval - t_gain)) {
              SwapChoice sc(current_gain, s_rank, t_rank, s_rank, ti.rank);
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

      // Options for other relevant positions for insertion in source
      // route (from source_insertions) include in-place insertion in
      // target route and other relevant positions from
      // target_insertions.
      for (const auto& si : source_insertions) {
        if ((si.rank != s_rank) && (si.rank != s_rank + 1) &&
            (si.eval != NO_EVAL)) {
          const Eval s_gain = source_delta - si.eval;

          if (!s_v.ok_for_range_bounds(s_eval - s_gain)) {
            // Don't bother further checking if max travel time
            // constraint is violated for source route.
            continue;
          }

          current_gain = s_gain + in_place_t_gain;
          if (best_gain < current_gain &&
              t_v.ok_for_range_bounds(t_eval - in_place_t_gain)) {
            SwapChoice sc(current_gain, s_rank, t_rank, si.rank, t_rank);
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
            if ((ti.rank != t_rank) && (ti.rank != t_rank + 1) &&
                (ti.eval != NO_EVAL)) {
              const Eval t_gain = target_delta - ti.eval;
              current_gain = s_gain + t_gain;
              if (best_gain < current_gain &&
                  t_v.ok_for_range_bounds(t_eval - t_gain)) {
                SwapChoice sc(current_gain, s_rank, t_rank, si.rank, ti.rank);
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

      std::ranges::sort(swap_choice_options, SwapChoiceCmp);

      assert(swap_choice_options.size() <= MAX_SWAP_CHOICES);

      for (const auto& sc : swap_choice_options) {
        // Browse interesting options by decreasing gain and check for
        // validity.

        // Early abort on invalid capacity bounds.
        const auto s_index = source.route[sc.s_rank];
        const auto& s_delivery = input.jobs[s_index].delivery;
        const auto& s_pickup = input.jobs[s_index].pickup;
        const auto t_index = target.route[sc.t_rank];
        const auto& t_delivery = input.jobs[t_index].delivery;

        if (const auto& t_pickup = input.jobs[t_index].pickup;
            !(t_delivery <= s_delivery_margin + s_delivery) ||
            !(t_pickup <= s_pickup_margin + s_pickup) ||
            !(s_delivery <= t_delivery_margin + t_delivery) ||
            !(s_pickup <= t_pickup_margin + t_pickup)) {
          continue;
        }

        const auto s_insert = get_insert_range(source.route,
                                               s_rank,
                                               target.route[t_rank],
                                               sc.insertion_in_source);

        Amount source_pickup = input.zero_amount();
        Amount source_delivery = input.zero_amount();
        for (auto i : s_insert.range) {
          const auto& job = input.jobs[i];
          if (job.type == JOB_TYPE::SINGLE) {
            source_pickup += job.pickup;
            source_delivery += job.delivery;
          }
        }

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
                                                       source_delivery,
                                                       s_insert.range.begin(),
                                                       s_insert.range.end(),
                                                       s_insert.first_rank,
                                                       s_insert.last_rank);

        if (source_valid) {
          const auto t_insert = get_insert_range(target.route,
                                                 t_rank,
                                                 source.route[s_rank],
                                                 sc.insertion_in_target);

          Amount target_pickup = input.zero_amount();
          Amount target_delivery = input.zero_amount();
          for (auto i : t_insert.range) {
            const auto& job = input.jobs[i];
            if (job.type == JOB_TYPE::SINGLE) {
              target_pickup += job.pickup;
              target_delivery += job.delivery;
            }
          }

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
                                                         target_delivery,
                                                         t_insert.range.begin(),
                                                         t_insert.range.end(),
                                                         t_insert.first_rank,
                                                         t_insert.last_rank);

          if (target_valid) {
            best_gain = sc.gain;
            best_choice = sc;
            best_choice.source_range_delivery = source_delivery;
            best_choice.target_range_delivery = target_delivery;
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

} // namespace vroom::ls

#endif
