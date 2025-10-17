#ifndef ROUTE_SPLIT_UTILS_H
#define ROUTE_SPLIT_UTILS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

namespace vroom::ls {

struct SplitChoice {
  Eval gain;
  Index split_rank;
  // Vehicle ranks are relative to empty_route_ranks.
  Index v_begin;
  Index v_end;
};

constexpr SplitChoice empty_route_split_choice = {NO_GAIN, 0, 0, 0};

template <class Route>
SplitChoice
compute_best_route_split_choice(const Input& input,
                                const utils::SolutionState& sol_state,
                                const Index s_vehicle,
                                const Route& source,
                                const std::vector<Index>& empty_route_ranks,
                                const Eval& best_known_gain) {
  auto best_choice = empty_route_split_choice;

  // Create actual empty routes for idle vehicles to use below in
  // validity checks.
  std::vector<Route> empty_routes;
  empty_routes.reserve(empty_route_ranks.size());
  for (auto v : empty_route_ranks) {
    empty_routes.emplace_back(input, v, input.zero_amount().size());
  }

  for (Index r = 1; r < source.size(); ++r) {
    // Starting at 1 in order to split in two "real" routes. "Begin"
    // route from start up to r (excluded) and "end" route from r to
    // the end.

    if (source.has_pending_delivery_after_rank(r - 1)) {
      continue;
    }

    auto first_best_end_eval = NO_EVAL;
    Index first_v_end = 0; // dummy init
    auto second_best_end_eval = NO_EVAL;
    Index second_v_end = 0; // dummy init

    const auto end_max_load = source.sub_route_max_load_after(r);
    const auto end_delivery = source.delivery_in_range(r, source.size());

    for (Index v_rank = 0; v_rank < empty_route_ranks.size(); ++v_rank) {
      const auto v = empty_route_ranks[v_rank];
      const auto& end_v = input.vehicles[v];

      if (r < sol_state.bwd_skill_rank[s_vehicle][v] ||
          !(end_max_load <= end_v.capacity) ||
          end_v.max_tasks < source.size() - r) {
        continue;
      }

      const auto current_end_eval =
        -std::get<0>(utils::addition_eval_delta(input,
                                                sol_state,
                                                empty_routes[v_rank],
                                                0,
                                                0,
                                                source,
                                                r,
                                                source.size()));

      if (!end_v.ok_for_range_bounds(current_end_eval)) {
        continue;
      }

      if (current_end_eval < second_best_end_eval &&
          // Worth checking end route full validity.
          empty_routes[v_rank].is_valid_addition_for_tw(input,
                                                        end_delivery,
                                                        source.route.begin() +
                                                          r,
                                                        source.route.end(),
                                                        0,
                                                        0)) {
        if (current_end_eval < first_best_end_eval) {
          // We have a new first choice.
          second_v_end = first_v_end;
          second_best_end_eval = first_best_end_eval;

          first_v_end = v_rank;
          first_best_end_eval = current_end_eval;
        } else {
          // We have a new second choice.
          second_v_end = v_rank;
          second_best_end_eval = current_end_eval;
        }
      }
    }

    if (first_best_end_eval == NO_EVAL) {
      // End route is valid for none of the empty vehicles, so
      // splitting on current rank is not doable anyway.
      continue;
    }

    const auto& init_eval = sol_state.route_evals[s_vehicle];
    if (init_eval - first_best_end_eval <= best_known_gain) {
      // Overall gain will be even lower with begin route cost.
      continue;
    }

    auto first_best_begin_eval = NO_EVAL;
    Index first_v_begin = 0; // dummy init
    auto second_best_begin_eval = NO_EVAL;
    Index second_v_begin = 0; // dummy init

    const auto begin_max_load = source.sub_route_max_load_before(r);
    const auto begin_delivery = source.delivery_in_range(0, r);

    for (Index v_rank = 0; v_rank < empty_route_ranks.size(); ++v_rank) {
      const auto v = empty_route_ranks[v_rank];
      const auto& begin_v = input.vehicles[v];

      if (sol_state.fwd_skill_rank[s_vehicle][v] < r ||
          !(begin_max_load <= begin_v.capacity) || begin_v.max_tasks < r) {
        continue;
      }

      const auto current_begin_eval =
        -std::get<0>(utils::addition_eval_delta(input,
                                                sol_state,
                                                empty_routes[v_rank],
                                                0,
                                                0,
                                                source,
                                                0,
                                                r));

      if (!begin_v.ok_for_range_bounds(current_begin_eval)) {
        continue;
      }

      if (current_begin_eval < second_best_begin_eval &&
          // Worth checking begin route full validity.
          empty_routes[v_rank].is_valid_addition_for_tw(input,
                                                        begin_delivery,
                                                        source.route.begin(),
                                                        source.route.begin() +
                                                          r,
                                                        0,
                                                        0)) {
        if (current_begin_eval < first_best_begin_eval) {
          // We have a new first choice.
          second_v_begin = first_v_begin;
          second_best_begin_eval = first_best_begin_eval;

          first_v_begin = v_rank;
          first_best_begin_eval = current_begin_eval;
        } else {
          // We have a new second choice.
          second_v_begin = v_rank;
          second_best_begin_eval = current_begin_eval;
        }
      }
    }

    if (first_best_begin_eval == NO_EVAL) {
      // Begin route is valid for none of the empty vehicles, so
      // splitting on current rank is not doable anyway.
      continue;
    }

    // Now we have at least one valid candidate for begin and end
    // route.
    SplitChoice current_split_choice;

    if (first_v_begin != first_v_end) {
      current_split_choice = {init_eval - first_best_begin_eval -
                                first_best_end_eval,
                              r,
                              first_v_begin,
                              first_v_end};
    } else {
      // Candidates are identical so we need to check second bests, if
      // any.
      if (second_best_begin_eval == NO_EVAL) {
        if (second_best_end_eval == NO_EVAL) {
          // No split possible as there is only one valid vehicle for
          // begin and end route.
          continue;
        }
        current_split_choice = {init_eval - first_best_begin_eval -
                                  second_best_end_eval,
                                r,
                                first_v_begin,
                                second_v_end};
      } else {
        if (second_best_end_eval == NO_EVAL) {
          current_split_choice = {init_eval - second_best_begin_eval -
                                    first_best_end_eval,
                                  r,
                                  second_v_begin,
                                  first_v_end};
        } else {
          // We do have second bests for both begin and end route,
          // checking best option.
          if (first_best_begin_eval + second_best_end_eval <
              second_best_begin_eval + first_best_end_eval) {
            current_split_choice = {init_eval - first_best_begin_eval -
                                      second_best_end_eval,
                                    r,
                                    first_v_begin,
                                    second_v_end};
          } else {
            current_split_choice = {init_eval - second_best_begin_eval -
                                      first_best_end_eval,
                                    r,
                                    second_v_begin,
                                    first_v_end};
          }
        }
      }
    }

    if (best_choice.gain < current_split_choice.gain) {
      best_choice = current_split_choice;
    }
  }

  return best_choice;
}

} // namespace vroom::ls

#endif
