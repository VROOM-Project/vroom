#ifndef ROUTE_SPLIT_UTILS_H
#define ROUTE_SPLIT_UTILS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>

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

// Pick the best and second best vehicles for one side of the split
// (evals are NO_EVAL for vehicles that can not take that side at
// all), checking full validity lazily through tw_valid, only for
// vehicles that would improve on the current second best.
template <class TwValid>
void pick_best_two_split_vehicles(const std::vector<Eval>& evals,
                                  TwValid&& tw_valid,
                                  Eval& first_best_eval,
                                  Index& first_v,
                                  Eval& second_best_eval,
                                  Index& second_v) {
  for (Index v_rank = 0; v_rank < evals.size(); ++v_rank) {
    const auto& current_eval = evals[v_rank];
    if (current_eval == NO_EVAL) {
      continue;
    }

    if (current_eval < second_best_eval &&
        // Worth checking route full validity.
        tw_valid(v_rank)) {
      if (current_eval < first_best_eval) {
        // We have a new first choice.
        second_v = first_v;
        second_best_eval = first_best_eval;

        first_v = v_rank;
        first_best_eval = current_eval;
      } else {
        // We have a new second choice.
        second_v = v_rank;
        second_best_eval = current_eval;
      }
    }
  }
}

template <class Route>
SplitChoice
compute_best_route_split_choice(const Input& input,
                                const utils::SolutionState& sol_state,
                                const Index s_vehicle,
                                const Route& source,
                                const std::vector<Index>& empty_route_ranks,
                                const std::vector<Route>& sol,
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

    const auto end_max_load = source.sub_route_max_load_after(r);
    const auto end_delivery = source.delivery_in_range(r, source.size());

    // Cheap checks and eval for using each empty vehicle as end
    // route, NO_EVAL when it can not.
    std::vector<Eval> end_evals(empty_route_ranks.size(), NO_EVAL);

    for (Index v_rank = 0; v_rank < empty_route_ranks.size(); ++v_rank) {
      const auto v = empty_route_ranks[v_rank];
      const auto& end_v = input.vehicles[v];

      if (r < sol_state.bwd_skill_rank[s_vehicle][v] ||
          !(end_max_load <= end_v.capacity) ||
          end_v.max_tasks < source.size() - r) {
        continue;
      }

      if (end_v.has_alternative_capacities() &&
          !end_v.can_carry(end_max_load) &&
          // Loads in end route are current loads minus pickups
          // done before rank r.
          !source.loads_fit(end_v,
                            r,
                            source.nb_steps(),
                            input.zero_amount() - source.fwd_pickups(r - 1))) {
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

      end_evals[v_rank] = current_end_eval;
    }

    const auto end_tw_valid = [&](Index v_rank) {
      return empty_routes[v_rank].is_valid_addition_for_tw(input,
                                                           end_delivery,
                                                           source.route.begin() +
                                                             r,
                                                           source.route.end(),
                                                           0,
                                                           0);
    };

    auto first_best_end_eval = NO_EVAL;
    Index first_v_end = 0; // dummy init
    auto second_best_end_eval = NO_EVAL;
    Index second_v_end = 0; // dummy init

    if (!input.has_vehicle_groups()) {
      pick_best_two_split_vehicles(end_evals,
                                   end_tw_valid,
                                   first_best_end_eval,
                                   first_v_end,
                                   second_best_end_eval,
                                   second_v_end);
    } else {
      // Validity is checked lazily further down, so only bound the
      // gain here with the cheapest end route, valid or not.
      first_best_end_eval =
        *std::min_element(end_evals.begin(),
                          end_evals.end(),
                          [](const Eval& a, const Eval& b) { return a < b; });
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

    const auto begin_max_load = source.sub_route_max_load_before(r);
    const auto begin_delivery = source.delivery_in_range(0, r);

    // Same for the begin route.
    std::vector<Eval> begin_evals(empty_route_ranks.size(), NO_EVAL);

    for (Index v_rank = 0; v_rank < empty_route_ranks.size(); ++v_rank) {
      const auto v = empty_route_ranks[v_rank];
      const auto& begin_v = input.vehicles[v];

      if (sol_state.fwd_skill_rank[s_vehicle][v] < r ||
          !(begin_max_load <= begin_v.capacity) || begin_v.max_tasks < r) {
        continue;
      }

      if (begin_v.has_alternative_capacities() &&
          !begin_v.can_carry(begin_max_load) &&
          // Loads in begin route are current loads minus deliveries
          // pending after rank r - 1.
          !source.loads_fit(begin_v,
                            0,
                            r + 1,
                            input.zero_amount() -
                              source.bwd_deliveries(r - 1))) {
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

      begin_evals[v_rank] = current_begin_eval;
    }

    const auto begin_tw_valid = [&](Index v_rank) {
      return empty_routes[v_rank].is_valid_addition_for_tw(input,
                                                           begin_delivery,
                                                           source.route.begin(),
                                                           source.route.begin() +
                                                             r,
                                                           0,
                                                           0);
    };

    SplitChoice current_split_choice;

    if (input.has_vehicle_groups()) {
      // The two vehicles are opened at once and the source route is
      // emptied, and a vehicle group limit applies to the pair, not
      // to each vehicle on its own (both may be allowed alone and
      // not together). So search the best valid pair directly,
      // vehicles sorted by eval on each side and validity checked
      // lazily, stopping as soon as no pair can beat the best one.
      std::vector<Index> begin_order(empty_route_ranks.size());
      std::iota(begin_order.begin(), begin_order.end(), 0);
      std::erase_if(begin_order,
                    [&](Index v_rank) { return begin_evals[v_rank] == NO_EVAL; });
      std::ranges::sort(begin_order, [&](Index a, Index b) {
        return begin_evals[a] < begin_evals[b];
      });

      std::vector<Index> end_order(empty_route_ranks.size());
      std::iota(end_order.begin(), end_order.end(), 0);
      std::erase_if(end_order,
                    [&](Index v_rank) { return end_evals[v_rank] == NO_EVAL; });
      std::ranges::sort(end_order, [&](Index a, Index b) {
        return end_evals[a] < end_evals[b];
      });

      if (begin_order.empty() || end_order.empty()) {
        continue;
      }

      // Validity results, computed at most once per vehicle and side.
      std::vector<std::optional<bool>> begin_valid(empty_route_ranks.size());
      std::vector<std::optional<bool>> end_valid(empty_route_ranks.size());
      const auto is_begin_valid = [&](Index v_rank) {
        if (!begin_valid[v_rank].has_value()) {
          begin_valid[v_rank] = begin_tw_valid(v_rank);
        }
        return begin_valid[v_rank].value();
      };
      const auto is_end_valid = [&](Index v_rank) {
        if (!end_valid[v_rank].has_value()) {
          end_valid[v_rank] = end_tw_valid(v_rank);
        }
        return end_valid[v_rank].value();
      };

      auto best_pair_eval = NO_EVAL;
      Index best_v_begin = 0; // dummy init
      Index best_v_end = 0;   // dummy init

      for (const auto v_begin : begin_order) {
        if (!(begin_evals[v_begin] + end_evals[end_order.front()] <
              best_pair_eval)) {
          // No remaining begin route can do better.
          break;
        }
        if (!is_begin_valid(v_begin)) {
          continue;
        }

        for (const auto v_end : end_order) {
          const auto pair_eval = begin_evals[v_begin] + end_evals[v_end];
          if (!(pair_eval < best_pair_eval)) {
            break;
          }
          if (v_end == v_begin ||
              !input.vehicle_groups_allow_opening_both(
                empty_route_ranks[v_begin],
                empty_route_ranks[v_end],
                s_vehicle,
                sol) ||
              !is_end_valid(v_end)) {
            continue;
          }

          // End routes are sorted, so this is the best pair for
          // current begin route.
          best_pair_eval = pair_eval;
          best_v_begin = v_begin;
          best_v_end = v_end;
          break;
        }
      }

      if (best_pair_eval == NO_EVAL) {
        continue;
      }

      current_split_choice = {init_eval - best_pair_eval,
                              r,
                              best_v_begin,
                              best_v_end};
    } else {
      auto first_best_begin_eval = NO_EVAL;
      Index first_v_begin = 0; // dummy init
      auto second_best_begin_eval = NO_EVAL;
      Index second_v_begin = 0; // dummy init

      pick_best_two_split_vehicles(begin_evals,
                                   begin_tw_valid,
                                   first_best_begin_eval,
                                   first_v_begin,
                                   second_best_begin_eval,
                                   second_v_begin);

      if (first_best_begin_eval == NO_EVAL) {
        // Begin route is valid for none of the empty vehicles, so
        // splitting on current rank is not doable anyway.
        continue;
      }

      // Now we have at least one valid candidate for begin and end
      // route.
      if (first_v_begin != first_v_end) {
        current_split_choice = {init_eval - first_best_begin_eval -
                                  first_best_end_eval,
                                r,
                                first_v_begin,
                                first_v_end};
      } else {
        // Candidates are identical so we need to check second bests,
        // if any.
        if (second_best_begin_eval == NO_EVAL) {
          if (second_best_end_eval == NO_EVAL) {
            // No split possible as there is only one valid vehicle
            // for begin and end route.
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
    }

    if (best_choice.gain < current_split_choice.gain) {
      best_choice = current_split_choice;
    }
  }

  return best_choice;
}

} // namespace vroom::ls

#endif
