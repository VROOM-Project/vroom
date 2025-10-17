#ifndef INSERTION_SEARCH_H
#define INSERTION_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

namespace vroom::ls {

struct RouteInsertion {
  Eval eval{NO_EVAL};
  Amount delivery;
  Index single_rank{0};
  Index pickup_rank{0};
  Index delivery_rank{0};

  explicit RouteInsertion(unsigned amount_size)
    : delivery(Amount(amount_size)) {
  }
};

template <class Route>
RouteInsertion
compute_best_insertion_single(const Input& input,
                              const utils::SolutionState& sol_state,
                              const Index j,
                              Index v,
                              const Route& route) {
  RouteInsertion result(input.get_amount_size());
  const auto& current_job = input.jobs[j];
  const auto& v_target = input.vehicles[v];

  if (input.vehicle_ok_with_job(v, j)) {
    for (Index rank = sol_state.insertion_ranks_begin[v][j];
         rank < sol_state.insertion_ranks_end[v][j];
         ++rank) {
      const Eval current_eval =
        utils::addition_eval(input, j, v_target, route.route, rank);
      if (current_eval.cost < result.eval.cost &&
          v_target.ok_for_range_bounds(sol_state.route_evals[v] +
                                       current_eval) &&
          route.is_valid_addition_for_capacity(input,
                                               current_job.pickup,
                                               current_job.delivery,
                                               rank) &&
          route.is_valid_addition_for_tw(input, j, rank)) {
        result.eval = current_eval;
        result.delivery = current_job.delivery;
        result.single_rank = rank;
      }
    }
  }
  return result;
}

template <class Route, std::forward_iterator Iter>
bool valid_for_capacity(const Input& input,
                        const Route& r,
                        Iter start,
                        Iter end,
                        Index pickup_r,
                        Index delivery_r) {
  Amount amount = input.zero_amount();

  for (auto it = start + 1; it != end - 1; ++it) {
    const auto& new_modified_job = input.jobs[*it];
    if (new_modified_job.type == JOB_TYPE::SINGLE) {
      amount += new_modified_job.delivery;
    }
  }

  return r.is_valid_addition_for_capacity_inclusion(input,
                                                    std::move(amount),
                                                    start,
                                                    end,
                                                    pickup_r,
                                                    delivery_r);
}

template <class Route>
RouteInsertion compute_best_insertion_pd(const Input& input,
                                         const utils::SolutionState& sol_state,
                                         const Index j,
                                         Index v,
                                         const Route& route,
                                         const Eval& cost_threshold) {
  RouteInsertion result(input.get_amount_size());
  const auto& current_job = input.jobs[j];
  const auto& v_target = input.vehicles[v];

  if (!input.vehicle_ok_with_job(v, j)) {
    return result;
  }

  result.eval = cost_threshold;

  // Pre-compute cost of addition for matching delivery.
  std::vector<Eval> d_adds(route.size() + 1);
  std::vector<unsigned char> valid_delivery_insertions(route.size() + 1, false);

  const auto begin_d_rank = sol_state.insertion_ranks_begin[v][j + 1];
  const auto end_d_rank = sol_state.insertion_ranks_end[v][j + 1];

  bool found_valid = false;
  for (unsigned d_rank = begin_d_rank; d_rank < end_d_rank; ++d_rank) {
    d_adds[d_rank] =
      utils::addition_eval(input, j + 1, v_target, route.route, d_rank);
    if (result.eval < d_adds[d_rank]) {
      valid_delivery_insertions[d_rank] = false;
    } else {
      valid_delivery_insertions[d_rank] =
        route.is_valid_addition_for_tw_without_max_load(input, j + 1, d_rank);
    }
    found_valid |= static_cast<bool>(valid_delivery_insertions[d_rank]);
  }

  if (!found_valid) {
    result.eval = NO_EVAL;
    return result;
  }

  for (Index pickup_r = sol_state.insertion_ranks_begin[v][j];
       pickup_r < sol_state.insertion_ranks_end[v][j];
       ++pickup_r) {
    const Eval p_add =
      utils::addition_eval(input, j, v_target, route.route, pickup_r);
    if (result.eval < p_add) {
      // Even without delivery insertion more expensive than current best.
      continue;
    }

    if (!route.is_valid_addition_for_load(input,
                                          current_job.pickup,
                                          pickup_r) ||
        !route.is_valid_addition_for_tw_without_max_load(input, j, pickup_r)) {
      continue;
    }

    // Build replacement sequence for current insertion.
    std::vector<Index> modified_with_pd;
    if (pickup_r <= end_d_rank) {
      modified_with_pd.reserve(end_d_rank - pickup_r + 2);
    }
    modified_with_pd.push_back(j);

    Amount modified_delivery = input.zero_amount();

    // No need to use begin_d_rank here thanks to
    // valid_delivery_insertions values.
    for (Index delivery_r = pickup_r; delivery_r < end_d_rank; ++delivery_r) {
      // Update state variables along the way before potential
      // early abort.
      if (pickup_r < delivery_r) {
        modified_with_pd.push_back(route.route[delivery_r - 1]);
        const auto& new_modified_job = input.jobs[route.route[delivery_r - 1]];
        if (new_modified_job.type == JOB_TYPE::SINGLE) {
          modified_delivery += new_modified_job.delivery;
        }
      }

      if (!static_cast<bool>(valid_delivery_insertions[delivery_r])) {
        continue;
      }

      Eval pd_eval;
      if (pickup_r == delivery_r) {
        pd_eval = utils::addition_eval(input,
                                       j,
                                       v_target,
                                       route.route,
                                       pickup_r,
                                       pickup_r + 1);
      } else {
        pd_eval = p_add + d_adds[delivery_r];
      }

      if (pd_eval < result.eval &&
          v_target.ok_for_range_bounds(sol_state.route_evals[v] + pd_eval)) {
        modified_with_pd.push_back(j + 1);

        // Update best cost depending on validity.
        bool is_valid = valid_for_capacity(input,
                                           route,
                                           modified_with_pd.begin(),
                                           modified_with_pd.end(),
                                           pickup_r,
                                           delivery_r);

        is_valid =
          is_valid && route.is_valid_addition_for_tw(input,
                                                     modified_delivery,
                                                     modified_with_pd.begin(),
                                                     modified_with_pd.end(),
                                                     pickup_r,
                                                     delivery_r);

        modified_with_pd.pop_back();

        if (is_valid) {
          result.eval = pd_eval;
          result.delivery = modified_delivery;
          result.pickup_rank = pickup_r;
          result.delivery_rank = delivery_r;
        }
      }
    }
  }

  assert(result.eval <= cost_threshold);
  if (result.eval == cost_threshold) {
    result.eval = NO_EVAL;
  }
  return result;
}

} // namespace vroom::ls
#endif
