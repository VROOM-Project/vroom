#ifndef INSERTION_SEARCH_H
#define INSERTION_SEARCH_H

#include "structures/typedefs.h"
#include "utils/helpers.h"

namespace vroom {
namespace ls {

struct RouteInsertion {
  Gain cost;
  Index single_rank;
  Index pickup_rank;
  Index delivery_rank;
};
constexpr RouteInsertion empty_insert = {std::numeric_limits<Gain>::max(),
                                         0,
                                         0,
                                         0};

template <class Route>
RouteInsertion compute_best_insertion_single(const Input& input,
                                             const Index j,
                                             Index v,
                                             const Route& route) {
  RouteInsertion result = empty_insert;
  const auto& current_job = input.jobs[j];
  const auto& v_target = input.vehicles[v];

  if (input.vehicle_ok_with_job(v, j)) {

    for (Index rank = 0; rank <= route.size(); ++rank) {
      Gain current_cost =
        utils::addition_cost(input, j, v_target, route.route, rank);
      if (current_cost < result.cost and
          route.is_valid_addition_for_capacity(input,
                                               current_job.pickup,
                                               current_job.delivery,
                                               rank) and
          route.is_valid_addition_for_tw(input, j, rank)) {
        result = {current_cost, rank, 0, 0};
      }
    }
  }
  return result;
}

template <class Route>
RouteInsertion compute_best_insertion_pd(const Input& input,
                                         const Index j,
                                         Index v,
                                         const Route& route,
                                         const Gain cost_threshold) {
  RouteInsertion result = empty_insert;
  const auto& current_job = input.jobs[j];
  const auto& v_target = input.vehicles[v];

  if (!input.vehicle_ok_with_job(v, j)) {
    return result;
  }

  result.cost = cost_threshold;

  // Pre-compute cost of addition for matching delivery.
  std::vector<Gain> d_adds(route.size() + 1);
  std::vector<unsigned char> valid_delivery_insertions(route.size() + 1);

  for (unsigned d_rank = 0; d_rank <= route.size(); ++d_rank) {
    d_adds[d_rank] =
      utils::addition_cost(input, j + 1, v_target, route.route, d_rank);
    valid_delivery_insertions[d_rank] =
      route.is_valid_addition_for_tw(input, j + 1, d_rank);
  }

  for (Index pickup_r = 0; pickup_r <= route.size(); ++pickup_r) {
    Gain p_add =
      utils::addition_cost(input, j, v_target, route.route, pickup_r);
    if (p_add > result.cost) {
      // Even without delivery insertion more expensive then current best
      continue;
    }

    if (!route.is_valid_addition_for_load(input,
                                          current_job.pickup,
                                          pickup_r) or
        !route.is_valid_addition_for_tw(input, j, pickup_r)) {
      continue;
    }

    // Build replacement sequence for current insertion.
    std::vector<Index> modified_with_pd({j});
    Amount modified_delivery = input.zero_amount();

    for (Index delivery_r = pickup_r; delivery_r <= route.size();
         ++delivery_r) {
      // Update state variables along the way before potential
      // early abort.
      if (pickup_r < delivery_r) {
        modified_with_pd.push_back(route.route[delivery_r - 1]);
        const auto& new_modified_job = input.jobs[route.route[delivery_r - 1]];
        if (new_modified_job.type == JOB_TYPE::SINGLE) {
          modified_delivery += new_modified_job.delivery;
        }
      }

      if (!(bool)valid_delivery_insertions[delivery_r]) {
        continue;
      }

      Gain pd_cost;
      if (pickup_r == delivery_r) {
        pd_cost = utils::addition_cost(input,
                                       j,
                                       v_target,
                                       route.route,
                                       pickup_r,
                                       pickup_r + 1);
      } else {
        pd_cost = p_add + d_adds[delivery_r];
      }

      Gain current_cost = pd_cost;

      if (current_cost < result.cost) {
        modified_with_pd.push_back(j + 1);

        // Update best cost depending on validity.
        bool is_valid =
          route
            .is_valid_addition_for_capacity_inclusion(input,
                                                      modified_delivery,
                                                      modified_with_pd.begin(),
                                                      modified_with_pd.end(),
                                                      pickup_r,
                                                      delivery_r);

        is_valid =
          is_valid && route.is_valid_addition_for_tw(input,
                                                     modified_with_pd.begin(),
                                                     modified_with_pd.end(),
                                                     pickup_r,
                                                     delivery_r);

        modified_with_pd.pop_back();

        if (is_valid) {
          result = {current_cost, 0, pickup_r, delivery_r};
        }
      }
    }
  }
  if (result.cost == cost_threshold) {
    result.cost = std::numeric_limits<Gain>::max();
  }
  return result;
}

} // namespace ls
} // namespace vroom
#endif
