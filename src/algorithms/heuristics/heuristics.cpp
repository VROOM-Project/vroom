/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "algorithms/heuristics/heuristics.h"
#include "utils/helpers.h"

namespace vroom::heuristics {

// Add seed job to route if required and return current cost of route
// without vehicle fixed cost.
template <class Route>
inline void seed_route(const Input& input,
                       Route& route,
                       INIT init,
                       const std::vector<std::vector<Eval>>& evals,
                       std::set<Index>& unassigned,
                       auto job_not_ok) {
  assert(route.empty() && init != INIT::NONE);

  const auto v_rank = route.vehicle_rank;
  const auto& vehicle = input.vehicles[v_rank];

  // Initialize current route with the "best" valid job.
  bool init_ok = false;

  Amount higher_amount(input.zero_amount());
  Cost furthest_cost = 0;
  Cost nearest_cost = std::numeric_limits<Cost>::max();
  Duration earliest_deadline = std::numeric_limits<Duration>::max();
  Index best_job_rank = 0;
  for (const auto job_rank : unassigned) {
    const auto& current_job = input.jobs[job_rank];

    if (!input.vehicle_ok_with_job(v_rank, job_rank) ||
        current_job.type == JOB_TYPE::DELIVERY || job_not_ok(job_rank)) {
      continue;
    }

    bool is_pickup = (current_job.type == JOB_TYPE::PICKUP);

    if (route.size() + (is_pickup ? 2 : 1) > vehicle.max_tasks) {
      continue;
    }

    bool try_validity = false;

    if (init == INIT::HIGHER_AMOUNT) {
      try_validity = (higher_amount < current_job.pickup ||
                      higher_amount < current_job.delivery);
    }
    if (init == INIT::EARLIEST_DEADLINE) {
      Duration current_deadline = is_pickup
                                    ? input.jobs[job_rank + 1].tws.back().end
                                    : current_job.tws.back().end;
      try_validity = (current_deadline < earliest_deadline);
    }
    if (init == INIT::FURTHEST) {
      try_validity = (furthest_cost < evals[job_rank][v_rank].cost);
    }
    if (init == INIT::NEAREST) {
      try_validity = (evals[job_rank][v_rank].cost < nearest_cost);
    }

    if (!try_validity) {
      continue;
    }

    bool is_valid = (vehicle.ok_for_range_bounds(evals[job_rank][v_rank])) &&
                    route.is_valid_addition_for_capacity(input,
                                                         current_job.pickup,
                                                         current_job.delivery,
                                                         0);
    if (is_pickup) {
      std::vector<Index> p_d({job_rank, static_cast<Index>(job_rank + 1)});
      is_valid = is_valid && route.is_valid_addition_for_tw(input,
                                                            input.zero_amount(),
                                                            p_d.begin(),
                                                            p_d.end(),
                                                            0,
                                                            0);
    } else {
      assert(current_job.type == JOB_TYPE::SINGLE);
      is_valid = is_valid && route.is_valid_addition_for_tw(input, job_rank, 0);
    }

    if (is_valid) {
      init_ok = true;
      best_job_rank = job_rank;

      switch (init) {
        using enum INIT;
      case NONE:
        assert(false);
        break;
      case HIGHER_AMOUNT:
        if (higher_amount < current_job.pickup) {
          higher_amount = current_job.pickup;
        }
        if (higher_amount < current_job.delivery) {
          higher_amount = current_job.delivery;
        }
        break;
      case EARLIEST_DEADLINE:
        earliest_deadline = is_pickup ? input.jobs[job_rank + 1].tws.back().end
                                      : current_job.tws.back().end;
        break;
      case FURTHEST:
        furthest_cost = evals[job_rank][v_rank].cost;
        break;
      case NEAREST:
        nearest_cost = evals[job_rank][v_rank].cost;
        break;
      }
    }
  }

  if (init_ok) {
    if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
      route.add(input, best_job_rank, 0);
      unassigned.erase(best_job_rank);
    }
    if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
      std::vector<Index> p_d(
        {best_job_rank, static_cast<Index>(best_job_rank + 1)});
      route.replace(input, input.zero_amount(), p_d.begin(), p_d.end(), 0, 0);
      unassigned.erase(best_job_rank);
      unassigned.erase(best_job_rank + 1);
    }
  }
}

template <class Route>
inline Eval fill_route(const Input& input,
                       Route& route,
                       std::set<Index>& unassigned,
                       const std::vector<Cost>& regrets,
                       double lambda) {
  const auto v_rank = route.vehicle_rank;
  const auto& vehicle = input.vehicles[v_rank];

  const bool init_route_is_empty = route.empty();
  Eval route_eval = utils::route_eval_for_vehicle(input, v_rank, route.route);

  bool keep_going = true;
  while (keep_going) {
    keep_going = false;
    double best_cost = std::numeric_limits<double>::max();
    Index best_job_rank = 0;
    Index best_r = 0;
    Index best_pickup_r = 0;
    Index best_delivery_r = 0;
    Amount best_modified_delivery = input.zero_amount();
    Eval best_eval;

    for (const auto job_rank : unassigned) {
      if (!input.vehicle_ok_with_job(v_rank, job_rank)) {
        continue;
      }

      const auto& current_job = input.jobs[job_rank];

      if (current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }

      if (current_job.type == JOB_TYPE::SINGLE &&
          route.size() + 1 <= vehicle.max_tasks) {
        for (Index r = 0; r <= route.size(); ++r) {
          const auto current_eval =
            utils::addition_cost(input, job_rank, vehicle, route.route, r);

          double current_cost = static_cast<double>(current_eval.cost) -
                                lambda * static_cast<double>(regrets[job_rank]);

          if (current_cost < best_cost &&
              (vehicle.ok_for_range_bounds(route_eval + current_eval)) &&
              route.is_valid_addition_for_capacity(input,
                                                   current_job.pickup,
                                                   current_job.delivery,
                                                   r) &&
              route.is_valid_addition_for_tw(input, job_rank, r)) {
            best_cost = current_cost;
            best_job_rank = job_rank;
            best_r = r;
            best_eval = current_eval;
          }
        }
      }

      if (current_job.type == JOB_TYPE::PICKUP &&
          route.size() + 2 <= vehicle.max_tasks) {
        // Pre-compute cost of addition for matching delivery.
        std::vector<Eval> d_adds(route.route.size() + 1);
        std::vector<unsigned char> valid_delivery_insertions(
          route.route.size() + 1);

        for (unsigned d_rank = 0; d_rank <= route.route.size(); ++d_rank) {
          d_adds[d_rank] = utils::addition_cost(input,
                                                job_rank + 1,
                                                vehicle,
                                                route.route,
                                                d_rank);
          valid_delivery_insertions[d_rank] =
            route.is_valid_addition_for_tw_without_max_load(input,
                                                            job_rank + 1,
                                                            d_rank);
        }

        for (Index pickup_r = 0; pickup_r <= route.size(); ++pickup_r) {
          const auto p_add = utils::addition_cost(input,
                                                  job_rank,
                                                  vehicle,
                                                  route.route,
                                                  pickup_r);

          if (!route.is_valid_addition_for_load(input,
                                                current_job.pickup,
                                                pickup_r) ||
              !route.is_valid_addition_for_tw_without_max_load(input,
                                                               job_rank,
                                                               pickup_r)) {
            continue;
          }

          // Build replacement sequence for current insertion.
          std::vector<Index> modified_with_pd;
          modified_with_pd.reserve(route.size() - pickup_r + 2);
          modified_with_pd.push_back(job_rank);

          Amount modified_delivery = input.zero_amount();

          for (Index delivery_r = pickup_r; delivery_r <= route.size();
               ++delivery_r) {
            // Update state variables along the way before potential
            // early abort.
            if (pickup_r < delivery_r) {
              modified_with_pd.push_back(route.route[delivery_r - 1]);
              const auto& new_modified_job =
                input.jobs[route.route[delivery_r - 1]];
              if (new_modified_job.type == JOB_TYPE::SINGLE) {
                modified_delivery += new_modified_job.delivery;
              }
            }

            if (!static_cast<bool>(valid_delivery_insertions[delivery_r])) {
              continue;
            }

            Eval current_eval;
            if (pickup_r == delivery_r) {
              current_eval = utils::addition_cost(input,
                                                  job_rank,
                                                  vehicle,
                                                  route.route,
                                                  pickup_r,
                                                  pickup_r + 1);
            } else {
              current_eval = p_add + d_adds[delivery_r];
            }

            double current_cost =
              current_eval.cost -
              lambda * static_cast<double>(regrets[job_rank]);

            if (current_cost < best_cost) {
              modified_with_pd.push_back(job_rank + 1);

              // Update best cost depending on validity.
              bool valid =
                (vehicle.ok_for_range_bounds(route_eval + current_eval)) &&
                route
                  .is_valid_addition_for_capacity_inclusion(input,
                                                            modified_delivery,
                                                            modified_with_pd
                                                              .begin(),
                                                            modified_with_pd
                                                              .end(),
                                                            pickup_r,
                                                            delivery_r) &&
                route.is_valid_addition_for_tw(input,
                                               modified_delivery,
                                               modified_with_pd.begin(),
                                               modified_with_pd.end(),
                                               pickup_r,
                                               delivery_r);

              modified_with_pd.pop_back();

              if (valid) {
                best_cost = current_cost;
                best_job_rank = job_rank;
                best_pickup_r = pickup_r;
                best_delivery_r = delivery_r;
                best_modified_delivery = modified_delivery;
                best_eval = current_eval;
              }
            }
          }
        }
      }
    }

    if (best_cost < std::numeric_limits<double>::max()) {
      if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
        route.add(input, best_job_rank, best_r);
        unassigned.erase(best_job_rank);
        keep_going = true;
      }
      if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
        std::vector<Index> modified_with_pd;
        modified_with_pd.reserve(best_delivery_r - best_pickup_r + 2);
        modified_with_pd.push_back(best_job_rank);

        std::copy(route.route.begin() + best_pickup_r,
                  route.route.begin() + best_delivery_r,
                  std::back_inserter(modified_with_pd));
        modified_with_pd.push_back(best_job_rank + 1);

        route.replace(input,
                      best_modified_delivery,
                      modified_with_pd.begin(),
                      modified_with_pd.end(),
                      best_pickup_r,
                      best_delivery_r);
        unassigned.erase(best_job_rank);
        unassigned.erase(best_job_rank + 1);
        keep_going = true;
      }

      route_eval += best_eval;
    }
  }

  if (init_route_is_empty && !route.empty()) {
    // Account for fixed cost if we actually filled an empty route.
    route_eval.cost += vehicle.fixed_cost();
  }

  return route_eval;
}

template <class Route>
Eval basic(const Input& input,
           std::vector<Route>& routes,
           std::set<Index> unassigned,
           std::vector<Index> vehicles_ranks,
           INIT init,
           double lambda,
           SORT sort) {
  // Ordering is based on vehicles description only so do not account
  // for initial routes if any.
  const auto nb_vehicles = vehicles_ranks.size();

  switch (sort) {
  case SORT::AVAILABILITY: {
    // Sort vehicles by decreasing "availability".
    std::ranges::stable_sort(vehicles_ranks,
                             [&](const auto lhs, const auto rhs) {
                               return input.vehicles[lhs] < input.vehicles[rhs];
                             });
    break;
  }
  case SORT::COST:
    // Sort vehicles by increasing fixed cost, then same as above.
    std::ranges::stable_sort(vehicles_ranks,
                             [&](const auto lhs, const auto rhs) {
                               const auto& v_lhs = input.vehicles[lhs];
                               const auto& v_rhs = input.vehicles[rhs];
                               return v_lhs.costs < v_rhs.costs ||
                                      (v_lhs.costs == v_rhs.costs &&
                                       input.vehicles[lhs] <
                                         input.vehicles[rhs]);
                             });
    break;
  }

  const auto& evals = input.jobs_vehicles_evals();

  // regrets[v][j] holds the min cost for reaching job j in an empty
  // route across all remaining vehicles **after** vehicle at rank v
  // in vehicles_ranks. Regrets are only computed for available
  // vehicles and unassigned jobs, but are based on empty routes
  // evaluations so do not account for initial routes if any.
  std::vector<std::vector<Cost>> regrets(nb_vehicles,
                                         std::vector<Cost>(input.jobs.size()));

  // Use own cost for last vehicle regret values.
  for (const auto j : unassigned) {
    regrets.back()[j] = evals[j][vehicles_ranks.back()].cost;
  }

  for (Index rev_v = 0; rev_v < nb_vehicles - 1; ++rev_v) {
    // Going trough vehicles backward from second to last.
    const auto v = nb_vehicles - 2 - rev_v;

    bool all_compatible_jobs_later_undoable = true;
    for (const auto j : unassigned) {
      regrets[v][j] =
        std::min(regrets[v + 1][j], (evals[j][vehicles_ranks[v + 1]]).cost);
      if (input.vehicle_ok_with_job(vehicles_ranks[v], j) &&
          regrets[v][j] < input.get_cost_upper_bound()) {
        all_compatible_jobs_later_undoable = false;
      }
    }

    if (all_compatible_jobs_later_undoable) {
      // We don't want to use all regrets equal to the cost upper
      // bound in this situation: it would defeat the purpose of using
      // regrets in the first place as all lambda values would yield
      // the same choices. Using the same approach as with last
      // vehicle.
      for (const auto j : unassigned) {
        regrets[v][j] = evals[j][vehicles_ranks[v]].cost;
      }
    }
  }

  Eval sol_eval;

  for (Index v = 0; v < nb_vehicles && !unassigned.empty(); ++v) {
    auto v_rank = vehicles_ranks[v];
    auto& current_r = routes[v_rank];

    if (current_r.empty() && init != INIT::NONE) {
      // Trivial lambda for no additional job validity constraint.
      constexpr auto job_not_ok = [](const Index) { return false; };

      seed_route(input, current_r, init, evals, unassigned, job_not_ok);
    }

    const auto current_eval =
      fill_route(input, current_r, unassigned, regrets[v], lambda);
    sol_eval += current_eval;
  }

  return sol_eval;
}

template <class Route>
Eval dynamic_vehicle_choice(const Input& input,
                            std::vector<Route>& routes,
                            std::set<Index> unassigned,
                            std::vector<Index> vehicles_ranks,
                            INIT init,
                            double lambda,
                            SORT sort) {
  const auto& evals = input.jobs_vehicles_evals();

  Eval sol_eval;

  while (!vehicles_ranks.empty() && !unassigned.empty()) {
    // For any unassigned job at j, jobs_min_costs[j]
    // (resp. jobs_second_min_costs[j]) holds the min cost
    // (resp. second min cost) of picking the job in an empty route
    // for any remaining vehicle. Evaluation are based on empty routes
    // so do not account for initial routes if any.
    std::vector<Cost> jobs_min_costs(input.jobs.size(),
                                     input.get_cost_upper_bound());
    std::vector<Cost> jobs_second_min_costs(input.jobs.size(),
                                            input.get_cost_upper_bound());
    for (const auto j : unassigned) {
      for (const auto v : vehicles_ranks) {
        if (evals[j][v].cost <= jobs_min_costs[j]) {
          jobs_second_min_costs[j] = jobs_min_costs[j];
          jobs_min_costs[j] = evals[j][v].cost;
        } else {
          if (evals[j][v].cost < jobs_second_min_costs[j]) {
            jobs_second_min_costs[j] = evals[j][v].cost;
          }
        }
      }
    }

    // Pick vehicle that has the biggest number of compatible
    // unassigned jobs closest to him than to any other different
    // vehicle still available.
    std::vector<unsigned> closest_jobs_count(input.vehicles.size(), 0);
    for (const auto j : unassigned) {
      for (const auto v : vehicles_ranks) {
        if (evals[j][v].cost == jobs_min_costs[j]) {
          ++closest_jobs_count[v];
        }
      }
    }

    Index v_rank;

    if (sort == SORT::AVAILABILITY) {
      const auto chosen_vehicle =
        std::ranges::min_element(vehicles_ranks,
                                 [&](const auto lhs, const auto rhs) {
                                   return closest_jobs_count[lhs] >
                                            closest_jobs_count[rhs] ||
                                          (closest_jobs_count[lhs] ==
                                             closest_jobs_count[rhs] &&
                                           input.vehicles[lhs] <
                                             input.vehicles[rhs]);
                                 });
      v_rank = *chosen_vehicle;
      vehicles_ranks.erase(chosen_vehicle);
    } else {
      assert(sort == SORT::COST);

      const auto chosen_vehicle =
        std::ranges::min_element(vehicles_ranks,
                                 [&](const auto lhs, const auto rhs) {
                                   const auto& v_lhs = input.vehicles[lhs];
                                   const auto& v_rhs = input.vehicles[rhs];
                                   return closest_jobs_count[lhs] >
                                            closest_jobs_count[rhs] ||
                                          (closest_jobs_count[lhs] ==
                                             closest_jobs_count[rhs] &&
                                           (v_lhs.costs < v_rhs.costs ||
                                            (v_lhs.costs == v_rhs.costs &&
                                             v_lhs < v_rhs)));
                                 });
      v_rank = *chosen_vehicle;
      vehicles_ranks.erase(chosen_vehicle);
    }

    // Once current vehicle is decided, then for any unassigned job at
    // j, regrets[j] holds the min cost of picking the job in an empty
    // route for other remaining vehicles. Regrets are only computed
    // for available vehicles and unassigned jobs, but are based on
    // empty routes evaluations so do not account for initial routes
    // if any.
    std::vector<Cost> regrets(input.jobs.size(), input.get_cost_upper_bound());

    bool all_compatible_jobs_later_undoable = true;
    for (const auto j : unassigned) {
      if (jobs_min_costs[j] < evals[j][v_rank].cost) {
        regrets[j] = jobs_min_costs[j];
      } else {
        regrets[j] = jobs_second_min_costs[j];
      }

      if (input.vehicle_ok_with_job(v_rank, j) &&
          regrets[j] < input.get_cost_upper_bound()) {
        all_compatible_jobs_later_undoable = false;
      }
    }

    if (all_compatible_jobs_later_undoable) {
      // Same approach as for basic heuristic.
      for (const auto j : unassigned) {
        regrets[j] = evals[j][v_rank].cost;
      }
    }

    auto& current_r = routes[v_rank];

    if (current_r.empty() && init != INIT::NONE) {
      auto job_not_ok =
        [&jobs_min_costs, &evals, v_rank](const Index job_rank) {
          // One of the remaining vehicles is closest to that job.
          return jobs_min_costs[job_rank] < evals[job_rank][v_rank].cost;
        };

      seed_route(input, current_r, init, evals, unassigned, job_not_ok);
    }

    const auto current_eval =
      fill_route(input, current_r, unassigned, regrets, lambda);
    sol_eval += current_eval;
  }

  return sol_eval;
}

template <class Route>
void set_route(const Input& input,
               Route& route,
               std::unordered_set<Index>& assigned) {
  assert(route.empty());
  const auto& vehicle = input.vehicles[route.vehicle_rank];

  // Startup load is the sum of deliveries for (single) jobs.
  Amount single_jobs_deliveries(input.zero_amount());
  for (const auto& step : vehicle.steps) {
    if (step.type == STEP_TYPE::JOB) {
      assert(step.job_type.has_value());

      if (step.job_type.value() == JOB_TYPE::SINGLE) {
        single_jobs_deliveries += input.jobs[step.rank].delivery;
      }
    }
  }
  if (!(single_jobs_deliveries <= vehicle.capacity)) {
    throw InputException(
      std::format("Route over capacity for vehicle {}.", vehicle.id));
  }

  // Track load and travel time during the route for validity.
  Amount current_load = single_jobs_deliveries;
  Eval eval_sum;
  std::optional<Index> previous_index;
  if (vehicle.has_start()) {
    previous_index = vehicle.start.value().index();
  }

  std::vector<Index> job_ranks;
  job_ranks.reserve(vehicle.steps.size());
  std::unordered_set<Index> expected_delivery_ranks;
  for (const auto& step : vehicle.steps) {
    if (step.type != STEP_TYPE::JOB) {
      continue;
    }

    const auto job_rank = step.rank;
    const auto& job = input.jobs[job_rank];
    job_ranks.push_back(job_rank);

    assert(!assigned.contains(job_rank));
    assigned.insert(job_rank);

    if (!input.vehicle_ok_with_job(route.vehicle_rank, job_rank)) {
      throw InputException(
        std::format("Missing skill or step out of reach for vehicle {} and "
                    "job {}.",
                    vehicle.id,
                    job.id));
    }

    // Update current travel time.
    if (previous_index.has_value()) {
      eval_sum += vehicle.eval(previous_index.value(), job.index());
    }
    previous_index = job.index();

    // Handle load.
    assert(step.job_type.has_value());
    switch (step.job_type.value()) {
    case JOB_TYPE::SINGLE: {
      current_load += job.pickup;
      current_load -= job.delivery;
      break;
    }
    case JOB_TYPE::PICKUP: {
      expected_delivery_ranks.insert(job_rank + 1);

      current_load += job.pickup;
      break;
    }
    case JOB_TYPE::DELIVERY: {
      auto search = expected_delivery_ranks.find(job_rank);
      if (search == expected_delivery_ranks.end()) {
        throw InputException(
          std::format("Invalid shipment in route for vehicle {}.", vehicle.id));
      }
      expected_delivery_ranks.erase(search);

      current_load -= job.delivery;
      break;
    }
    default:
      assert(false);
    }

    // Check validity after this step wrt capacity.
    if (!(current_load <= vehicle.capacity)) {
      throw InputException(
        std::format("Route over capacity for vehicle {}.", vehicle.id));
    }
  }

  if (vehicle.has_end() && !job_ranks.empty()) {
    // Update with last route leg.
    assert(previous_index.has_value());
    eval_sum +=
      vehicle.eval(previous_index.value(), vehicle.end.value().index());
  }
  if (!vehicle.ok_for_travel_time(eval_sum.duration)) {
    throw InputException(
      std::format("Route over max_travel_time for vehicle {}.", vehicle.id));
  }
  if (!vehicle.ok_for_distance(eval_sum.distance)) {
    throw InputException(
      std::format("Route over max_distance for vehicle {}.", vehicle.id));
  }

  if (vehicle.max_tasks < job_ranks.size()) {
    throw InputException(
      std::format("Too many tasks for vehicle {}.", vehicle.id));
  }

  if (!expected_delivery_ranks.empty()) {
    throw InputException(
      std::format("Invalid shipment in route for vehicle {}.", vehicle.id));
  }

  // Now route is OK with regard to capacity, max_travel_time,
  // max_tasks, precedence and skills constraints.
  if (!job_ranks.empty()) {
    if (!route.is_valid_addition_for_tw(input,
                                        single_jobs_deliveries,
                                        job_ranks.begin(),
                                        job_ranks.end(),
                                        0,
                                        0)) {
      throw InputException(
        std::format("Infeasible route for vehicle {}.", vehicle.id));
    }

    route.replace(input,
                  single_jobs_deliveries,
                  job_ranks.begin(),
                  job_ranks.end(),
                  0,
                  0);
  }
}

template <class Route>
std::unordered_set<Index> set_initial_routes(const Input& input,
                                             std::vector<Route>& routes) {
  std::unordered_set<Index> assigned;

  std::ranges::for_each(routes,
                        [&](auto& r) { set_route(input, r, assigned); });

  return assigned;
}

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

template Eval basic(const Input& input,
                    RawSolution& routes,
                    std::set<Index> unassigned,
                    std::vector<Index> vehicles_ranks,
                    INIT init,
                    double lambda,
                    SORT sort);

template Eval dynamic_vehicle_choice(const Input& input,
                                     RawSolution& routes,
                                     std::set<Index> unassigned,
                                     std::vector<Index> vehicles_ranks,
                                     INIT init,
                                     double lambda,
                                     SORT sort);

template std::unordered_set<Index> set_initial_routes(const Input& input,
                                                      RawSolution& routes);

template Eval basic(const Input& input,
                    TWSolution& routes,
                    std::set<Index> unassigned,
                    std::vector<Index> vehicles_ranks,
                    INIT init,
                    double lambda,
                    SORT sort);

template Eval dynamic_vehicle_choice(const Input& input,
                                     TWSolution& routes,
                                     std::set<Index> unassigned,
                                     std::vector<Index> vehicles_ranks,
                                     INIT init,
                                     double lambda,
                                     SORT sort);

template std::unordered_set<Index> set_initial_routes(const Input& input,
                                                      TWSolution& routes);

} // namespace vroom::heuristics
