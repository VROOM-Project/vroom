/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>
#include <set>

#include "problems/cvrp/heuristics/solomon.h"
#include "utils/helpers.h"

raw_solution cvrp_basic_heuristic(const input& input,
                                  INIT_T init,
                                  float lambda) {
  raw_solution routes(input._vehicles.size());

  std::set<index_t> unassigned;
  for (index_t j = 0; j < input._jobs.size(); ++j) {
    unassigned.insert(j);
  }

  // One level of indirection to allow easy ordering of the vehicles
  // within the heuristic.
  std::vector<index_t> vehicles_ranks(input._vehicles.size());
  std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);
  // Sort vehicles by "higher" capacity.
  std::stable_sort(vehicles_ranks.begin(),
                   vehicles_ranks.end(),
                   [&](const auto lhs, const auto rhs) {
                     return input._vehicles[rhs].capacity
                            << input._vehicles[lhs].capacity;
                   });

  const auto& m = input.get_matrix();

  // costs[j] is the cost of fetching job j in an empty route from one
  // of the vehicles (consistent across vehicles in the homogeneous
  // case).
  const auto& v = input._vehicles[0];

  std::vector<cost_t> costs(input._jobs.size());
  for (std::size_t j = 0; j < input._jobs.size(); ++j) {
    index_t j_index = input._jobs[j].index();

    cost_t current_cost = 0;
    if (v.has_start()) {
      current_cost += m[v.start.get().index()][j_index];
    }
    if (v.has_end()) {
      current_cost += m[j_index][v.end.get().index()];
    }
    costs[j] = current_cost;
  }

  for (index_t v = 0; v < input._vehicles.size(); ++v) {
    auto v_rank = vehicles_ranks[v];
    auto& route = routes[v_rank];

    const auto& vehicle = input._vehicles[v_rank];

    amount_t route_amount(input.amount_size());

    if (init != INIT_T::NONE) {
      // Initialize current route with the "best" valid job.
      amount_t higher_amount(input.amount_size());
      cost_t furthest_cost = 0;
      index_t best_job_rank = 0;
      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank) or
            !(input._jobs[job_rank].amount <= vehicle.capacity)) {
          continue;
        }

        if (init == INIT_T::HIGHER_AMOUNT and
            higher_amount << input._jobs[job_rank].amount) {
          higher_amount = input._jobs[job_rank].amount;
          best_job_rank = job_rank;
        }
        if (init == INIT_T::FURTHEST and furthest_cost < costs[job_rank]) {
          furthest_cost = costs[job_rank];
          best_job_rank = job_rank;
        }
      }
      if ((init == INIT_T::HIGHER_AMOUNT and route_amount << higher_amount) or
          (init == INIT_T::FURTHEST and furthest_cost > 0)) {
        route.insert(route.begin(), best_job_rank);
        route_amount += input._jobs[best_job_rank].amount;
        unassigned.erase(best_job_rank);
      }
    }

    bool keep_going = true;
    while (keep_going) {
      keep_going = false;
      float best_cost = std::numeric_limits<float>::max();
      index_t best_job_rank = 0;
      index_t best_r = 0;

      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank) or
            !(route_amount + input._jobs[job_rank].amount <=
              vehicle.capacity)) {
          continue;
        }

        for (index_t r = 0; r <= route.size(); ++r) {
          float current_add =
            addition_cost(input, m, job_rank, vehicle, route, r);

          float current_cost =
            current_add - lambda * static_cast<float>(costs[job_rank]);

          if (current_cost < best_cost) {
            best_cost = current_cost;
            best_job_rank = job_rank;
            best_r = r;
          }
        }
      }

      if (best_cost < std::numeric_limits<float>::max()) {
        route.insert(route.begin() + best_r, best_job_rank);
        route_amount += input._jobs[best_job_rank].amount;
        unassigned.erase(best_job_rank);
        keep_going = true;
      }
    }
  }

  return routes;
}

raw_solution cvrp_dynamic_vehicle_choice_heuristic(const input& input,
                                                   INIT_T init,
                                                   float lambda) {
  raw_solution routes(input._vehicles.size());

  std::set<index_t> unassigned;
  for (index_t j = 0; j < input._jobs.size(); ++j) {
    unassigned.insert(j);
  }

  std::vector<index_t> vehicles_ranks(input._vehicles.size());
  std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);

  const auto& m = input.get_matrix();

  // costs[j][v] is the cost of fetching job j in an empty route from
  // vehicle at vehicles_ranks[v].
  std::vector<std::vector<cost_t>> costs(input._jobs.size(),
                                         std::vector<cost_t>(
                                           input._vehicles.size()));
  for (std::size_t j = 0; j < input._jobs.size(); ++j) {
    index_t j_index = input._jobs[j].index();

    for (std::size_t v = 0; v < vehicles_ranks.size(); ++v) {
      const auto& vehicle = input._vehicles[vehicles_ranks[v]];
      cost_t current_cost = 0;
      if (vehicle.has_start()) {
        current_cost += m[vehicle.start.get().index()][j_index];
      }
      if (vehicle.has_end()) {
        current_cost += m[j_index][vehicle.end.get().index()];
      }
      costs[j][v] = current_cost;
    }
  }

  while (!vehicles_ranks.empty() and !unassigned.empty()) {
    // For any unassigned job at j, jobs_min_costs[j]
    // (resp. jobs_second_min_costs[j]) holds the min cost
    // (resp. second min cost) of picking the job in an empty route
    // for any remaining vehicle.
    std::vector<cost_t> jobs_min_costs(input._jobs.size(),
                                       std::numeric_limits<cost_t>::max());
    std::vector<cost_t>
      jobs_second_min_costs(input._jobs.size(),
                            std::numeric_limits<cost_t>::max());
    for (const auto job_rank : unassigned) {
      for (const auto v_rank : vehicles_ranks) {
        if (costs[job_rank][v_rank] <= jobs_min_costs[job_rank]) {
          jobs_second_min_costs[job_rank] = jobs_min_costs[job_rank];
          jobs_min_costs[job_rank] = costs[job_rank][v_rank];
        } else {
          if (costs[job_rank][v_rank] < jobs_second_min_costs[job_rank]) {
            jobs_second_min_costs[job_rank] = costs[job_rank][v_rank];
          }
        }
      }
    }

    // Pick vehicle that has the biggest number of compatible jobs
    // closest to him than to any other different vehicle.
    std::vector<unsigned> closest_jobs_count(input._vehicles.size(), 0);
    for (const auto job_rank : unassigned) {
      for (const auto v_rank : vehicles_ranks) {
        if (costs[job_rank][v_rank] == jobs_min_costs[job_rank]) {
          ++closest_jobs_count[v_rank];
        }
      }
    }

    const auto chosen_vehicle =
      std::min_element(vehicles_ranks.begin(),
                       vehicles_ranks.end(),
                       [&](const auto lhs, const auto rhs) {
                         return closest_jobs_count[lhs] >
                                  closest_jobs_count[rhs] or
                                (closest_jobs_count[lhs] ==
                                   closest_jobs_count[rhs] and
                                 (input._vehicles[rhs].capacity
                                  << input._vehicles[lhs].capacity));
                       });
    auto v_rank = *chosen_vehicle;
    vehicles_ranks.erase(chosen_vehicle);

    // Once current vehicle is decided, regrets[j] holds the min cost
    // of picking the job in an empty route for other remaining
    // vehicles.
    std::vector<cost_t> regrets(input._jobs.size(),
                                std::numeric_limits<cost_t>::max());
    for (const auto job_rank : unassigned) {
      if (jobs_min_costs[job_rank] < costs[job_rank][v_rank]) {
        regrets[job_rank] = jobs_min_costs[job_rank];
      } else {
        regrets[job_rank] = jobs_second_min_costs[job_rank];
      }
    }

    const auto& vehicle = input._vehicles[v_rank];
    auto& route = routes[v_rank];

    amount_t route_amount(input.amount_size());

    if (init != INIT_T::NONE) {
      // Initialize current route with the "best" valid job that is
      //  closest for current vehicle than to any other remaining
      //  vehicle.
      amount_t higher_amount(input.amount_size());
      cost_t furthest_cost = 0;
      index_t best_job_rank = 0;
      for (const auto job_rank : unassigned) {
        if (jobs_min_costs[job_rank] < costs[job_rank][v_rank] or
            // One of the remaining vehicles is closest to that job.
            !input.vehicle_ok_with_job(v_rank, job_rank) or
            !(input._jobs[job_rank].amount <= vehicle.capacity)) {
          continue;
        }

        if (init == INIT_T::HIGHER_AMOUNT and
            higher_amount << input._jobs[job_rank].amount) {
          higher_amount = input._jobs[job_rank].amount;
          best_job_rank = job_rank;
        }
        if (init == INIT_T::FURTHEST and
            furthest_cost < costs[job_rank][v_rank]) {
          furthest_cost = costs[job_rank][v_rank];
          best_job_rank = job_rank;
        }
      }
      if ((init == INIT_T::HIGHER_AMOUNT and route_amount << higher_amount) or
          (init == INIT_T::FURTHEST and furthest_cost > 0)) {
        route.insert(route.begin(), best_job_rank);
        route_amount += input._jobs[best_job_rank].amount;
        unassigned.erase(best_job_rank);
      }
    }

    bool keep_going = true;
    while (keep_going) {
      keep_going = false;
      float best_cost = std::numeric_limits<float>::max();
      index_t best_job_rank = 0;
      index_t best_r = 0;

      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank) or
            !(route_amount + input._jobs[job_rank].amount <=
              vehicle.capacity)) {
          continue;
        }

        for (index_t r = 0; r <= route.size(); ++r) {
          float current_add =
            addition_cost(input, m, job_rank, vehicle, route, r);

          float current_cost =
            current_add - lambda * static_cast<float>(regrets[job_rank]);

          if (current_cost < best_cost) {
            best_cost = current_cost;
            best_job_rank = job_rank;
            best_r = r;
          }
        }
      }

      if (best_cost < std::numeric_limits<float>::max()) {
        route.insert(route.begin() + best_r, best_job_rank);
        route_amount += input._jobs[best_job_rank].amount;
        unassigned.erase(best_job_rank);
        keep_going = true;
      }
    }
  }

  return routes;
}
