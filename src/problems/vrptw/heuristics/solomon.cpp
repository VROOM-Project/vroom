/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_set>

#include "problems/vrptw/heuristics/solomon.h"
#include "utils/helpers.h"

tw_solution solomon(const input& input, INIT_T init, float lambda) {
  tw_solution routes;
  for (index_t i = 0; i < input._vehicles.size(); ++i) {
    routes.emplace_back(input, i);
  }

  std::unordered_set<index_t> unassigned;
  for (index_t j = 0; j < input._jobs.size(); ++j) {
    unassigned.insert(j);
  }

  const auto& m = input.get_matrix();

  // costs[j][v] is the cost of fetching job j in an empty route from
  // vehicle v. regrets[j][v] is the minimum cost of fetching job j in
  // an empty route from any vehicle after v.
  std::vector<std::vector<cost_t>> costs(input._jobs.size(),
                                         std::vector<cost_t>(
                                           input._vehicles.size()));
  std::vector<std::vector<cost_t>> regrets(input._jobs.size(),
                                           std::vector<cost_t>(
                                             input._vehicles.size()));
  for (std::size_t j = 0; j < input._jobs.size(); ++j) {
    index_t j_index = input._jobs[j].index();

    regrets[j].back() = INFINITE_COST;

    for (std::size_t v = input._vehicles.size() - 1; v > 0; --v) {
      const auto& vehicle = input._vehicles[v];
      cost_t current_cost = 0;
      if (vehicle.has_start()) {
        current_cost += m[vehicle.start.get().index()][j_index];
      }
      if (vehicle.has_end()) {
        current_cost += m[j_index][vehicle.end.get().index()];
      }
      costs[j][v] = current_cost;
      regrets[j][v - 1] = std::min(regrets[j][v], current_cost);
    }

    const auto& vehicle = input._vehicles[0];
    cost_t current_cost = 0;
    if (vehicle.has_start()) {
      current_cost += m[vehicle.start.get().index()][j_index];
    }
    if (vehicle.has_end()) {
      current_cost += m[j_index][vehicle.end.get().index()];
    }
    costs[j][0] = current_cost;
  }

  for (index_t v = 0; v < input._vehicles.size(); ++v) {
    auto& tw_r = routes[v];
    const auto& vehicle = input._vehicles[v];

    amount_t route_amount(input.amount_size());

    if (init != INIT_T::NONE) {
      // Initialize current route with the "best" valid job that is
      //  closest for current vehicle than to any other remaining
      //  vehicle.
      amount_t higher_amount(input.amount_size());
      cost_t furthest_cost = 0;
      duration_t earliest_deadline = std::numeric_limits<duration_t>::max();
      index_t best_job_rank = 0;
      for (const auto& job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v, job_rank) or
            vehicle.capacity < input._jobs[job_rank].amount) {
          continue;
        }

        auto current_cost =
          addition_cost(input, m, job_rank, vehicle, tw_r.route, 0);
        if (regrets[job_rank][v] < current_cost) {
          continue;
        }

        if (init == INIT_T::HIGHER_AMOUNT and
            higher_amount < input._jobs[job_rank].amount and
            tw_r.is_valid_addition_for_tw(job_rank, 0)) {
          higher_amount = input._jobs[job_rank].amount;
          best_job_rank = job_rank;
        }
        if (init == INIT_T::EARLIEST_DEADLINE) {
          duration_t current_deadline = input._jobs[job_rank].tws.back().end;
          if (current_deadline < earliest_deadline and
              tw_r.is_valid_addition_for_tw(job_rank, 0)) {
            earliest_deadline = current_deadline;
            best_job_rank = job_rank;
          }
        }
        if (init == INIT_T::FURTHEST and furthest_cost < current_cost and
            tw_r.is_valid_addition_for_tw(job_rank, 0)) {
          furthest_cost = current_cost;
          best_job_rank = job_rank;
        }
      }
      if ((init == INIT_T::HIGHER_AMOUNT and route_amount < higher_amount) or
          (init == INIT_T::EARLIEST_DEADLINE and
           earliest_deadline < std::numeric_limits<duration_t>::max()) or
          (init == INIT_T::FURTHEST and furthest_cost > 0)) {
        tw_r.add(best_job_rank, 0);
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

      for (const auto& job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v, job_rank) or
            vehicle.capacity < route_amount + input._jobs[job_rank].amount) {
          continue;
        }

        for (index_t r = 0; r <= tw_r.route.size(); ++r) {
          float current_add =
            addition_cost(input, m, job_rank, vehicle, tw_r.route, r);

          float current_cost =
            current_add - lambda * static_cast<float>(regrets[job_rank][v]);

          if (current_cost < best_cost and
              tw_r.is_valid_addition_for_tw(job_rank, r)) {
            best_cost = current_cost;
            best_job_rank = job_rank;
            best_r = r;
          }
        }
      }

      if (best_cost < std::numeric_limits<float>::max()) {
        tw_r.add(best_job_rank, best_r);
        route_amount += input._jobs[best_job_rank].amount;
        unassigned.erase(best_job_rank);
        keep_going = true;
      }
    }
  }

  return routes;
}
