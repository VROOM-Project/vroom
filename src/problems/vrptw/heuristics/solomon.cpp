/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>
#include <set>

#include "problems/vrptw/heuristics/solomon.h"
#include "utils/helpers.h"

tw_solution homogeneous_solomon(const input& input, INIT_T init, float lambda) {
  // TODO
  // assert(input._homogeneous_locations and input._vehicles.size() > 0);
  tw_solution routes;

  std::set<index_t> unassigned;
  for (index_t j = 0; j < input._jobs.size(); ++j) {
    unassigned.insert(j);
  }

  // One level of indirection to allow easy ordering of the vehicles
  // within the heuristic.
  std::vector<index_t> vehicles_ranks(input._vehicles.size());
  std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);
  // Sort vehicles by "higher" capacity or by time window in case of
  // capacities ties.
  std::sort(vehicles_ranks.begin(),
            vehicles_ranks.end(),
            [&](const auto lhs, const auto rhs) {
              auto& v_lhs = input._vehicles[lhs];
              auto& v_rhs = input._vehicles[rhs];
              return (v_rhs.capacity << v_lhs.capacity) or
                     (v_lhs.capacity == v_rhs.capacity and
                      v_lhs.tw.length > v_rhs.tw.length);
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
    routes.emplace_back(input, vehicles_ranks[v]);
    auto& tw_r = routes.back();

    const auto& vehicle = input._vehicles[vehicles_ranks[v]];

    amount_t route_amount(input.amount_size());

    if (init != INIT_T::NONE) {
      // Initialize current route with the "best" valid job.
      amount_t higher_amount(input.amount_size());
      cost_t furthest_cost = 0;
      duration_t earliest_deadline = std::numeric_limits<duration_t>::max();
      index_t best_job_rank = 0;
      for (const auto& job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v, job_rank) or
            !(input._jobs[job_rank].amount <= vehicle.capacity) or
            !tw_r.is_valid_addition_for_tw(job_rank, 0)) {
          continue;
        }

        if (init == INIT_T::HIGHER_AMOUNT and
            higher_amount << input._jobs[job_rank].amount) {
          higher_amount = input._jobs[job_rank].amount;
          best_job_rank = job_rank;
        }
        if (init == INIT_T::EARLIEST_DEADLINE) {
          duration_t current_deadline = input._jobs[job_rank].tws.back().end;
          if (current_deadline < earliest_deadline) {
            earliest_deadline = current_deadline;
            best_job_rank = job_rank;
          }
        }
        if (init == INIT_T::FURTHEST and furthest_cost < costs[job_rank]) {
          furthest_cost = costs[job_rank];
          best_job_rank = job_rank;
        }
      }
      if ((init == INIT_T::HIGHER_AMOUNT and route_amount << higher_amount) or
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
            !(route_amount + input._jobs[job_rank].amount <=
              vehicle.capacity)) {
          continue;
        }

        for (index_t r = 0; r <= tw_r.route.size(); ++r) {
          float current_add =
            addition_cost(input, m, job_rank, vehicle, tw_r.route, r);

          float current_cost =
            current_add - lambda * static_cast<float>(costs[job_rank]);

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
