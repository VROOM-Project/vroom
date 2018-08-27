/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_set>

#include "problems/vrptw/heuristics/best_insertion.h"
#include "utils/helpers.h"

tw_solution best_insertion(const input& input) {
  tw_solution routes;
  for (index_t i = 0; i < input._vehicles.size(); ++i) {
    routes.emplace_back(input, i);
  }

  std::unordered_set<index_t> unassigned;
  for (index_t j = 0; j < input._jobs.size(); ++j) {
    unassigned.insert(j);
  }

  for (index_t v = 0; v < input._vehicles.size(); ++v) {
    auto& tw_r = routes[v];
    const auto& vehicle = input._vehicles[v];

    amount_t route_amount(input.amount_size());

    bool keep_going = true;
    while (keep_going) {
      keep_going = false;
      gain_t best_cost = std::numeric_limits<gain_t>::max();
      index_t best_job_rank = 0;
      index_t best_r = 0;

      for (const auto& job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v, job_rank) or
            vehicle.capacity < route_amount + input._jobs[job_rank].amount) {
          continue;
        }

        for (index_t r = 0; r <= tw_r.route.size(); ++r) {
          gain_t current_cost = addition_cost(input,
                                              input.get_matrix(),
                                              job_rank,
                                              vehicle,
                                              tw_r.route,
                                              r);

          if (current_cost < best_cost and
              tw_r.is_valid_addition_for_tw(job_rank, r)) {
            best_cost = current_cost;
            best_job_rank = job_rank;
            best_r = r;
          }
        }
      }

      if (best_cost < std::numeric_limits<gain_t>::max()) {
        tw_r.add(best_job_rank, best_r);
        route_amount += input._jobs[best_job_rank].amount;
        unassigned.erase(best_job_rank);
        keep_going = true;
      }
    }
  }

  return routes;
}
