/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "clustering.h"

std::vector<std::vector<index_t>> clustering(const input& input) {
  // Test cutting the problem in several TSP (dumbly adding jobs in
  // order for now).
  std::vector<std::vector<index_t>> clusters;

  unsigned j = 0;

  for (unsigned i = 0; i < input._vehicles.size(); ++i) {
    clusters.push_back({});
    auto& current_cluster = clusters.back();

    auto& vehicle_capacity = input._vehicles[i].capacity.get();

    amount_t current_amount(vehicle_capacity.size());
    if (input._vehicles[i].has_start()) {
      auto start_index = input._vehicles[i].start.get().index();
      current_cluster.push_back(start_index);

      if (input._vehicles[i].has_end()) {
        auto end_index = input._vehicles[i].end.get().index();
        if (start_index != end_index) {
          current_cluster.push_back(input._vehicles[i].end.get().index());
        }
      }
    } else {
      assert(input._vehicles[i].has_end());
      current_cluster.push_back(input._vehicles[i].end.get().index());
    }

    while ((current_amount < vehicle_capacity) and (j < input._jobs.size())) {
      auto next_amount = current_amount + input._jobs[j].amount.get();
      if (next_amount <= vehicle_capacity) {
        current_cluster.push_back(input._jobs[j].index());
        current_amount = next_amount;
        ++j;
      } else {
        break;
      }
    }
  }

  return clusters;
}
