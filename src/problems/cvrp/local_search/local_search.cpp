/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "cross_exchange.h"
#include "exchange.h"
#include "local_search.h"
#include "or_opt.h"
#include "relocate.h"

cvrp_local_search::cvrp_local_search(const input& input, raw_solution& sol)
  : _input(input),
    _sol(sol),
    _amounts(sol.size(), amount_t(input.amount_size())) {

  std::cout << "Amount lower bound: ";
  auto amount_lower_bound = _input.get_amount_lower_bound();
  for (std::size_t r = 0; r < amount_lower_bound.size(); ++r) {
    std::cout << amount_lower_bound[r];
  }
  std::cout << std::endl;

  for (std::size_t i = 0; i < sol.size(); ++i) {
    for (const auto rank : sol[i]) {
      _amounts[i] += _input._jobs[rank].amount;
    }
    auto& capacity = _input._vehicles[i].capacity;

    std::cout << "Amount for vehicle at rank " << i << ": ";
    for (std::size_t r = 0; r < _amounts[i].size(); ++r) {
      std::cout << _amounts[i][r] << " / " << capacity[r] << " ; ";
    }
    std::cout << std::endl;
  }
}

void cvrp_local_search::run() {
  std::cout << "Running CVRP local search." << std::endl;

  auto amount_lower_bound = _input.get_amount_lower_bound();
  auto double_amount_lower_bound = amount_lower_bound + amount_lower_bound;

  auto V = _input._vehicles.size();
  std::vector<std::vector<std::unique_ptr<ls_operator>>> best_ops(V);
  for (std::size_t v = 0; v < V; ++v) {
    best_ops[v] = std::vector<std::unique_ptr<ls_operator>>(V);
  }

  // List of source/target pairs we need to test (all at first).
  std::vector<std::pair<index_t, index_t>> s_t_pairs;
  for (unsigned s_v = 0; s_v < _sol.size(); ++s_v) {
    for (unsigned t_v = 0; t_v < _sol.size(); ++t_v) {
      if (s_v == t_v) {
        continue;
      }
      s_t_pairs.emplace_back(s_v, t_v);
    }
  }

  std::vector<std::vector<gain_t>> best_gains(V, std::vector<gain_t>(V, 0));

  unsigned ls_step = 0;
  // write_to_json(_input.format_solution(_sol),
  //               false,
  //               "ls_log_" + std::to_string(ls_step) + "_sol.json");

  gain_t best_gain = 1;

  while (best_gain > 0) {
    ++ls_step;

    // Relocate stuff
    for (const auto& s_t : s_t_pairs) {
      if (_input._vehicles[s_t.second].capacity <
          _amounts[s_t.second] + amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() == 0) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
          relocate
            r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
          if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<relocate>(r);
          }
        }
      }
    }

    // Exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first) {
        // This operator is symmetric.
        continue;
      }
      if ((_sol[s_t.first].size() == 0) or (_sol[s_t.second].size() == 0)) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size(); ++t_rank) {
          exchange
            r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
          if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<exchange>(r);
          }
        }
      }
    }

    // Or-opt stuff
    for (const auto& s_t : s_t_pairs) {
      if (_input._vehicles[s_t.second].capacity <
          _amounts[s_t.second] + double_amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() < 2) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
          or_opt
            r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
          if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<or_opt>(r);
          }
        }
      }
    }

    // CROSS-exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first) {
        // This operator is symmetric.
        continue;
      }
      if ((_sol[s_t.first].size() < 2) or (_sol[s_t.second].size() < 2)) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size() - 1;
             ++t_rank) {
          cross_exchange
            r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
          if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<cross_exchange>(r);
          }
        }
      }
    }

    // Find best overall gain.
    best_gain = 0;
    index_t best_source = 0;
    index_t best_target = 0;

    for (unsigned s_v = 0; s_v < _sol.size(); ++s_v) {
      for (unsigned t_v = 0; t_v < _sol.size(); ++t_v) {
        if (s_v == t_v) {
          continue;
        }
        if (best_gains[s_v][t_v] > best_gain) {
          best_gain = best_gains[s_v][t_v];
          best_source = s_v;
          best_target = t_v;
        }
      }
    }

    // Apply matching operator.
    if (best_gain > 0) {
      std::cout << "* Best operator choice " << std::endl;

      assert(best_ops[best_source][best_target] != nullptr);

      best_ops[best_source][best_target]->log();

      best_ops[best_source][best_target]->apply();

      // write_to_json(_input.format_solution(_sol),
      //               false,
      //               "ls_log_" + std::to_string(ls_step) + "_sol.json");

      // Set gains to zero for what needs to be recomputed in the next round.
      s_t_pairs.clear();
      best_gains[best_source] = std::vector<gain_t>(V, 0);
      best_gains[best_target] = std::vector<gain_t>(V, 0);

      for (unsigned v = 0; v < _sol.size(); ++v) {
        if (v != best_source) {
          s_t_pairs.emplace_back(best_source, v);
          s_t_pairs.emplace_back(v, best_source);
          best_gains[v][best_source] = 0;
          best_gains[best_source][v] = 0;
        }
        if (v != best_target) {
          s_t_pairs.emplace_back(best_target, v);
          s_t_pairs.emplace_back(v, best_target);
          best_gains[v][best_target] = 0;
          best_gains[best_target][v] = 0;
        }
      }
    }
  }
}
