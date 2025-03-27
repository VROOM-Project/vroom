/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <iterator>
#include <numeric>
#include <ranges>
#include <thread>
#include <unordered_map>

#include "problems/tsp/heuristics/local_search.h"
#include "utils/helpers.h"

namespace vroom::tsp {

LocalSearch::LocalSearch(const Matrix<UserCost>& matrix,
                         std::pair<bool, Index> avoid_start_relocate,
                         const std::list<Index>& tour,
                         unsigned nb_threads)
  : _matrix(matrix),
    _avoid_start_relocate(std::move(avoid_start_relocate)),
    _edges(_matrix.size()),
    _nb_threads(std::min(nb_threads, static_cast<unsigned>(tour.size()))),
    _rank_limits(_nb_threads) {
  // Build _edges vector representation.
  auto location = tour.cbegin();
  const Index first_index = *location;
  Index last_index = first_index;
  ++location;
  while (location != tour.cend()) {
    const Index current_index = *location;
    _edges[last_index] = current_index;
    last_index = current_index;
    ++location;
  }
  _edges[last_index] = first_index;

  // Build a vector of bounds that easily split the [0, _edges.size()]
  // look-up range 'evenly' between threads for relocate and or-opt
  // operator.
  const std::size_t range_width = _edges.size() / _nb_threads;
  std::iota(_rank_limits.begin(), _rank_limits.end(), 0);
  std::transform(_rank_limits.begin(),
                 _rank_limits.end(),
                 _rank_limits.begin(),
                 [range_width](std::size_t v) { return range_width * v; });
  // Shifting the limits to dispatch remaining ranks among more
  // threads for a more even load balance. This way the load
  // difference between ranges should be at most 1.
  const std::size_t remainder = _edges.size() % _nb_threads;
  std::size_t shift = 0;
  for (std::size_t i = 1; i < _rank_limits.size(); ++i) {
    if (shift < remainder) {
      ++shift;
    }
    _rank_limits[i] += shift;
  }
  _rank_limits.push_back(_edges.size());

  // Build a vector of bounds that easily split the [0, _edges.size()]
  // look-up range 'evenly' between threads for 2-opt symmetric
  // operator.
  _sym_two_opt_rank_limits.reserve(_nb_threads + 1);
  _sym_two_opt_rank_limits.push_back(0);

  if (_nb_threads > 1) {
    // When avoiding duplicate tests in two-opt (symmetric case), the
    // first choice for edge_1 requires number_of_lookups[0] checks
    // for edge_2, the next requires number_of_lookups[1] and so
    // on. If several threads are used, splitting the share between
    // them is based on this workload.

    std::vector<unsigned> number_of_lookups(_edges.size() - 1);
    number_of_lookups[0] = _edges.size() - 3;
    std::iota(number_of_lookups.rbegin(), number_of_lookups.rend() - 1, 0);

    std::vector<unsigned> cumulated_lookups;
    std::partial_sum(number_of_lookups.begin(),
                     number_of_lookups.end(),
                     std::back_inserter(cumulated_lookups));

    const unsigned total_lookups = _edges.size() * (_edges.size() - 3) / 2;
    const unsigned thread_lookup_share = total_lookups / _nb_threads;

    Index rank = 0;
    for (std::size_t i = 1; i < _nb_threads; ++i) {
      // Finding nodes that separate current tour in _nb_threads ranges.
      while (cumulated_lookups[rank] < i * thread_lookup_share) {
        ++rank;
      }
      ++rank;
      _sym_two_opt_rank_limits.push_back(rank);
    }
  }
  _sym_two_opt_rank_limits.push_back(_edges.size());
}

UserCost LocalSearch::relocate_step() {
  if (_edges.size() < 3) {
    // Not enough edges for the operator to make sense.
    return 0;
  }

  // Lambda function to search for the best move in a range of
  // elements from _edges.
  auto look_up = [&](Index start,
                     Index end,
                     UserCost& best_gain,
                     Index& best_edge_1_start,
                     Index& best_edge_2_start) {
    for (Index edge_1_start = start; edge_1_start < end; ++edge_1_start) {
      const Index edge_1_end = _edges[edge_1_start];
      // Going through the tour while checking for insertion of
      // edge_1_end between two other nodes (edge_2_*).
      //
      // Namely edge_1_start --> edge_1_end --> next is replaced by
      // edge_1_start --> next while edge_2_start --> edge_2_end is
      // replaced by edge_2_start --> edge_1_end --> edge_2_end.
      const Index next = _edges[edge_1_end];

      // Precomputing weights not depending on edge_2_*.
      auto first_potential_add = _matrix[edge_1_start][next];
      auto edge_1_weight = _matrix[edge_1_start][edge_1_end];
      auto edge_1_end_next_weight = _matrix[edge_1_end][next];

      if (edge_1_weight + edge_1_end_next_weight - first_potential_add <
          best_gain) {
        // if edge_2_start --> edge_2_end is shorter than
        // edge_2_start --> edge_1_end --> edge_2_end (which it should be)
        // than the gain can't be larger than the improvement between
        // edge_1_start --> edge_1_end --> next  and
        // edge_1_start --> next
        // Note: No harm is done if this underflows due to triangle inequality
        // violations
        continue;
      }

      Index edge_2_start = next;
      while (edge_2_start != edge_1_start) {
        const Index edge_2_end = _edges[edge_2_start];
        const auto before_cost = edge_1_weight + edge_1_end_next_weight +
                                 _matrix[edge_2_start][edge_2_end];

        if (const auto after_cost = first_potential_add +
                                    _matrix[edge_2_start][edge_1_end] +
                                    _matrix[edge_1_end][edge_2_end];
            before_cost > after_cost) {
          const auto gain = before_cost - after_cost;
          if (gain > best_gain) {
            best_edge_1_start = edge_1_start;
            best_edge_2_start = edge_2_start;
            best_gain = gain;
          }
        }
        // Go for next possible second edge.
        edge_2_start = edge_2_end;
      }
    }
  };

  // Store best values per thread.
  std::vector<UserCost> best_gains(_nb_threads, 0);
  std::vector<Index> best_edge_1_starts(_nb_threads);
  std::vector<Index> best_edge_2_starts(_nb_threads);

  std::vector<std::jthread> threads;
  threads.reserve(_nb_threads);

  for (std::size_t i = 0; i < _nb_threads; ++i) {
    threads.emplace_back(look_up,
                         _rank_limits[i],
                         _rank_limits[i + 1],
                         std::ref(best_gains[i]),
                         std::ref(best_edge_1_starts[i]),
                         std::ref(best_edge_2_starts[i]));
  }

  for (auto& t : threads) {
    t.join();
  }

  // Spot best gain found among all threads.
  auto best_rank =
    std::distance(best_gains.begin(), std::ranges::max_element(best_gains));
  auto best_gain = best_gains[best_rank];
  const Index best_edge_1_start = best_edge_1_starts[best_rank];
  const Index best_edge_2_start = best_edge_2_starts[best_rank];

  if (best_gain > 0) {
    // Performing best possible exchange.
    const Index best_edge_1_end = _edges[best_edge_1_start];
    const Index best_edge_2_end = _edges[best_edge_2_start];

    _edges[best_edge_1_start] = _edges[best_edge_1_end];
    _edges[best_edge_1_end] = best_edge_2_end;
    _edges[best_edge_2_start] = best_edge_1_end;
  }

  return best_gain;
}

UserCost LocalSearch::perform_all_relocate_steps(const Deadline& deadline) {
  UserCost total_gain = 0;
  UserCost gain = 0;
  do {
    if (deadline.has_value() && deadline.value() < utils::now()) {
      break;
    }

    gain = this->relocate_step();

    if (gain > 0) {
      total_gain += gain;
    }
  } while (gain > 0);

  return total_gain;
}

UserCost LocalSearch::avoid_loop_step() {
  // In some cases, the solution can contain "loops" that other
  // operators can't fix. Those are found with two steps:
  //
  // 1) searching for all nodes that can be relocated somewhere else
  // AT NO COST because they are already on some other way.
  //
  // 2) listing all "chains" of two or more consecutive such nodes.
  //
  // Starting from the longest such chain, the fix is to:
  //
  // 3) relocate all nodes along the chain until an amelioration pops
  // out, meaning a "loop" has been undone.

  UserCost gain = 0;

  // Going through all candidate nodes for relocation.
  Index previous_candidate = 0;
  Index candidate = _edges[previous_candidate];

  // Remember previous steps for each node, required for step 3.
  std::vector<Index> previous(_matrix.size());
  previous.at(candidate) = previous_candidate;

  // Storing chains as described in 2.
  std::vector<std::list<Index>> relocatable_chains;
  std::list<Index> current_relocatable_chain;

  // Remember possible position for further relocation of candidate
  // nodes.
  std::unordered_map<Index, Index> possible_position;

  do {
    Index current = _edges[candidate];

    bool candidate_relocatable = false;
    if (!_avoid_start_relocate.first ||
        candidate != _avoid_start_relocate.second) {
      while ((current != previous_candidate) && !candidate_relocatable) {
        const Index next = _edges[current];
        if ((_matrix[current][candidate] + _matrix[candidate][next] <=
             _matrix[current][next]) &&
            (_matrix[current][candidate] > 0) &&
            (_matrix[candidate][next] > 0)) {
          // Relocation at no cost, set aside the case of identical
          // locations.
          candidate_relocatable = true;
          // Remember possible relocate position for candidate.
          possible_position.emplace(candidate, current);
        }
        current = next;
      }
    }
    if (candidate_relocatable) {
      current_relocatable_chain.push_back(candidate);
    } else {
      if (current_relocatable_chain.size() > 1) {
        relocatable_chains.push_back(current_relocatable_chain);
      }
      current_relocatable_chain.clear();
    }
    previous_candidate = candidate;
    candidate = _edges[candidate];
    previous.at(candidate) = previous_candidate;
  } while (candidate != 0);

  // Reorder to try the longest chains first.
  std::ranges::sort(relocatable_chains,
                    [](const std::list<Index>& lhs,
                       const std::list<Index>& rhs) {
                      return lhs.size() > rhs.size();
                    });

  bool amelioration_found = false;
  for (auto const& chain : relocatable_chains) {
    // Going through step 3. for all chains by decreasing length.
    UserCost before_cost = 0;
    UserCost after_cost = 0;

    // Work on copies as modifications are needed while going through
    // the chain.
    std::vector<Index> edges_c = _edges;
    std::vector<Index> previous_c = previous;

    for (auto const& step : chain) {
      // Compare situations to see if relocating current step after
      // possible_position.at(step) will decrease overall cost.
      //
      // Situation before:
      //
      // previous_c.at(step)-->step-->edges_c.at(step)
      // possible_position.at(step)-->edges_c.at(possible_position.at(step))
      //
      // Situation after:
      //
      // previous_c.at(step)-->edges_c.at(step)
      // possible_position.at(step)-->step-->edges_c.at(possible_position.at(step))

      before_cost += _matrix[previous_c.at(step)][step];
      before_cost += _matrix[step][edges_c.at(step)];
      after_cost += _matrix[previous_c.at(step)][edges_c.at(step)];
      before_cost += _matrix[possible_position.at(step)]
                            [edges_c.at(possible_position.at(step))];
      after_cost += _matrix[possible_position.at(step)][step];
      after_cost += _matrix[step][edges_c.at(possible_position.at(step))];

      // Linking previous_c.at(step) with edges_c.at(step) in both
      // ways as remembering previous nodes is required.
      previous_c.at(edges_c.at(step)) = previous_c.at(step);
      edges_c.at(previous_c.at(step)) = edges_c.at(step);

      // Relocating step between possible_position.at(step) and
      // edges_c.at(possible_position.at(step)) in both ways too.
      edges_c.at(step) = edges_c.at(possible_position.at(step));
      previous_c.at(edges_c.at(possible_position.at(step))) = step;

      edges_c.at(possible_position.at(step)) = step;
      previous_c.at(step) = possible_position.at(step);

      if (before_cost > after_cost) {
        amelioration_found = true;
        gain = before_cost - after_cost;
        _edges.swap(edges_c); // Keep changes.
        break;
      }
    }
    if (amelioration_found) {
      break;
    }
  }

  return gain;
}

UserCost LocalSearch::perform_all_avoid_loop_steps(const Deadline& deadline) {
  UserCost total_gain = 0;
  UserCost gain = 0;
  do {
    if (deadline.has_value() && deadline.value() < utils::now()) {
      break;
    }

    gain = this->avoid_loop_step();

    if (gain > 0) {
      total_gain += gain;
    }
  } while (gain > 0);

  return total_gain;
}

UserCost LocalSearch::two_opt_step() {
  if (_edges.size() < 4) {
    // Not enough edges for the operator to make sense.
    return 0;
  }

  // Lambda function to search for the best move in a range of
  // elements from _edges.
  auto look_up = [&](Index start,
                     Index end,
                     UserCost& best_gain,
                     Index& best_edge_1_start,
                     Index& best_edge_2_start) {
    for (Index edge_1_start = start; edge_1_start < end; ++edge_1_start) {
      const Index edge_1_end = _edges[edge_1_start];
      for (Index edge_2_start = edge_1_start + 1; edge_2_start < _edges.size();
           ++edge_2_start) {
        // Trying to improve two "crossing edges".
        //
        // Namely edge_1_start --> edge_1_end and edge_2_start -->
        // edge_2_end are replaced by edge_1_start --> edge_2_start and
        // edge_1_end --> edge_2_end. The tour between edge_1_end and
        // edge_2_start need to be reversed.
        //
        // In the symmetric case, trying the move with edges (e_2, e_1)
        // is the same as with (e_1, e_2), so assuming edge_1_start <
        // edge_2_start avoids testing pairs in both orders.

        const Index edge_2_end = _edges[edge_2_start];
        if ((edge_2_start == edge_1_end) || (edge_2_end == edge_1_start)) {
          // Operator doesn't make sense.
          continue;
        }

        auto before_cost =
          _matrix[edge_1_start][edge_1_end] + _matrix[edge_2_start][edge_2_end];
        auto after_cost =
          _matrix[edge_1_start][edge_2_start] + _matrix[edge_1_end][edge_2_end];

        if (before_cost > after_cost) {
          auto gain = before_cost - after_cost;
          if (gain > best_gain) {
            best_gain = gain;
            best_edge_1_start = edge_1_start;
            best_edge_2_start = edge_2_start;
          }
        }
      }
    }
  };

  // Store best values per thread.
  std::vector<UserCost> best_gains(_nb_threads, 0);
  std::vector<Index> best_edge_1_starts(_nb_threads);
  std::vector<Index> best_edge_2_starts(_nb_threads);

  // Start other threads, keeping a piece of the range for the main
  // thread.
  std::vector<std::jthread> threads;
  threads.reserve(_nb_threads);

  for (std::size_t i = 0; i < _nb_threads; ++i) {
    threads.emplace_back(look_up,
                         _sym_two_opt_rank_limits[i],
                         _sym_two_opt_rank_limits[i + 1],
                         std::ref(best_gains[i]),
                         std::ref(best_edge_1_starts[i]),
                         std::ref(best_edge_2_starts[i]));
  }

  for (auto& t : threads) {
    t.join();
  }

  // Spot best gain found among all threads.
  auto best_rank =
    std::distance(best_gains.begin(), std::ranges::max_element(best_gains));
  auto best_gain = best_gains[best_rank];
  const Index best_edge_1_start = best_edge_1_starts[best_rank];
  const Index best_edge_2_start = best_edge_2_starts[best_rank];

  if (best_gain > 0) {
    const Index best_edge_1_end = _edges[best_edge_1_start];
    const Index best_edge_2_end = _edges[best_edge_2_start];
    // Storing part of the tour that needs to be reversed.
    std::vector<Index> to_reverse;
    for (Index current = best_edge_1_end; current != best_edge_2_start;
         current = _edges[current]) {
      to_reverse.push_back(current);
    }
    // Performing exchange.
    Index current = best_edge_2_start;
    _edges[best_edge_1_start] = current;
    for (const auto& next : std::ranges::reverse_view(to_reverse)) {
      _edges[current] = next;
      current = next;
    }
    _edges[current] = best_edge_2_end;
  }

  return best_gain;
}

UserCost LocalSearch::asym_two_opt_step() {
  if (_edges.size() < 4) {
    // Not enough edges for the operator to make sense.
    return 0;
  }

  // The initial node for the first edge is arbitrary but it is handy
  // to keep in mind the previous one for stopping conditions.
  const Index previous_init = _edges.front();
  Index init = _edges[previous_init];

  // Lambda function to search for the best move in a range of
  // elements from _edges.
  auto look_up = [&](Index start,
                     Index end,
                     UserCost& best_gain,
                     Index& best_edge_1_start,
                     Index& best_edge_2_start) {
    Index edge_1_start = start;

    do {
      // Going through the edges in the order of the current tour.
      const Index edge_1_end = _edges[edge_1_start];
      Index edge_2_start = _edges[edge_1_end];
      Index edge_2_end = _edges[edge_2_start];
      // Trying to improve two "crossing edges".
      //
      // Namely edge_1_start --> edge_1_end and edge_2_start -->
      // edge_2_end are replaced by edge_1_start --> edge_2_start and
      // edge_1_end --> edge_2_end. The tour between edge_1_end and
      // edge_2_start need to be reversed.
      UserCost before_reversed_part_cost = 0;
      UserCost after_reversed_part_cost = 0;
      Index previous = edge_1_end;

      while (edge_2_end != edge_1_start) {
        // Going through the edges in the order of the current tour
        // (mandatory for before_cost and after_cost efficient
        // computation).
        auto before_cost =
          _matrix[edge_1_start][edge_1_end] + _matrix[edge_2_start][edge_2_end];
        auto after_cost =
          _matrix[edge_1_start][edge_2_start] + _matrix[edge_1_end][edge_2_end];

        // Updating the cost of the part of the tour that needs to be
        // reversed.
        before_reversed_part_cost += _matrix[previous][edge_2_start];
        after_reversed_part_cost += _matrix[edge_2_start][previous];

        // Adding to the costs for comparison.
        before_cost += before_reversed_part_cost;
        after_cost += after_reversed_part_cost;

        if (before_cost > after_cost) {
          auto gain = before_cost - after_cost;
          if (gain > best_gain) {
            best_gain = gain;
            best_edge_1_start = edge_1_start;
            best_edge_2_start = edge_2_start;
          }
        }
        // Go for next possible second edge.
        previous = edge_2_start;
        edge_2_start = edge_2_end;
        edge_2_end = _edges[edge_2_start];
      }
      edge_1_start = _edges[edge_1_start];
    } while (edge_1_start != end);
  };

  // Store best values per thread.
  std::vector<UserCost> best_gains(_nb_threads, 0);
  std::vector<Index> best_edge_1_starts(_nb_threads);
  std::vector<Index> best_edge_2_starts(_nb_threads);
  const std::size_t thread_range = _edges.size() / _nb_threads;

  // The limits in the range given to each thread are not ranks but
  // actual nodes used to browse a piece of the current tour.
  std::vector<std::size_t> limit_nodes;
  limit_nodes.reserve(_nb_threads + 1);
  limit_nodes.push_back(init);
  Index node = init;
  for (std::size_t i = 0; i < _nb_threads - 1; ++i) {
    // Finding nodes that separate current tour in _nb_threads ranges.
    for (std::size_t j = 0; j < thread_range; ++j) {
      node = _edges[node];
    }
    limit_nodes.push_back(node);
  }
  limit_nodes.push_back(init);

  std::vector<std::jthread> threads;
  threads.reserve(_nb_threads);

  for (std::size_t i = 0; i < _nb_threads; ++i) {
    threads.emplace_back(look_up,
                         limit_nodes[i],
                         limit_nodes[i + 1],
                         std::ref(best_gains[i]),
                         std::ref(best_edge_1_starts[i]),
                         std::ref(best_edge_2_starts[i]));
  }

  for (auto& t : threads) {
    t.join();
  }

  // Spot best gain found among all threads.
  auto best_rank =
    std::distance(best_gains.begin(), std::ranges::max_element(best_gains));
  auto best_gain = best_gains[best_rank];
  const Index best_edge_1_start = best_edge_1_starts[best_rank];
  const Index best_edge_2_start = best_edge_2_starts[best_rank];

  if (best_gain > 0) {
    const Index best_edge_1_end = _edges[best_edge_1_start];
    const Index best_edge_2_end = _edges[best_edge_2_start];
    // Storing part of the tour that needs to be reversed.
    std::vector<Index> to_reverse;
    for (Index current = best_edge_1_end; current != best_edge_2_start;
         current = _edges[current]) {
      to_reverse.push_back(current);
    }
    // Performing exchange.
    Index current = best_edge_2_start;
    _edges[best_edge_1_start] = current;
    for (const auto& next : std::ranges::reverse_view(to_reverse)) {
      _edges[current] = next;
      current = next;
    }
    _edges[current] = best_edge_2_end;
  }

  return best_gain;
}

UserCost LocalSearch::perform_all_two_opt_steps(const Deadline& deadline) {
  UserCost total_gain = 0;
  UserCost gain = 0;
  do {
    if (deadline.has_value() && deadline.value() < utils::now()) {
      break;
    }

    gain = this->two_opt_step();

    if (gain > 0) {
      total_gain += gain;
    }
  } while (gain > 0);

  return total_gain;
}

UserCost LocalSearch::perform_all_asym_two_opt_steps(const Deadline& deadline) {
  UserCost total_gain = 0;
  UserCost gain = 0;
  do {
    if (deadline.has_value() && deadline.value() < utils::now()) {
      break;
    }

    gain = this->asym_two_opt_step();

    if (gain > 0) {
      total_gain += gain;
    }
  } while (gain > 0);

  return total_gain;
}

UserCost LocalSearch::or_opt_step() {
  if (_edges.size() < 4) {
    // Not enough edges for the operator to make sense.
    return 0;
  }

  // Lambda function to search for the best move in a range of
  // elements from _edges.
  auto look_up = [&](Index start,
                     Index end,
                     UserCost& best_gain,
                     Index& best_edge_1_start,
                     Index& best_edge_2_start) {
    for (Index edge_1_start = start; edge_1_start < end; ++edge_1_start) {
      const Index edge_1_end = _edges[edge_1_start];
      const Index next = _edges[edge_1_end];
      const Index next_2 = _edges[next];
      Index edge_2_start = next_2;
      // Going through the tour while checking the move of edge after
      // edge_1_end in place of another edge (edge_2_*).
      //
      // Namely edge_1_start --> edge_1_end --> next --> next_2 is
      // replaced by edge_1_start --> next_2 while edge_2_start -->
      // edge_2_end is replaced by edge_2_start --> edge_1_end
      // --> next --> edge_2_end.

      // Precomputing weights not depending on edge_2.
      auto first_potential_add = _matrix[edge_1_start][next_2];
      auto edge_1_weight = _matrix[edge_1_start][edge_1_end];
      auto next_next_2_weight = _matrix[next][next_2];

      while (edge_2_start != edge_1_start) {
        const Index edge_2_end = _edges[edge_2_start];
        const auto before_cost = edge_1_weight + next_next_2_weight +
                                 _matrix[edge_2_start][edge_2_end];
        if (const auto after_cost = first_potential_add +
                                    _matrix[edge_2_start][edge_1_end] +
                                    _matrix[next][edge_2_end];
            before_cost > after_cost) {
          const auto gain = before_cost - after_cost;
          if (gain > best_gain) {
            best_gain = gain;
            best_edge_1_start = edge_1_start;
            best_edge_2_start = edge_2_start;
          }
        }
        // Go for next possible second edge.
        edge_2_start = edge_2_end;
      }
    }
  };

  // Store best values per thread.
  std::vector<UserCost> best_gains(_nb_threads, 0);
  std::vector<Index> best_edge_1_starts(_nb_threads);
  std::vector<Index> best_edge_2_starts(_nb_threads);

  std::vector<std::jthread> threads;
  threads.reserve(_nb_threads);

  for (std::size_t i = 0; i < _nb_threads; ++i) {
    threads.emplace_back(look_up,
                         _rank_limits[i],
                         _rank_limits[i + 1],
                         std::ref(best_gains[i]),
                         std::ref(best_edge_1_starts[i]),
                         std::ref(best_edge_2_starts[i]));
  }

  for (auto& t : threads) {
    t.join();
  }

  // Spot best gain found among all threads.
  auto best_rank =
    std::distance(best_gains.begin(), std::ranges::max_element(best_gains));
  auto best_gain = best_gains[best_rank];
  const Index best_edge_1_start = best_edge_1_starts[best_rank];
  const Index best_edge_2_start = best_edge_2_starts[best_rank];

  if (best_gain > 0) {
    const Index best_edge_1_end = _edges[best_edge_1_start];
    const Index next = _edges[best_edge_1_end];

    // Performing exchange.
    _edges[best_edge_1_start] = _edges[next];
    _edges[next] = _edges[best_edge_2_start];
    _edges[best_edge_2_start] = best_edge_1_end;
  }
  return best_gain;
}

UserCost LocalSearch::perform_all_or_opt_steps(const Deadline& deadline) {
  UserCost total_gain = 0;
  UserCost gain = 0;
  do {
    if (deadline.has_value() && deadline.value() < utils::now()) {
      break;
    }

    gain = this->or_opt_step();
    if (gain > 0) {
      total_gain += gain;
    }
  } while (gain > 0);

  return total_gain;
}

std::list<Index> LocalSearch::get_tour(Index first_index) const {
  std::list<Index> tour;
  tour.push_back(first_index);
  Index next_index = _edges[first_index];
  while (next_index != first_index) {
    tour.push_back(next_index);
    next_index = _edges[next_index];
  }
  return tour;
}

} // namespace vroom::tsp
