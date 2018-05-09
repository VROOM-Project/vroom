/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>
#include <limits>
#include <list>
#include <set>

#include "algorithms/munkres.h"
#include "structures/abstract/edge.h"

template <class T>
std::unordered_map<index_t, index_t>
minimum_weight_perfect_matching(const matrix<T>& m) {

  // Trivial initial labeling.
  std::unordered_map<index_t, T> labeling_x;
  std::unordered_map<index_t, T> labeling_y;
  for (index_t i = 0; i < m.size(); ++i) {
    labeling_y.emplace(i, 0);
    T min_weight = std::numeric_limits<T>::max();
    for (index_t j = 0; j < m.size(); ++j) {
      if (m[i][j] < min_weight) {
        min_weight = m[i][j];
      }
    }
    labeling_x.emplace(i, min_weight);
  }

  // Initial empty matching.
  std::unordered_map<index_t, index_t> matching_xy;
  std::unordered_map<index_t, index_t> matching_yx;

  // Alternating tree.
  std::unordered_map<index_t, index_t> alternating_tree;

  while (matching_xy.size() < m.size()) {
    // Step 1.

    alternating_tree.clear();
    std::set<index_t> S;
    std::set<index_t> T_set;

    // Finding any unmatched x.
    index_t unmatched_x = 0;
    while (matching_xy.find(unmatched_x) != matching_xy.end()) {
      ++unmatched_x;
    }
    S.insert(unmatched_x);

    // Saving relevant neighbors in equality graph in alternating_tree
    // and initializing slacks.
    std::unordered_map<index_t, T> slack;
    for (index_t y = 0; y < m.size(); ++y) {
      if (labeling_x.at(unmatched_x) + labeling_y.at(y) == m[unmatched_x][y]) {
        alternating_tree.emplace(y, unmatched_x);
      }
      slack.emplace(y,
                    m[unmatched_x][y] - labeling_x.at(unmatched_x) -
                      labeling_y.at(y));
    }

    bool augmented_path = false;

    while (!augmented_path) {
      // Test if neighbors of S in equality graph equals T_set or not
      // (note that T_set is included in S neighbors).
      if (alternating_tree.size() == T_set.size()) {
        // Step 2.

        T alpha = std::numeric_limits<T>::max();
        for (index_t y = 0; y < m.size(); ++y) {
          // Computing alpha, the minimum of slack values over
          // complement of T_set.
          if (T_set.find(y) == T_set.end()) {
            T current_slack = slack.at(y);
            if (current_slack < alpha) {
              alpha = current_slack;
            }
          }
        }

        // Update labelings
        for (auto const& x : S) {
          labeling_x.at(x) = labeling_x.at(x) + alpha;
        }
        for (auto const& y : T_set) {
          labeling_y.at(y) = labeling_y.at(y) - alpha;
        }

        // Updating relevant neighbors in new equality graph and
        // updating slacks.
        for (index_t y = 0; y < m.size(); ++y) {
          if (T_set.find(y) == T_set.end()) {
            slack.at(y) = slack.at(y) - alpha;

            for (auto const& x : S) {
              if (labeling_x.at(x) + labeling_y.at(y) == m[x][y]) {
                if (alternating_tree.find(y) == alternating_tree.end()) {
                  alternating_tree.emplace(y, x);
                }
              }
            }
          }
        }
      }

      // Step 3.

      // First y in equality neighbors not in T_set.
      index_t chosen_y;
      for (auto const& edge : alternating_tree) {
        if (T_set.find(edge.first) == T_set.end()) {
          // MUST happen before endge reaches the end of
          // alternating_tree.
          chosen_y = edge.first;
          break;
        }
      }

      auto matching_y = matching_yx.find(chosen_y);
      if (matching_y != matching_yx.end()) {
        // Chosen y is actually matched in M, update S and T_set and
        // proceed to step 2.
        index_t matched_x = matching_y->second;

        S.insert(matched_x);
        T_set.insert(chosen_y);

        // Updating slacks.
        for (index_t y = 0; y < m.size(); ++y) {
          T current_value = slack.at(y);
          T new_value =
            m[matched_x][y] - labeling_x.at(matched_x) - labeling_y.at(y);
          if (new_value < current_value) {
            slack.at(y) = new_value;
          }
        }
      } else {
        // Find larger matching using M-alternating path. The path is
        // described at each step by:
        //
        // chosen_y -- chosen_x -- next_y -- [...] -- unmatched_x
        //
        // where (chosen_x, next_y) is already in matching and should
        // be removed and (chosen_x, chosen_y) is to be added.

        index_t current_y = chosen_y;
        index_t current_x = alternating_tree.at(current_y);

        while (current_x != unmatched_x) {
          index_t next_y = matching_xy.at(current_x);

          // Remove alternating edge from current matching.
          matching_xy.erase(matching_xy.find(current_x));
          matching_yx.erase(matching_yx.find(next_y));

          // Add edge from alternating tree to matching.
          matching_xy.emplace(current_x, current_y);
          matching_yx.emplace(current_y, current_x);

          current_y = next_y;
          current_x = alternating_tree.at(current_y);
        }
        // Adding last edge from alternating tree.
        matching_xy.emplace(current_x, current_y);
        matching_yx.emplace(current_y, current_x);

        // Back to step 1.
        augmented_path = true;
      }
    }
  }
  return matching_xy;
}

template <class T>
std::unordered_map<index_t, index_t>
greedy_symmetric_approx_mwpm(const matrix<T>& m) {
  // Fast greedy algorithm for finding a symmetric perfect matching,
  // choosing always smaller possible value, no minimality
  // assured. Matrix size should be even!
  assert(m.size() % 2 == 0);

  std::unordered_map<index_t, index_t> matching;
  std::set<index_t> remaining_indices;
  for (index_t i = 0; i < m.size(); ++i) {
    remaining_indices.insert(i);
  }

  while (remaining_indices.size() > 0) {
    T min_weight = std::numeric_limits<T>::max();
    index_t first_chosen_index;
    index_t second_chosen_index;
    std::set<index_t>::iterator chosen_i;
    std::set<index_t>::iterator chosen_j;
    for (auto i = remaining_indices.begin(); i != remaining_indices.end();
         ++i) {
      auto j = i;
      ++j;
      for (; j != remaining_indices.end(); ++j) {
        T current_weight = m[*i][*j];
        if (current_weight < min_weight) {
          min_weight = current_weight;
          first_chosen_index = *i;
          second_chosen_index = *j;
          chosen_i = i;
          chosen_j = j;
        }
      }
    }
    matching.emplace(first_chosen_index, second_chosen_index);
    remaining_indices.erase(chosen_j);
    remaining_indices.erase(chosen_i);
  }

  return matching;
}

template std::unordered_map<index_t, index_t>
minimum_weight_perfect_matching(const matrix<cost_t>& m);

template std::unordered_map<index_t, index_t>
greedy_symmetric_approx_mwpm(const matrix<cost_t>& m);
