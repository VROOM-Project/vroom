#ifndef MUNKRES_H
#define MUNKRES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_map>
#include <limits>
#include <set>
#include <list>
#include <cassert>
#include "../structures/matrix.h"
#include "../structures/edge.h"

template <class T>
std::unordered_map<index_t, index_t> minimum_weight_perfect_matching(const matrix<T>& m){

  // Trivial initial labeling.
  std::unordered_map<index_t, T> labeling_x;
  std::unordered_map<index_t, T> labeling_y;
  for(index_t i = 0; i < m.size(); ++i){
    labeling_y.emplace(i, 0);
    T min_weight = std::numeric_limits<T>::max();
    for(index_t j = 0; j < m.size(); ++j){
      if(m[i][j] < min_weight){
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

  while(matching_xy.size() < m.size()){
    // Step 1.

    alternating_tree.clear();
    std::set<index_t> S;
    std::set<index_t> T_set;
    
    // Finding any unmatched x.
    index_t unmatched_x = 0;
    while(matching_xy.find(unmatched_x) != matching_xy.end()){
      ++unmatched_x;
    }
    S.insert(unmatched_x);

    // Saving relevant neighbors in equality graph in alternating_tree
    // and initializing slacks.
    std::unordered_map<index_t, T> slack;
    for(index_t y = 0; y < m.size(); ++y){
      if(labeling_x.at(unmatched_x) + labeling_y.at(y) == m[unmatched_x][y]){
        alternating_tree.emplace(y, unmatched_x);
      }
      slack.emplace(y,
                    m[unmatched_x][y] - labeling_x.at(unmatched_x) - labeling_y.at(y)
                    );
    }

    bool augmented_path = false;

    while(!augmented_path){
      // Test if neighbors of S in equality graph equals T_set or not
      // (note that T_set is included in S neighbors).
      if(alternating_tree.size() == T_set.size()){
        // Step 2.

        T alpha = std::numeric_limits<T>::max();
        for(index_t y = 0; y < m.size(); ++y){
          // Computing alpha, the minimum of slack values over
          // complement of T_set.
          if(T_set.find(y) == T_set.end()){
            T current_slack = slack.at(y);
            if(current_slack < alpha){
              alpha = current_slack;
            }
          }
        }

        // Update labelings
        for(auto const& x: S){
          labeling_x.at(x) = labeling_x.at(x) + alpha;
        }
        for(auto const& y: T_set){
          labeling_y.at(y) = labeling_y.at(y) - alpha;
        }

        // Updating relevant neighbors in new equality graph and
        // updating slacks.
        for(index_t y = 0; y < m.size(); ++y){
          if(T_set.find(y) == T_set.end()){
            slack.at(y) = slack.at(y) - alpha;
            
            for(auto const& x: S){
              if(labeling_x.at(x) + labeling_y.at(y) == m[x][y]){
                if(alternating_tree.find(y) == alternating_tree.end()){
                  alternating_tree.emplace(y, x);
                }
              }
            }
          }
        }
      }

      // Step 3.
      index_t chosen_y;        // First y in equality neighbors not
                                // in T_set.
      for(auto const& edge: alternating_tree){
        if(T_set.find(edge.first) == T_set.end()){
          // MUST happen before endge reaches the end of
          // alternating_tree.
          chosen_y = edge.first;
          break;
        }
      }

      auto matching_y = matching_yx.find(chosen_y);
      if(matching_y != matching_yx.end()){
        // Chosen y is actually matched in M, update S and T_set and
        // proceed to step 2.
        index_t matched_x = matching_y->second;

        S.insert(matched_x);
        T_set.insert(chosen_y);

        // Updating slacks.
        for(index_t y = 0; y < m.size(); ++y){
          T current_value = slack.at(y);
          T new_value
            = m[matched_x][y] - labeling_x.at(matched_x) - labeling_y.at(y);
          if(new_value < current_value){
            slack.at(y) = new_value;
          }
        }
      }
      else{
        // Find larger matching using M-alternating path. The path is
        // described at each step by:
        // 
        // chosen_y -- chosen_x -- next_y -- [...] -- unmatched_x
        //
        // where (chosen_x, next_y) is already in matching and should
        // be removed and (chosen_x, chosen_y) is to be added.
        
        index_t current_y = chosen_y;
        index_t current_x = alternating_tree.at(current_y);

        while(current_x != unmatched_x){
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
std::unordered_map<index_t, index_t> branch_and_bound_symmetric_mwpm(const matrix<T>& m){
  // Note: intended for an even-sized matrix with inf value on the
  // diagonal (no i-i matches may be produced anyway).

  // Branch and bound algorithm using a tree whose nodes are chosen
  // edges, taking advantage of comparison order between edges. First
  // level of tree is 0-1, 0-2, ... 0-(n-1), then 0-1 is the parent of
  // 2-3, 2-4... (all greater edges starting with 2). 0-2 is the
  // parent of 1-3, 1-4, 1-5... Computing children nodes just requires
  // to remember remaining indices (removed along while picking edges
  // from the parents.

  // The BB_tree_node structure allows a depth-first search of the
  // above described tree, stopping at any depth when getting over the
  // best weight currently known.

  struct BB_tree_node{
    const matrix<T>& mat;
    std::list<edge<T>> chosen_edges;
    std::set<index_t> remaining_indices;
    T cumulated_weight;

    // Constructor for the root node.
    BB_tree_node(const matrix<T>& mat):
      mat(mat),
      cumulated_weight(0)
    {
      for(index_t i = 0; i < mat.size(); ++i){
        remaining_indices.insert(i);
      }
    }

    // Constructor for children nodes.
    BB_tree_node(const BB_tree_node& parent,
                 const edge<T>& chosen_edge):
      mat(parent.mat),
      chosen_edges(parent.chosen_edges),
      remaining_indices(parent.remaining_indices),
      cumulated_weight(parent.cumulated_weight)
    {
      // Update chosen edges.
      this->chosen_edges.push_back(chosen_edge);
      // Update remaining indices and cumulated weight.
      index_t first = chosen_edge.get_first_vertex();
      index_t second = chosen_edge.get_second_vertex();
      this->remaining_indices.erase(first);
      this->remaining_indices.erase(second);
      this->cumulated_weight += chosen_edge.get_weight();
    }

    std::list<edge<T>> get_children_edges(){
      std::list<edge<T>> children_edges;

      if(remaining_indices.size() > 0){
        auto current = remaining_indices.cbegin();
        index_t first_index = *current;
        ++current;
        while(current != remaining_indices.cend()){
          children_edges.emplace_back(first_index,
                                      *current,
                                      mat(first_index, *current)
                                      );
          ++current;
        }
      }
      return children_edges;
    }
  };

  T current_best_weight = std::numeric_limits<T>::max();
  std::list<edge<T>> best_edges_choice;

  BB_tree_node root (m);

  // Using a list as a pile to operate a depth-first search on the
  // above described tree.
  std::list<BB_tree_node> yet_to_visit;
  yet_to_visit.push_back(root);

  while(!yet_to_visit.empty()){
    // First on the pile.
    BB_tree_node current_node = yet_to_visit.back();
    yet_to_visit.pop_back();

    if(current_node.remaining_indices.size() == 0
       and current_node.cumulated_weight < current_best_weight){
      // Current node is a leaf giving a better result.
      current_best_weight = current_node.cumulated_weight;
      best_edges_choice = current_node.chosen_edges;
    }
    else{
      // Leaf not reached yet.
      if(current_node.cumulated_weight < current_best_weight){
        // Else sure to get a bigger weight in the end so doing
        // nothing cuts the branch.
        std::list<edge<T>> children_edges = current_node.get_children_edges();
        children_edges.sort([](auto const& a, auto const& b)
                            {
                              return (a.get_weight() < b.get_weight());
                            });  
        // Putting highest weights first to pick them later.
        for(auto edge = children_edges.rbegin();
            edge != children_edges.rend();
            ++edge){
          yet_to_visit.emplace_back(current_node, *edge);
        }
      }
    }
  }  
  
  std::unordered_map<index_t, index_t> matching;
  for(auto const& edge: best_edges_choice){
    matching.emplace(edge.get_first_vertex(), edge.get_second_vertex());
  }
  
  return matching;
}

template <class T>
std::unordered_map<index_t, index_t> greedy_symmetric_approx_mwpm(const matrix<T>& m){
  // Fast greedy algorithm for finding a symmetric perfect matching,
  // choosing always smaller possible value, no minimality
  // assured. Matrix size should be even!
  assert(m.size() % 2 == 0);
    
  std::unordered_map<index_t, index_t> matching;
  std::set<index_t> remaining_indices;
  for(index_t i = 0; i < m.size(); ++i){
    remaining_indices.insert(i);
  }

  while(remaining_indices.size() > 0){
    T min_weight = std::numeric_limits<T>::max();
    index_t first_chosen_index;
    index_t second_chosen_index;
    std::set<index_t>::iterator chosen_i;
    std::set<index_t>::iterator chosen_j;
    for(auto i = remaining_indices.begin();
        i != remaining_indices.end();
        ++i){
      auto j = i;
      ++j;
      for(; j != remaining_indices.end(); ++j){
        T current_weight = m[*i][*j];
        if(current_weight < min_weight){
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

#endif
