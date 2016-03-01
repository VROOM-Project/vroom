/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "mst_heuristic.h"

std::list<index_t> mst_heuristic::build_solution(const tsp& instance){
  BOOST_LOG_TRIVIAL(trace) << "* Graph has " 
                           << instance.size() 
                           << " nodes.";

  // Getting minimum spanning tree of associated graph under the form
  // of an adjacency list.
  std::unordered_map<index_t, std::list<index_t>> adjacency_list
    = minimum_spanning_tree(instance.get_symmetrized_graph()).get_adjacency_list();
  assert(adjacency_list.size() >= 2);
  
  // Initializing the depth-first search of the minimum spanning tree
  // with any vertex.
  std::stack<index_t> df_list;

  index_t current_vertex = adjacency_list.begin()->first;
  df_list.push(current_vertex);

  std::list<index_t> tour;

  while(!df_list.empty()){
    current_vertex = df_list.top();
    df_list.pop();

    for(const auto& vertex: adjacency_list[current_vertex]){
      // Adding neighbour for further visit.
      df_list.push(vertex);
      // Making sure current edge won't be used backward later.
      adjacency_list[vertex].remove(current_vertex);
    }

    tour.push_back(current_vertex);
  }
  
  return tour;
}
