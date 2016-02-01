/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
