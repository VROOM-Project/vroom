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

std::list<index_t> mst_heuristic::build_solution(tsp_sym& instance){
  // Using the symmetric problem derived from the general one.
  undirected_graph<distance_t> graph (instance.get_matrix());

  // Getting minimum spanning tree of associated graph under the form
  // of an adjacency list.
  std::unordered_map<index_t, std::list<index_t>> adjacency_list
    = minimum_spanning_tree(graph).get_adjacency_list();
  
  // Initializing the depth-first search of the minimum spanning tree
  // with any vertex. Using the list as a stack.
  std::list<index_t> df_list;

  index_t current_vertex = adjacency_list.begin()->first;
  df_list.push_back(current_vertex);

  std::list<index_t> tour;

  while(!df_list.empty()){
    current_vertex = df_list.back();
    df_list.pop_back();

    for(auto vertex = adjacency_list[current_vertex].cbegin();
        vertex != adjacency_list[current_vertex].cend();
        ++vertex){
      // Adding neighbour for further visit.
      df_list.push_back(*vertex);
      // Making sure current edge won't be used backward later.
      adjacency_list[*vertex].remove(current_vertex);
    }

    tour.push_back(current_vertex);
  }
  
  return tour;
}
