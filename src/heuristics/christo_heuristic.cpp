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

#include "christo_heuristic.h"

std::list<index_t> christo_heuristic::build_solution(tsp_sym& instance){
  // The eulerian sub-graph further used is made of a minimum spanning
  // tree with a minimum weight perfect matching on its odd degree
  // vertices.

  undirected_graph<distance_t> mst_graph
    = minimum_spanning_tree(instance.get_graph());

  // Getting minimum spanning tree of associated graph under the form
  // of an adjacency list.
  std::unordered_map<index_t, std::list<index_t>> adjacency_list
    = mst_graph.get_adjacency_list();

  // Getting odd degree vertices from the minimum spanning tree.
  std::vector<index_t> mst_odd_vertices;
  for(auto adjacency = adjacency_list.cbegin();
      adjacency != adjacency_list.cend();
      ++adjacency){
    if(adjacency->second.size() % 2 == 1){
      mst_odd_vertices.push_back(adjacency->first);
    }
  }

  // Getting corresponding matrix for the generated sub-graph.
  static_matrix<distance_t> sub_matrix
    = instance.get_matrix().get_sub_matrix(mst_odd_vertices);

  // Making each node impossible to match with itself in minimum
  // weight perfect matching to come.
  for(index_t i = 0; i < sub_matrix.size(); ++i){
    // Setting max value would cause trouble with further additions...
    sub_matrix[i][i] = 3 * (std::numeric_limits<distance_t>::max() / 4);
  }

  // Computing minimum weight perfect matching.
  std::unordered_map<index_t, index_t> mwpm
    = minimum_weight_perfect_matching(sub_matrix);

  // Storing those edges from mwpm that are coherent regarding
  // symmetry (y -> x whenever x -> y). Remembering the rest of them
  // for further use. Edges are not doubled in mwpm_final.
  std::unordered_map<index_t, index_t> mwpm_final;
  std::vector<index_t> wrong_vertices;

  unsigned total_ok = 0;
  for(auto edge = mwpm.begin();
      edge != mwpm.end();
      ++edge){
    if(mwpm.at(edge->second) == edge->first){
      mwpm_final.emplace(std::min(edge->first, edge->second),
                         std::max(edge->first, edge->second));
      ++total_ok;
    }
    else{
      wrong_vertices.push_back(edge->first);
    }
  }
  
  if(!wrong_vertices.empty()){
    std::unordered_map<index_t, index_t> remaining_greedy_mwpm
      = greedy_symmetric_approx_mwpm(sub_matrix.get_sub_matrix(wrong_vertices));

    // Adding edges obtained with greedy algo for the missing vertices
    // in mwpm_final.
    for(auto edge = remaining_greedy_mwpm.cbegin();
        edge != remaining_greedy_mwpm.cend();
        ++edge){
      mwpm_final.emplace(std::min(wrong_vertices[edge->first],
                                  wrong_vertices[edge->second]),
                         std::max(wrong_vertices[edge->first],
                                  wrong_vertices[edge->second]));
    }
  }

  // Building eulerian graph.
  std::vector<edge<distance_t>> eulerian_graph_edges
    = mst_graph.get_edges();
  
  // Adding edges from minimum weight perfect matching (with the
  // original vertices index). Edges appear twice in matching so we
  // need to remember the one already added.
  std::set<index_t> already_added;
  // weight = 0;
  for(auto edge = mwpm_final.cbegin(); edge != mwpm_final.cend(); ++edge){
    index_t first_index = mst_odd_vertices[edge->first];
    index_t second_index = mst_odd_vertices[edge->second];
    if(already_added.find(first_index) == already_added.end()){
      eulerian_graph_edges.emplace_back(first_index,
                                        second_index,
                                        instance.get_matrix()[first_index][second_index]
                                        );
      already_added.insert(second_index);
    }
  }

  // Building Eulerian graph from the edges.
  undirected_graph<distance_t> eulerian_graph (eulerian_graph_edges);

  // Hierholzer's algorithm: building and joining closed tours with
  // vertices that still have adjacent edges.
  std::unordered_map<index_t, std::list<index_t>> eulerian_adjacency_list
    = eulerian_graph.get_adjacency_list();

  std::list<index_t> eulerian_path;
  eulerian_path.push_back(eulerian_adjacency_list.begin()->first);

  // Building and Joining tours as long as necessary.
  bool complete_tour;
  
  do{
    complete_tour = true;       // presumed complete
    std::list<index_t>::iterator new_tour_start;
    // Finding first element of eulerian_path that still has an
    // adjacent edge (if any).
    for(auto vertex = eulerian_path.begin();
        vertex != eulerian_path.end();
        ++vertex){
      if(eulerian_adjacency_list[*vertex].size() > 0){
        new_tour_start = vertex;
        complete_tour = false;
        break;
      }
    }

    if(!complete_tour){
      // Add new tour to initial eulerian path and check again.
      std::list<index_t> new_tour;
      index_t initial_vertex = *new_tour_start;
      index_t current_vertex = initial_vertex;
      index_t next_vertex;
      // Start building new tour.
      do{
        new_tour.push_back(current_vertex);
        // Find next vertex from any adjacent edge and remove used edge.
        next_vertex = eulerian_adjacency_list[current_vertex].front();
        eulerian_adjacency_list[current_vertex].pop_front();
        for(auto vertex = eulerian_adjacency_list[next_vertex].begin();
            vertex != eulerian_adjacency_list[next_vertex].end();
            ++vertex){
          if(*vertex == current_vertex){
            eulerian_adjacency_list[next_vertex].erase(vertex);
            break;
          }
        }
        current_vertex = next_vertex;
      } while(current_vertex != initial_vertex);

      // Adding new tour to existing eulerian path.
      eulerian_path.insert(new_tour_start, new_tour.begin(), new_tour.end());
    }
  }while(!complete_tour);    
    
  std::set<index_t> already_visited;
  std::list<index_t> tour;
  for(auto vertex = eulerian_path.cbegin();
      vertex != eulerian_path.cend();
      ++vertex){
    auto ret = already_visited.insert(*vertex);
    if(ret.second){
      // Vertex not already visited.
      tour.push_back(*vertex);
    }
  }
  return tour;
}
