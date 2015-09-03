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

#include "ccao_heuristic.h"

std::list<index_t> ccao_heuristic::build_solution(tsp_sym& instance){
  const matrix<distance_t>& m = instance.get_matrix();

  // Initial tour with the convex hull of all vertices is turned into
  // a map to facilitate further insertions. 
  std::list<index_t> convex_hull_tour
    = convex_hull(instance.get_locations());

  std::set<index_t> remaining_vertices;
  for(index_t i = 0; i < instance.size(); ++i){
    remaining_vertices.insert(i);
  }
  
  std::map<index_t, index_t> tour;
  auto step = convex_hull_tour.begin();
  index_t first_step = *step;
  index_t current_step = *step;
  while((++step) != convex_hull_tour.end()){
    tour.emplace(current_step, *step);
    current_step = *step;
    remaining_vertices.erase(current_step);
  }
  tour.emplace(convex_hull_tour.back(), first_step);
  remaining_vertices.erase(first_step);

  while(!remaining_vertices.empty()){
    // Finding best place in the current tour for each remaining vertex
    // (inserting k after best_previous). Remembering best remaining
    // vertex to pick it in the end.
    index_t best_vertex;
    index_t best_previous;
    double vertex_min_value = std::numeric_limits<double>::max();
    for(auto k = remaining_vertices.begin();
        k != remaining_vertices.end();
        ++k){
      index_t current_previous = 0; // Initialization actually never used.
      // Should be signed in case of non-metric instance (where
      // triangular inequality doesn't hold).
      long int place_min_value = std::numeric_limits<long int>::max();
      for(auto e = tour.begin(); e != tour.end(); ++e){
        long int place_current_value
          = m(e->first, *k) + m(*k, e->second) - m(e->first, e->second);
        if(place_current_value < place_min_value){
          current_previous = e->first;
          place_min_value = place_current_value;
        }
      }
      index_t current_next = tour.at(current_previous);
      double vertex_current_value
        = (double) (m(current_previous, *k) + m(*k, current_next))
        / m(current_previous, current_next);
      if(vertex_current_value < vertex_min_value){
        best_vertex = *k;
        best_previous = current_previous;
        vertex_min_value = vertex_current_value;
      }
    }

    // Perform addition of best vertex and remove from remaining.
    tour.emplace(best_vertex, tour.at(best_previous));
    tour.at(best_previous) = best_vertex;
    remaining_vertices.erase(best_vertex);
  }
  std::list<index_t> tour_as_list;
  index_t first = tour.begin()->first;
  tour_as_list.push_back(first);
  index_t next = tour.at(first);
  while(next != first){
    tour_as_list.push_back(next);
    next = tour.at(next);
  }

  return tour_as_list;
}
