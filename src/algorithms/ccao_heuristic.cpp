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

std::list<unsigned> ccao_heuristic::build_solution(tsp& instance){
  // Using the symmetric problem derived from the general one.
  const matrix<unsigned> m = instance.get_symmetrized_matrix();

  // Initial tour with the convex hull of all vertices is turned into
  // a map to facilitate further insertions. 
  std::list<unsigned> convex_hull_tour
    = convex_hull(instance.get_places());

  std::set<unsigned> remaining_vertices;
  for(unsigned i = 0; i < instance.size(); ++i){
    remaining_vertices.insert(i);
  }
  
  std::map<unsigned, unsigned> tour;
  auto step = convex_hull_tour.begin();
  unsigned first_step = *step;
  unsigned current_step = *step;
  while((++step) != convex_hull_tour.end()){
    tour.emplace(current_step, *step);
    current_step = *step;
    remaining_vertices.erase(current_step);
  }
  tour.emplace(convex_hull_tour.back(), first_step);
  remaining_vertices.erase(first_step);

  // unsigned i = 0;
  while(!remaining_vertices.empty()){
    // Finding best place in the current tour for each remaining vertex
    // (inserting k after best_previous). Remembering best remaining
    // vertex to pick it in the end.
    unsigned best_vertex;
    unsigned best_previous;
    double vertex_min_value = std::numeric_limits<double>::max();
    for(auto k = remaining_vertices.begin();
        k != remaining_vertices.end();
        ++k){
      unsigned current_previous;
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
      unsigned current_next = tour.at(current_previous);
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
  
    // std::list<unsigned> tour_as_list;
    // unsigned first = tour.begin()->first;
    // tour_as_list.push_back(first);
    // unsigned next = tour.at(first);
    // while(next != first){
    //   tour_as_list.push_back(next);
    //   next = tour.at(next);
    // }
    // instance.log_to_file(tour_as_list, "convex_hull_" + std::to_string(++i) + ".json");
  }
  std::list<unsigned> tour_as_list;
  unsigned first = tour.begin()->first;
  tour_as_list.push_back(first);
  unsigned next = tour.at(first);
  while(next != first){
    tour_as_list.push_back(next);
    next = tour.at(next);
  }

  return tour_as_list;
}
