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

#include "local_search.h"

local_search::local_search(matrix<unsigned> matrix, std::list<unsigned> tour):
  _matrix(matrix){
  auto place = tour.cbegin();
  unsigned first_index = *place;
  unsigned current_index = first_index;
  unsigned last_index = first_index;
  ++place;
  while(place != tour.cend()){
    current_index = *place;
    _edges.emplace(last_index, current_index);
    last_index = current_index;
    ++place;
  }
  _edges.emplace(last_index, first_index);
}

unsigned local_search::two_opt_step(){
  unsigned gain = 0;
  for(auto edge_1 = _edges.cbegin(); edge_1 != _edges.cend(); ++edge_1){
    auto edge_2 = edge_1;
    ++edge_2;
    bool amelioration_found = false;
    for(; edge_2 != _edges.cend(); ++edge_2){
      int current_diff = _matrix(edge_1->first, edge_1->second)
        + _matrix(edge_2->first, edge_2->second)
        - _matrix(edge_1->first, edge_2->first)
        - _matrix(edge_1->second, edge_2->second);
      if(current_diff > 0){
        amelioration_found = true;
        gain = current_diff;
        std::cout << "Gain de :" << current_diff << std::endl;
        // std::cout << edge_1->first
        //           << "-" << _matrix(edge_1->first, edge_1->second) << "->"
        //           << edge_1->second
        //           << std::endl;
        // std::cout << edge_2->first
        //           << "-" << _matrix(edge_2->first, edge_2->second) << "->"
        //           << edge_2->second
        //           << std::endl;

        // Storing part of the tour that needs to be reversed.
        std::list<unsigned> to_reverse;
        for(unsigned current = edge_1->second;
            current != edge_2->first;
            current = _edges.at(current)){
          to_reverse.push_back(current);
        }
        // Performing exchange.
        unsigned current = edge_2->first;
        unsigned last = edge_2->second;
        _edges.at(edge_1->first) = current;
        for(auto next = to_reverse.rbegin(); next != to_reverse.rend(); ++next){
          _edges.at(current) = *next;
          current = *next;
       }
        _edges.at(current) = last;
        break;
      }
    }
    if(amelioration_found){
      break;
    }
  }

  return gain;
}

std::list<unsigned> local_search::get_tour(unsigned first_index) const{
  std::list<unsigned> tour;
  tour.push_back(first_index);
  unsigned next_index = _edges.at(first_index);
  while(next_index != first_index){
    tour.push_back(next_index);
    next_index = _edges.at(next_index);
  }
  return tour;
}
