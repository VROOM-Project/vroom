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
