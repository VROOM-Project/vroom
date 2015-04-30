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

#include "tsp.h"

tsp::tsp(matrix<double> m)
  :_matrix(m) {}

const matrix<double>& tsp::get_matrix() const{
  return _matrix;
}

std::size_t tsp::size(){
  return _matrix.size();
}

double tsp::cost(const std::list<unsigned>& tour) const{
  double cost = 0;
  unsigned init_step;

  auto step = tour.cbegin();
  if(tour.size() > 0){
    init_step = *step;
  }

  unsigned previous_step = init_step;
  ++step;
  for(; step != tour.cend(); ++step){
    cost += _matrix(previous_step, *step);
    previous_step = *step;
  }
  if(tour.size() > 0){
    cost += _matrix(previous_step, init_step);
  }
  return cost;
}
  
