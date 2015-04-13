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

tsp::tsp(std::vector<std::vector<unsigned>> matrix)
  :_matrix(matrix)
{
  unsigned size = matrix.size();
  // Checking for a square matrix
  for(auto line = matrix.cbegin(); line != matrix.cend(); ++line){
    if (line->size() != size){
      std::cout << "Error in input matrix, square matrix required!\n";
      exit(1);
    }
  }
}

std::vector<std::vector<unsigned>> tsp::get_matrix(){
  return _matrix;
}

std::size_t tsp::get_size(){
  return _matrix.size();
}
