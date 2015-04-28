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

#include "matrix.h"

matrix::matrix(std::vector<std::vector<unsigned>> matrix_as_vector):
  _size(matrix_as_vector.size()),
  _inner_rep(matrix_as_vector)
{
  // Checking for a square matrix
  for(auto line = matrix_as_vector.cbegin();
      line != matrix_as_vector.cend();
      ++line){
    if (line->size() != _size){
      std::cout << "Error in input matrix, square matrix required!\n";
      exit(1);
    }
  }
}

std::size_t matrix::size() const{
  return _size;
}

void matrix::print() const{
  for(auto i = _inner_rep.cbegin(); i != _inner_rep.cend(); ++i){
    for(auto val = (*i).cbegin(); val != (*i).cend(); ++val){
      std::cout << *val << " ; ";
    }
    std::cout << std::endl;
  }
}

matrix matrix::get_sub_matrix(const std::vector<unsigned>& indices) const{
  std::vector<std::vector<unsigned>> sub_matrix;
  for(auto i = indices.cbegin(); i != indices.cend(); ++i){
    std::vector<unsigned> current_line;
    for(auto j = indices.cbegin(); j != indices.cend(); ++j){
      current_line.push_back(_inner_rep[*i][*j]);
    }
    sub_matrix.push_back(current_line);
  }
  return matrix (sub_matrix);
}

unsigned matrix::operator()(unsigned i, unsigned j) const{
  return _inner_rep[i][j];
}
