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

#ifndef MATRIX_H
#define MATRIX_H
#include <iostream>
#include <vector>
#include "./typedefs.h"

template <class T>
class matrix{

private:
  std::size_t _size;
  std::vector<std::vector<T>> _inner_rep;
  
public:
  matrix():
    _size(0) {}
  
  matrix(std::vector<std::vector<T>> matrix_as_vector):
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

  std::size_t size() const{
    return _size;
  }

  void print() const{
    for(auto i = _inner_rep.cbegin(); i != _inner_rep.cend(); ++i){
      for(auto val = (*i).cbegin(); val != (*i).cend(); ++val){
        std::cout << *val << " ; ";
      }
      std::cout << std::endl;
    }
  }

  matrix<T> get_sub_matrix(const std::vector<index_t>& indices) const{
    std::vector<std::vector<T>> sub_matrix;
    for(auto i = indices.cbegin(); i != indices.cend(); ++i){
      std::vector<T> current_line;
      for(auto j = indices.cbegin(); j != indices.cend(); ++j){
        current_line.push_back(_inner_rep[*i][*j]);
      }
      sub_matrix.push_back(current_line);
    }
    return matrix<T> (sub_matrix);
  }

  T operator()(index_t i, index_t j) const{
    return _inner_rep[i][j];
  }

  void set(index_t i, index_t j, T value){
    _inner_rep[i][j] = value;
  }
};

#endif
