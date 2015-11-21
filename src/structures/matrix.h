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
#include "../utils/exceptions.h"

template <class T>
class line: private std::vector<T>{

  using parent = std::vector<T>;

public:
  using parent::size;
  using parent::operator[];

  line(std::size_t n) : parent(n) {
  }
};

template <class T>
class matrix : private std::vector<line<T>>{
  
  using parent = std::vector<line<T>>;
  
 public:
  using parent::size;
  using parent::operator[];
  
  matrix(std::size_t n): 
    parent(n, line<T>(n)){
  }
  
  matrix<T> get_sub_matrix(const std::vector<index_t>& indices) const{
    matrix<T> sub_matrix {indices.size()};
    for(std::size_t i = 0; i < indices.size(); ++i){
      for(std::size_t j = 0; j < indices.size(); ++j){
        sub_matrix[i][j] = (*this)[indices[i]][indices[j]];
      }
    }
    return sub_matrix;
  }

  bool is_symmetric() const{
    bool is_sym = true;
    std::size_t i = 0;
    while(is_sym and (i < this->size())){
      std::size_t j = i + 1;
      while(is_sym and (j < this->size())){
        is_sym &= ((*this)[i][j] == (*this)[j][i]);
        ++j;
      }
      ++i;
    }
    return is_sym;
  }
};

#endif
