#ifndef MATRIX_H
#define MATRIX_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

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
};

#endif
