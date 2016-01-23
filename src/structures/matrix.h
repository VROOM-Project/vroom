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
class line{
 public:
  virtual index_t size() const {
    return 0;
  }

  virtual T operator[](const index_t index) const {
    return 0;
  }
};

template <class T>
class matrix{
 public:
  virtual index_t size() const {
    return 0;
  };

  virtual line<T> operator[](const index_t index) const {
    return line<T>();
  }

  virtual matrix<T> get_sub_matrix(const std::vector<index_t>& indices) const {
    return matrix<T>();
  }

  virtual bool is_symmetric() const {
    return true;
  }
};

template <class T>
class static_line: public line<T>, private std::vector<T>{

  using parent = std::vector<T>;

public:
  using parent::size;
  using parent::operator[];

  static_line(std::size_t n) : parent(n) {
  }
};

template <class T>
class static_matrix : public matrix<T>, private std::vector<static_line<T>>{
  
  using parent = std::vector<static_line<T>>;
  
 public:
  using parent::size;
  using parent::operator[];
  
  static_matrix(std::size_t n): 
    parent(n, static_line<T>(n)){
  }
  
  matrix<T> get_sub_matrix(const std::vector<index_t>& indices) const{
    static_matrix<T> sub_matrix {indices.size()};
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

template <class T>
class virtual_euclidian_line: private line<T>{
 private:
  std::size_t _row;
  std::vector<std::pair<double, double>> _locations;

 public:
  std::size_t size() const {
    return _locations.size();
  }

  T operator[](index_t index) const {
     return sqrt(pow(_locations[row].first - _locations[index].first, 2) + pow(_locations[row].second - _locations[index].second, 2));
  }

  virtual_euclidian_line(std::vector<std::pair<double, double>> locations):
    _locations(locations){
  }

  void row(index_t row){
    _row = row;
  }
};


template <class T>
class virtual_euclidian_matrix: private matrix<T>{
 private:
  std::vector<std::pair<double, double>> _locations;
  virtual_euclidian_line<T> line;

 public:
  T operator[](index_t index) const{
    line.row(index);
    return line[index];
  }

  virtual_euclidian_matrix(std::vector<std::pair<double, double>> locations):
    _locations(locations),
    line(virtual_euclidian_line<T>(locations)){
  }

  matrix<T> get_sub_matrix(const std::vector<index_t>& indices) const{
    static_matrix<T> sub_matrix {indices.size()};
    for(std::size_t i = 0; i < indices.size(); ++i){
      for(std::size_t j = 0; j < indices.size(); ++j){
        sub_matrix[i][j] = (*this)[indices[i]][indices[j]];
      }
    }
    return sub_matrix;
  }

  bool is_symmetric() const{
    return true;
  }
};

#endif
