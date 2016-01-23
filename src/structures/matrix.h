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
#include <algorithm>
#include <vector>
#include <cmath>
#include "./typedefs.h"
#include "../utils/exceptions.h"

template <class T>
class line{
 private:
  std::size_t _row;
  std::vector<std::pair<double, double>> _locations;

 public:
  std::size_t size() const {
    return _locations.size();
  }

  T operator[](index_t index) const {
    return (_row == index) ?
      3 * (std::numeric_limits<distance_t>::max() / 4):
      sqrt(std::pow(_locations[_row].first - _locations[index].first, 2)
           + std::pow(_locations[_row].second - _locations[index].second, 2));
  }

  line(std::vector<std::pair<double, double>> locations):
    _locations(locations){
  }

  void set_row(index_t row){
    _row = row;
  }

  std::pair<double, double> get_location(index_t index) const{
    return _locations[index];
  }
};

template <class T>
class matrix{
 private:
  line<T> _line;

 public:
  const line<T>& operator[](index_t index){
    _line.set_row(index);
    return _line;
  }

  matrix(std::vector<std::pair<double, double>> locations):
    _line(locations){
  }

  matrix(){}

  std::size_t size() const{
    return _line.size();
  }

  matrix<T> get_sub_matrix(const std::vector<index_t>& indices) const{
    std::vector<std::pair<double, double>> new_locations;
    std::transform(indices.begin(), indices.cend(),
                   std::back_inserter(new_locations),
                   [this](index_t i) {return _line.get_location(i);});
    return matrix<T>(new_locations);
  }

  bool is_symmetric() const{
    return true;
  }
};

#endif
