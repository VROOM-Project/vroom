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
  const index_t _row;
  const std::vector<std::pair<double, double>>* const _locations_ptr;

 public:
  line(index_t row,
       const std::vector<std::pair<double, double>>& locations):
    _row(row),
    _locations_ptr(&locations){
  }

  T operator[](index_t index) const {
    return (_row == index) ?
      3 * (std::numeric_limits<distance_t>::max() / 4):
      sqrt(std::pow((*_locations_ptr)[_row].first - (*_locations_ptr)[index].first, 2)
           + std::pow((*_locations_ptr)[_row].second - (*_locations_ptr)[index].second, 2));
  }
};

template <class T>
class matrix{
 private:
  std::vector<std::pair<double, double>> _locations;
  std::vector<line<T>> _lines;

 public:
  const line<T>& operator[](index_t index) const{
    return _lines[index];
  }

  matrix(std::vector<std::pair<double, double>> locations):
    _locations(locations){
    for(index_t i = 0; i < _locations.size(); ++i){
      _lines.emplace_back(i, _locations);
    }
  }

  matrix(){}

  std::size_t size() const{
    return _locations.size();
  }

  matrix<T> get_sub_matrix(const std::vector<index_t>& indices) const{
    std::vector<std::pair<double, double>> new_locations;
    std::transform(indices.begin(), indices.cend(),
                   std::back_inserter(new_locations),
                   [this](index_t i) {return _locations[i];});
    return matrix<T>(new_locations);
  }

  bool is_symmetric() const{
    return true;
  }
};

#endif
