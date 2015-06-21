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

#ifndef MATRIX_LOADER_H
#define MATRIX_LOADER_H
#include<string>
#include<vector>
#include "matrix.h"

template <class T, class V> 
class matrix_loader{

public:
  virtual matrix<T> load_matrix(const std::vector<std::pair<V, V>>& places) = 0;

  virtual ~matrix_loader() {}

protected:
  matrix_loader() {}

  static unsigned nint(double x){
    return (unsigned) (x + 0.5);
  }
};

#endif
