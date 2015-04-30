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

#ifndef TSP_H
#define TSP_H
#include <iostream>
#include <vector>
#include <list>
#include "./structures/matrix.h"

class tsp{
private:
  // Cost matrix
  const matrix<double> _matrix;
  
public:
  tsp(matrix<double> m);

  const matrix<double>& get_matrix() const;

  std::size_t size();

  double cost(const std::list<unsigned>& tour) const;
};

#endif
