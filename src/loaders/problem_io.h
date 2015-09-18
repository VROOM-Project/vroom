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

#ifndef PROBLEM_IO_H
#define PROBLEM_IO_H
#include<string>
#include<vector>
#include "../structures/matrix.h"

template <class T, class V> 
class problem_io{

public:
  virtual matrix<T> get_matrix() const = 0;

  virtual std::string get_route(const std::list<index_t>& tour) const = 0;

  virtual std::string get_route_geometry(const std::list<index_t>& tour) const = 0;

  virtual ~problem_io() {}

protected:
  problem_io() {}
};

#endif
