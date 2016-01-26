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

#ifndef HEURISTIC_H
#define HEURISTIC_H
#include<string>
#include<list>
#include "../structures/typedefs.h"
#include "../structures/tsp.h"

class heuristic{

public:
  virtual std::list<index_t> build_solution(const tsp& instance) = 0;

  virtual ~heuristic() {}

protected:
  heuristic();
};

#endif
