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

#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H
#include <list>
#include <map>
#include "../structures/matrix.h"

class local_search{
private:
  matrix<unsigned> _matrix;
  std::map<unsigned, unsigned> _edges;

public:

  local_search(matrix<unsigned> matrix, std::list<unsigned> tour);

  std::list<unsigned> get_tour(unsigned first_index) const;
  
};

#endif
