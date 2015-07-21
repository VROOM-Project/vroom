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
#include <chrono>
#include "../structures/matrix.h"
#include "../utils/logger.h"
#include "../structures/tsp.h"

class local_search{
private:
  tsp* _problem;
  matrix<unsigned> _matrix;
  std::map<unsigned, unsigned> _edges;

public:

  local_search(tsp* problem, std::list<unsigned> tour);

  unsigned relocate_step();

  unsigned perform_all_relocate_steps();

  unsigned two_opt_step();

  unsigned perform_all_two_opt_steps();

  unsigned or_opt_step();

  unsigned perform_all_or_opt_steps();
    
  std::list<unsigned> get_tour(unsigned first_index) const;
  
};

#endif
