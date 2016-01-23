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
#include <unordered_map>
#include <vector>
#include "../structures/typedefs.h"
#include "../structures/matrix.h"
#include "../structures/tsp.h"

class local_search{
private:
  const matrix<distance_t> _matrix;
  const bool _symmetric_matrix;
  std::vector<index_t> _edges;
  bool _verbose;

public:

  local_search(const tsp& problem,
               const std::list<index_t>& tour,
               bool verbose);

  distance_t relocate_step();

  distance_t perform_all_relocate_steps();

  distance_t avoid_loop_step();

  distance_t perform_all_avoid_loop_steps();

  distance_t two_opt_step();

  distance_t perform_all_two_opt_steps();

  distance_t or_opt_step();

  distance_t perform_all_or_opt_steps();
    
  std::list<index_t> get_tour(index_t first_index) const;
  
};

#endif
