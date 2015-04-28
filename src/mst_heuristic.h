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

#ifndef MST_HEURISTIC_H
#define MST_HEURISTIC_H
#include <list>
#include <set>
#include "heuristic.h"
#include "tsp_sym.h"
#include "./structures/edge.h"
#include "./structures/undirected_graph.h"

// Simple heuristic based on a deep-search on a minimal spanning tree.
class mst_heuristic : public heuristic{
public:

  virtual std::list<unsigned> build_solution(tsp_sym& instance);
};

#endif
