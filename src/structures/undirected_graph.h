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

#ifndef UNDIRECTED_GRAPH_H
#define UNDIRECTED_GRAPH_H
#include <list>
#include <set>
#include <unordered_map>
#include <algorithm>
#include "edge.h"
#include "matrix.h"

class undirected_graph{

private:
  unsigned _size;
  // Embedding two representations for different uses depending on
  // context.
  std::list<edge> _edges;
  std::unordered_map<unsigned, std::set<unsigned>> _adjacency_list;

public:
  undirected_graph(matrix m);

  undirected_graph(std::list<edge> edges);

  std::list<edge> get_edges() const;

  std::unordered_map<unsigned, std::set<unsigned>> get_adjacency_list() const;

  undirected_graph get_minimum_spanning_tree();

  void print_edges() const;

  void print_adjacency_list() const;
};

#endif
