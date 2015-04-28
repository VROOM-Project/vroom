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

#include "undirected_graph.h"

undirected_graph::undirected_graph(matrix m):
  _size(m.size())
{
  bool matrix_ok = true;
  for(unsigned i = 0; i < _size; ++i){
    matrix_ok &= (m(i, i) == 0);
    for(unsigned j = i + 1; j < _size; ++j){
      matrix_ok &= (m(i, j) == m(j, i));
      _edges.emplace_front(i, j, m(i, j));
      _adjacency_list[i].insert(j);
      _adjacency_list[j].insert(i);
    }
  }
  if(!matrix_ok){
    std::cout << "Error in input matrix, symmetric matrix required!\n";
    exit(1);
  }
}

undirected_graph::undirected_graph(std::list<edge> edges):
  _edges(edges){
  for(auto edge = _edges.cbegin();
      edge != _edges.cend();
      ++edge){
    unsigned first = edge->get_first_vertex();
    unsigned second = edge->get_second_vertex();
    
    _adjacency_list[first].insert(second);
    _adjacency_list[second].insert(first);
  }
  _size = _adjacency_list.size();
}

std::size_t undirected_graph::size() const{
  return _size;
}

std::list<edge> undirected_graph::get_edges() const{
  return _edges;
}

std::unordered_map<unsigned, std::set<unsigned>> undirected_graph::get_adjacency_list() const{
  return _adjacency_list;
}

void undirected_graph::print_edges() const{
  for(auto edge = _edges.cbegin(); edge != _edges.cend(); ++edge){
    edge->log();
    std::cout << " ; ";
  }
  std::cout << std::endl;
}

void undirected_graph::print_adjacency_list() const{
  for(auto it = _adjacency_list.cbegin();
      it != _adjacency_list.cend();
      ++it){
    std::cout << it->first << "->(";
    for(auto element = (it->second).cbegin();
        element != (it->second).cend();
        ++element){
      std::cout << *element << " ; ";
    }
    std::cout << ") \n";
  }
}
