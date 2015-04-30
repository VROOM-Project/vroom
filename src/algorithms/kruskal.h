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

#ifndef KRUSKAL_H
#define KRUSKAL_H
#include <list>
#include <algorithm>            // sort
#include "../structures/edge.h"
#include "../structures/undirected_graph.h"

template <class T>
undirected_graph<T> minimum_spanning_tree(const undirected_graph<T>& graph){
  // We just need the edges from original graph
  std::list<edge<T>> edges = graph.get_edges();
  
  // First sorting edges by weight
  struct {
    bool operator()(const edge<T> &a, const edge<T> &b){
      return a.get_weight() < b.get_weight();
    }   
  } comp;
  edges.sort(comp);

  // Empty list to initialize the tree's edges
  std::list<edge<T>> mst;

  // During Kruskal algorithm, the number of connected components will
  // decrease until we obtain a single component (the final tree). We
  // use the smallest vertex as a representative of connected
  // components.
  std::unordered_map<unsigned, unsigned> representative;
  for(unsigned i = 0; i < graph.size(); ++i){
    representative.emplace(i, i);
  }

  for(auto edge = edges.cbegin(); edge != edges.cend(); ++edge){
    unsigned first_vertex = edge->get_first_vertex();
    unsigned second_vertex = edge->get_second_vertex();

    unsigned first_rep = representative[first_vertex];
    unsigned second_rep = representative[second_vertex];
    if(first_rep != second_rep){
      // Adding current edge won't create a cycle as vertices are in
      // separate connected componentes.
      mst.push_back(*edge);
      // Both vertices are now in the same connected component,
      // setting new representative for all elements of second
      // component.
      for(auto it = representative.begin();
          it != representative.end();
          ++it){
        if(it->second == second_rep){
          it->second = first_rep;
        }
      }      
    }
  }

  undirected_graph<T> mst_as_graph (mst);
  return mst_as_graph;
}

#endif
