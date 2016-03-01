#ifndef KRUSKAL_H
#define KRUSKAL_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>
#include <algorithm>            // sort
#include "../structures/typedefs.h"
#include "../structures/edge.h"
#include "../structures/undirected_graph.h"

template <class T>
undirected_graph<T> minimum_spanning_tree(const undirected_graph<T>& graph){
  // We just need the edges from original graph.
  std::vector<edge<T>> edges = graph.get_edges();
  
  // First sorting edges by weight.
  std::sort(edges.begin(), edges.end(),
            [] (const auto &a, const auto &b){
              return a.get_weight() < b.get_weight();
            });

  // Storing the edges of the minimum spanning tree.
  std::vector<edge<T>> mst;

  // During Kruskal algorithm, the number of connected components will
  // decrease until we obtain a single component (the final tree). We
  // use the smallest vertex as a representative of connected
  // components.
  std::unordered_map<index_t, index_t> representative;
  for(index_t i = 0; i < graph.size(); ++i){
    representative.emplace(i, i);
  }

  for(const auto& edge: edges){
    index_t first_vertex = edge.get_first_vertex();
    index_t second_vertex = edge.get_second_vertex();

    index_t first_rep = representative[first_vertex];
    index_t second_rep = representative[second_vertex];
    if(first_rep != second_rep){
      // Adding current edge won't create a cycle as vertices are in
      // separate connected componentes.
      mst.push_back(edge);
      // Both vertices are now in the same connected component,
      // setting new representative for all elements of second
      // component.
      for(auto& e: representative){
        if(e.second == second_rep){
          e.second = first_rep;
        }
      }      
    }
  }

  return undirected_graph<T> (mst);
}

#endif
