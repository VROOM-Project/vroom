/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>

#include "algorithms/kruskal.h"

namespace vroom::utils {

template <class T>
UndirectedGraph<T> minimum_spanning_tree(const UndirectedGraph<T>& graph) {
  // We just need the edges from original graph.
  std::vector<Edge<T>> edges = graph.get_edges();

  // First sorting edges by weight.
  std::ranges::sort(edges, [](const auto& a, const auto& b) {
    return a.get_weight() < b.get_weight();
  });

  // Storing the edges of the minimum spanning tree.
  std::vector<Edge<T>> mst;
  mst.reserve(graph.size() - 1);

  // During Kruskal algorithm, the number of connected components will
  // decrease until we obtain a single component (the final tree). We
  // use the smallest vertex as a representative of connected
  // components.
  std::vector<Index> representative(graph.size());
  std::iota(representative.begin(), representative.end(), 0);

  for (const auto& edge : edges) {
    const Index first_vertex = edge.get_first_vertex();
    const Index second_vertex = edge.get_second_vertex();

    const Index first_rep = representative[first_vertex];
    const Index second_rep = representative[second_vertex];
    if (first_rep != second_rep) {
      // Adding current edge won't create a cycle as vertices are in
      // separate connected components.
      mst.push_back(edge);
      // Both vertices are now in the same connected component,
      // setting new representative for all elements of second
      // component. Relies on first_vertex < second_vertex (see edge
      // ctor).
      for (auto& e : representative) {
        if (e == second_rep) {
          e = first_rep;
        }
      }
    }
  }
  assert(mst.size() == graph.size() - 1);

  return UndirectedGraph<T>(std::move(mst));
}

template UndirectedGraph<UserCost>
minimum_spanning_tree(const UndirectedGraph<UserCost>& graph);

} // namespace vroom::utils
