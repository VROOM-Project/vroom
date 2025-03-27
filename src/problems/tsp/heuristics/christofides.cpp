/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>
#include <unordered_set>

#include "algorithms/kruskal.h"
#include "algorithms/munkres.h"
#include "problems/tsp/heuristics/christofides.h"

namespace vroom::tsp {

std::list<Index> christofides(const Matrix<UserCost>& sym_matrix) {
  // The eulerian sub-graph further used is made of a minimum spanning
  // tree with a minimum weight perfect matching on its odd degree
  // vertices.

  // Compute symmetric graph from the matrix.
  const auto sym_graph = utils::UndirectedGraph<UserCost>(sym_matrix);

  // Work on a minimum spanning tree seen as a graph.
  const auto mst_graph = utils::minimum_spanning_tree(sym_graph);

  // Getting minimum spanning tree of associated graph under the form
  // of an adjacency list.
  const std::unordered_map<Index, std::list<Index>> adjacency_list =
    mst_graph.get_adjacency_list();

  // Getting odd degree vertices from the minimum spanning tree.
  std::vector<Index> mst_odd_vertices;
  for (const auto& [index, list] : adjacency_list) {
    if (list.size() % 2 == 1) {
      mst_odd_vertices.push_back(index);
    }
  }

  // Getting corresponding matrix for the generated sub-graph.
  const Matrix<UserCost> sub_matrix =
    sym_matrix.get_sub_matrix(mst_odd_vertices);

  // Computing minimum weight perfect matching.
  std::unordered_map<Index, Index> mwpm =
    utils::minimum_weight_perfect_matching(sub_matrix);

  // Storing those edges from mwpm that are coherent regarding
  // symmetry (y -> x whenever x -> y). Remembering the rest of them
  // for further use. Edges are not doubled in mwpm_final.
  std::unordered_map<Index, Index> mwpm_final;
  std::vector<Index> wrong_vertices;

  for (const auto& [source, target] : mwpm) {
    if (mwpm.at(target) == source) {
      mwpm_final.emplace(std::min(source, target), std::max(source, target));
    } else {
      wrong_vertices.push_back(source);
    }
  }

  if (!wrong_vertices.empty()) {
    const std::unordered_map<Index, Index> remaining_greedy_mwpm =
      utils::greedy_symmetric_approx_mwpm(
        sub_matrix.get_sub_matrix(wrong_vertices));

    // Adding edges obtained with greedy algo for the missing vertices
    // in mwpm_final.
    for (const auto& [source, target] : remaining_greedy_mwpm) {
      mwpm_final.emplace(std::min(wrong_vertices[source],
                                  wrong_vertices[target]),
                         std::max(wrong_vertices[source],
                                  wrong_vertices[target]));
    }
  }

  // Building eulerian graph.
  std::vector<utils::Edge<UserCost>> eulerian_graph_edges =
    mst_graph.get_edges();

  // Adding edges from minimum weight perfect matching (with the
  // original vertices index). Edges appear twice in matching so we
  // need to remember the one already added.
  std::unordered_set<Index> already_added;
  for (const auto& [source, target] : mwpm_final) {
    const Index first_index = mst_odd_vertices[source];
    const Index second_index = mst_odd_vertices[target];
    if (!already_added.contains(first_index)) {
      eulerian_graph_edges.emplace_back(first_index,
                                        second_index,
                                        sym_matrix[first_index][second_index]);
      already_added.insert(second_index);
    }
  }

  // Building Eulerian graph from the edges.
  const utils::UndirectedGraph<UserCost> eulerian_graph(
    std::move(eulerian_graph_edges));
  assert(eulerian_graph.size() >= 2);

  // Hierholzer's algorithm: building and joining closed tours with
  // vertices that still have adjacent edges.
  std::unordered_map<Index, std::list<Index>> eulerian_adjacency_list =
    eulerian_graph.get_adjacency_list();

  std::list<Index> eulerian_path;
  eulerian_path.push_back(eulerian_adjacency_list.begin()->first);

  // Building and joining tours as long as necessary.
  bool complete_tour;

  do {
    complete_tour = true; // presumed complete
    std::list<Index>::iterator new_tour_start;
    // Finding first element of eulerian_path that still has an
    // adjacent edge (if any).
    for (auto vertex = eulerian_path.begin(); vertex != eulerian_path.end();
         ++vertex) {
      if (!eulerian_adjacency_list[*vertex].empty()) {
        new_tour_start = vertex;
        complete_tour = false;
        break;
      }
    }

    if (!complete_tour) {
      // Add new tour to initial eulerian path and check again.
      std::list<Index> new_tour;
      const Index initial_vertex = *new_tour_start;
      Index current_vertex = initial_vertex;
      Index next_vertex;
      // Start building new tour.
      do {
        new_tour.push_back(current_vertex);
        // Find next vertex from any adjacent edge and remove used edge.
        next_vertex = eulerian_adjacency_list[current_vertex].front();
        eulerian_adjacency_list[current_vertex].pop_front();
        for (auto vertex = eulerian_adjacency_list[next_vertex].begin();
             vertex != eulerian_adjacency_list[next_vertex].end();
             ++vertex) {
          if (*vertex == current_vertex) {
            eulerian_adjacency_list[next_vertex].erase(vertex);
            break;
          }
        }
        current_vertex = next_vertex;
      } while (current_vertex != initial_vertex);

      // Adding new tour to existing eulerian path.
      eulerian_path.insert(new_tour_start, new_tour.begin(), new_tour.end());
    }
  } while (!complete_tour);

  std::unordered_set<Index> already_visited;
  std::list<Index> tour;
  for (const auto& vertex : eulerian_path) {
    const auto [iter, insert_ok] = already_visited.insert(vertex);
    if (insert_ok) {
      // Vertex not already visited.
      tour.push_back(vertex);
    }
  }
  return tour;
}

} // namespace vroom::tsp
