/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include <fstream>
#include <random>
#include <set>

#include "algorithms/kruskal.h"
#include "algorithms/munkres.h"
#include "problems/tsp/heuristics/christofides.h"

std::list<index_t> christofides(const matrix<cost_t>& sym_matrix) {
  // The eulerian sub-graph further used is made of a minimum spanning
  // tree with a minimum weight perfect matching on its odd degree
  // vertices.

  // Compute symmetric graph from the matrix.
  auto sym_graph = undirected_graph<cost_t>(sym_matrix);

  // Work on a minimum spanning tree seen as a graph.
  auto mst_graph = minimum_spanning_tree(sym_graph);

  // Getting minimum spanning tree of associated graph under the form
  // of an adjacency list.
  std::unordered_map<index_t, std::list<index_t>> adjacency_list =
    mst_graph.get_adjacency_list();

  // Getting odd degree vertices from the minimum spanning tree.
  std::vector<index_t> mst_odd_vertices;
  for (const auto& adjacency : adjacency_list) {
    if (adjacency.second.size() % 2 == 1) {
      mst_odd_vertices.push_back(adjacency.first);
    }
  }

  // Getting corresponding matrix for the generated sub-graph.
  matrix<cost_t> sub_matrix = sym_matrix.get_sub_matrix(mst_odd_vertices);

  // EXPERIMENT! Use Blossom V to calculate a minimum weight perfect matching
  // to see if there are any significant gains to be had. The algorithm is 
  // described here:
  // https://pdfs.semanticscholar.org/930b/9f9c3bc7acf3277dd2361076d40ff03774b2.pdf
  
  // As this is only an experiment we write out a file of edges and have the 
  // original blossom5 code: 
  // http://pub.ist.ac.at/~vnk/software/blossom5-v2.05.src.tar.gz 
  // do the work.

  // The license of the blossom5 code is incompatible with the license used by
  // vroom in general, but as long as it is only used for research purpose it
  // is within bounds.

  // Hence, in order to build and run vroom on this branch you need to compile 
  // (and include on your PATH) the blossom5 executable yourself, and make sure
  // you are not using it in a commercial setting. 

  auto node_count = sub_matrix.size();
  auto edge_count = node_count * (node_count - 1);

  std::ofstream output;
  output.open("sub_matrix");
  output << node_count << " " << edge_count << std::endl;

  for (size_t i = 0; i < node_count; ++i)
    for (size_t j = 0; j < node_count; ++j)
      if (i != j)
        output << i << " " << j << " " << sym_matrix[i][j] << std::endl;

  output.close();

  std::system("blossom5 -e sub_matrix -w mwpm_final");

  std::ifstream input;
  input.open("mwpm_final");

  index_t i, j;
  std::unordered_map<index_t, index_t> mwpm_final;
  
  std::string skip;
  std::getline(input, skip); // skip first line
  while (!input.eof()) {
    input >> i >> j;
    mwpm_final[i] = j;
  }

  input.close();

  // Building eulerian graph.
  std::vector<edge<cost_t>> eulerian_graph_edges = mst_graph.get_edges();

  // Adding edges from minimum weight perfect matching (with the
  // original vertices index). Edges appear twice in matching so we
  // need to remember the one already added.
  std::set<index_t> already_added;
  for (const auto& edge : mwpm_final) {
    index_t first_index = mst_odd_vertices[edge.first];
    index_t second_index = mst_odd_vertices[edge.second];
    if (already_added.find(first_index) == already_added.end()) {
      eulerian_graph_edges.emplace_back(first_index,
                                        second_index,
                                        sym_matrix[first_index][second_index]);
      already_added.insert(second_index);
    }
    else
      std::cout << "already there: " << first_index << " " << second_index << std::endl;
  }

  // Building Eulerian graph from the edges.
  undirected_graph<cost_t> eulerian_graph(eulerian_graph_edges);
  assert(eulerian_graph.size() >= 2);

  // Hierholzer's algorithm: building and joining closed tours with
  // vertices that still have adjacent edges.
  std::unordered_map<index_t, std::list<index_t>> eulerian_adjacency_list =
    eulerian_graph.get_adjacency_list();

  std::list<index_t> eulerian_path;
  eulerian_path.push_back(eulerian_adjacency_list.begin()->first);

  // Building and joining tours as long as necessary.
  bool complete_tour;

  do {
    complete_tour = true; // presumed complete
    std::list<index_t>::iterator new_tour_start;
    // Finding first element of eulerian_path that still has an
    // adjacent edge (if any).
    for (auto vertex = eulerian_path.begin(); vertex != eulerian_path.end();
         ++vertex) {
      if (eulerian_adjacency_list[*vertex].size() > 0) {
        new_tour_start = vertex;
        complete_tour = false;
        break;
      }
    }

    if (!complete_tour) {
      // Add new tour to initial eulerian path and check again.
      std::list<index_t> new_tour;
      index_t initial_vertex = *new_tour_start;
      index_t current_vertex = initial_vertex;
      index_t next_vertex;
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

  std::set<index_t> already_visited;
  std::list<index_t> tour;
  for (const auto& vertex : eulerian_path) {
    auto ret = already_visited.insert(vertex);
    if (ret.second) {
      // Vertex not already visited.
      tour.push_back(vertex);
    }
  }

  return tour;
}
