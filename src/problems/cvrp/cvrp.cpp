/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "cvrp.h"
#include "../../structures/vroom/input/input.h"

cvrp::cvrp(const input& input) : vrp(input) {
}

std::vector<std::list<index_t>> cvrp::solve(unsigned nb_threads) const {
  struct param {
    CLUSTERING_T type;
    INIT_T init;
    double regret_coeff;
  };

  auto start_clustering = std::chrono::high_resolution_clock::now();
  BOOST_LOG_TRIVIAL(info) << "[CVRP] Start clustering heuristic(s).";

  std::vector<param> parameters;
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 0});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.5});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 1});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 0});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 0.5});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 1});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::HIGHER_AMOUNT, 0});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::HIGHER_AMOUNT, 0.5});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::HIGHER_AMOUNT, 1});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 0});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 0.5});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 1});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 0});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 0.5});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 1});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 0});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 0.5});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 1});

  std::vector<clustering> clusterings;
  std::mutex clusterings_mutex;

  // Split the work among threads.
  std::vector<std::vector<std::size_t>>
    thread_ranks(nb_threads, std::vector<std::size_t>());
  for (std::size_t i = 0; i < parameters.size(); ++i) {
    thread_ranks[i % nb_threads].push_back(i);
  }

  auto run_clustering = [&](const std::vector<std::size_t>& param_ranks) {
    for (auto rank : param_ranks) {
      auto& p = parameters[rank];
      clustering c(_input, p.type, p.init, p.regret_coeff);

      std::lock_guard<std::mutex> guard(clusterings_mutex);
      clusterings.push_back(std::move(c));
    }
  };

  std::vector<std::thread> clustering_threads;

  for (std::size_t i = 0; i < nb_threads; ++i) {
    clustering_threads.emplace_back(run_clustering, thread_ranks[i]);
  }

  for (auto& t : clustering_threads) {
    t.join();
  }

  auto best_c = std::min_element(clusterings.begin(), clusterings.end());

  std::string strategy =
    (best_c->type == CLUSTERING_T::PARALLEL) ? "parallel" : "sequential";
  std::string init_str;
  switch (best_c->init) {
  case INIT_T::NONE:
    init_str = "none";
    break;
  case INIT_T::HIGHER_AMOUNT:
    init_str = "higher_amount";
    break;
  case INIT_T::NEAREST:
    init_str = "nearest";
    break;
  }
  BOOST_LOG_TRIVIAL(trace) << "Best clustering:" << strategy << ";" << init_str
                           << ";" << best_c->regret_coeff << ";"
                           << best_c->clusters.size() << ";"
                           << best_c->assigned_jobs << ";"
                           << best_c->edges_cost;

  auto end_clustering = std::chrono::high_resolution_clock::now();

  auto clustering_computing_time =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_clustering -
                                                          start_clustering)
      .count();

  BOOST_LOG_TRIVIAL(info) << "[CVRP] Done, took " << clustering_computing_time
                          << " ms.";

  BOOST_LOG_TRIVIAL(info) << "[CVRP] Launching TSPs ";

  auto nb_tsp = best_c->clusters.size();

  // Vector of TSP solutions as lists.
  std::vector<std::list<index_t>> tsp_sols(nb_tsp, std::list<index_t>());

  // Run TSP solving for a list of clusters in turn, each with
  // provided number of threads.
  auto run_tsp = [&](const std::vector<unsigned>& cluster_ranks,
                     unsigned tsp_threads) {
    for (auto cl_rank : cluster_ranks) {
      tsp p(_input, best_c->clusters[cl_rank], cl_rank);

      tsp_sols[cl_rank] = p.solve(tsp_threads)[0];
    }
  };

  std::vector<std::thread> tsp_threads;

  if (nb_tsp <= nb_threads) {
    // We launch nb_tsp TSP solving and each of them can be
    // multi-threaded.
    std::vector<unsigned> thread_per_tsp(nb_tsp, 0);
    for (std::size_t i = 0; i < nb_threads; ++i) {
      ++thread_per_tsp[i % nb_tsp];
    }

    for (unsigned i = 0; i < nb_tsp; ++i) {
      std::vector<unsigned> cluster_ranks({i});
      tsp_threads.emplace_back(run_tsp, cluster_ranks, thread_per_tsp[i]);
    }
  } else {
    // We launch nb_threads threads, each one solving several TSP.
    std::vector<std::vector<unsigned>>
    ranks_per_thread(nb_threads, std::vector<unsigned>());
    for (std::size_t i = 0; i < nb_tsp; ++i) {
      ranks_per_thread[i % nb_threads].push_back(i);
    }

    for (std::size_t i = 0; i < nb_threads; ++i) {
      tsp_threads.emplace_back(run_tsp, ranks_per_thread[i], 1);
    }
  }

  for (auto& t : tsp_threads) {
    t.join();
  }

  auto end_tsps = std::chrono::high_resolution_clock::now();
  auto tsp_computing_time =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_tsps -
                                                          end_clustering)
      .count();

  BOOST_LOG_TRIVIAL(info) << "[CVRP] Done with TSPs, took "
                          << tsp_computing_time << " ms.";

  return tsp_sols;
}
