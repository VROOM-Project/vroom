/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "tsp_strategy.h"

void solve_atsp(const cl_args_t& cl_args){
  // Store timings.
  timing_t computing_times;

  // Building problem object with embedded table request.
  auto start_problem_build = std::chrono::high_resolution_clock::now();

  BOOST_LOG_TRIVIAL(info) 
    << "[Matrix] Start matrix computing and problem loading.";

  tsp asymmetric_tsp (cl_args);

  auto end_problem_build = std::chrono::high_resolution_clock::now();

  computing_times.matrix_loading =
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_problem_build - start_problem_build).count();

  BOOST_LOG_TRIVIAL(info) << "[Matrix] Done, took "
                          << computing_times.matrix_loading << " ms.";

  // Applying heuristic.
  auto start_heuristic = std::chrono::high_resolution_clock::now();
  BOOST_LOG_TRIVIAL(info) 
    << "[Heuristic] Start heuristic on symmetrized problem.";

  std::unique_ptr<heuristic> christo_h = std::make_unique<christo_heuristic>();
  std::list<index_t> christo_sol
    = christo_h->build_solution(asymmetric_tsp);
  distance_t christo_cost = asymmetric_tsp.symmetrized_cost(christo_sol);

  auto end_heuristic = std::chrono::high_resolution_clock::now();

  computing_times.heuristic = 
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_heuristic - start_heuristic).count();

  BOOST_LOG_TRIVIAL(info) << "[Heuristic] Done, took "
                          << computing_times.heuristic << " ms.";

  BOOST_LOG_TRIVIAL(info) << "[Heuristic] Symmetric solution cost is "
                          << christo_cost << ".";


  // Local search on symmetric problem.
  // Applying deterministic, fast local search to improve the current
  // solution in a small amount of time. All possible moves for the
  // different neighbourhoods are performed, stopping when reaching a
  // local minima.
  auto start_sym_local_search = std::chrono::high_resolution_clock::now();
  BOOST_LOG_TRIVIAL(info) 
    << "[Local search] Start local search on symmetrized problem.";
  BOOST_LOG_TRIVIAL(info) 
    << "[Local search] Using " << cl_args.nb_threads << " thread(s).";

  local_search sym_ls (asymmetric_tsp.get_symmetrized_matrix(),
                       true,    // Symmetrized problem.
                       christo_sol,
                       cl_args.nb_threads);

  distance_t sym_two_opt_gain = 0;
  distance_t sym_relocate_gain = 0;
  distance_t sym_or_opt_gain = 0;

  do{
    // All possible 2-opt moves.
    sym_two_opt_gain = sym_ls.perform_all_two_opt_steps();

    // All relocate moves.
    sym_relocate_gain = sym_ls.perform_all_relocate_steps();

    // All or-opt moves.
    sym_or_opt_gain = sym_ls.perform_all_or_opt_steps();
  }while((sym_two_opt_gain > 0) 
         or (sym_relocate_gain > 0) 
         or (sym_or_opt_gain > 0));

  // Default for first input location.
  index_t first_loc_index = 0;
  if(!cl_args.force_start and cl_args.force_end){
    // Requiring the tour to be described from the "forced" end
    // location.
    first_loc_index = asymmetric_tsp.size() - 1;
  }
  std::list<index_t> current_sol = sym_ls.get_tour(first_loc_index);
  auto current_cost = asymmetric_tsp.symmetrized_cost(current_sol);

  auto end_sym_local_search = std::chrono::high_resolution_clock::now();

  auto sym_local_search_duration 
    = std::chrono::duration_cast<std::chrono::milliseconds>
    (end_sym_local_search - start_sym_local_search).count();
  BOOST_LOG_TRIVIAL(info) << "[Local search] Done, took "
                          << sym_local_search_duration << " ms.";

  BOOST_LOG_TRIVIAL(info) << "[Local search] Symmetric solution cost is now "
                          << current_cost
                          << " (" 
                          << std::fixed << std::setprecision(2)
                          << 100 *(((double) current_cost) / christo_cost - 1)
                          << "%).";

  auto asym_local_search_duration = 0;

  if(!asymmetric_tsp.is_symmetric()){
    // Back to the asymmetric problem, picking the best way.
    std::list<index_t> reverse_current_sol (current_sol);
    reverse_current_sol.reverse();
    distance_t direct_cost = asymmetric_tsp.cost(current_sol);
    distance_t reverse_cost = asymmetric_tsp.cost(reverse_current_sol);

    // Cost reference after symmetric local search.
    distance_t sym_ls_cost = std::min(direct_cost, reverse_cost);

    // Local search on asymmetric problem.
    local_search asym_ls (asymmetric_tsp.get_matrix(),
                          false, // Not the symmetrized problem.
                          (direct_cost <= reverse_cost) ? 
                          current_sol: reverse_current_sol,
                          cl_args.nb_threads);

    auto start_asym_local_search = std::chrono::high_resolution_clock::now();
    BOOST_LOG_TRIVIAL(info) 
      << "[Asym. local search] Back to asymmetric problem, initial solution cost is "
      << sym_ls_cost << ".";
  
    BOOST_LOG_TRIVIAL(info) 
      << "[Asym. local search] Start local search on asymmetric problem.";

    BOOST_LOG_TRIVIAL(info) 
      << "[Asym. local search] Using " << cl_args.nb_threads << " thread(s).";

    distance_t asym_two_opt_gain = 0;
    distance_t asym_relocate_gain = 0;
    distance_t asym_or_opt_gain = 0;
    distance_t asym_avoid_loops_gain = 0;
  
    do{
      // All avoid-loops moves.
      asym_avoid_loops_gain = asym_ls.perform_all_avoid_loop_steps();

      // All possible 2-opt moves.
      asym_two_opt_gain = asym_ls.perform_all_asym_two_opt_steps();

      // All relocate moves.
      asym_relocate_gain = asym_ls.perform_all_relocate_steps();

      // All or-opt moves.
      asym_or_opt_gain = asym_ls.perform_all_or_opt_steps();
    }while((asym_two_opt_gain > 0) 
           or (asym_relocate_gain > 0) 
           or (asym_or_opt_gain > 0)
           or (asym_avoid_loops_gain > 0));


    current_sol = asym_ls.get_tour(first_loc_index);
    current_cost = asymmetric_tsp.cost(current_sol);

    auto end_asym_local_search = std::chrono::high_resolution_clock::now();

    asym_local_search_duration 
      = std::chrono::duration_cast<std::chrono::milliseconds>
      (end_asym_local_search - start_asym_local_search).count();
    BOOST_LOG_TRIVIAL(info) << "[Asym. local search] Done, took "
                            << asym_local_search_duration << " ms.";

    BOOST_LOG_TRIVIAL(info) 
      << "[Asym. local search] Asymmetric solution cost is now "
      << current_cost
      << " (" 
      << std::fixed << std::setprecision(2)
      << 100 *(((double) current_cost) / sym_ls_cost - 1)
      << "%).";
  }

  computing_times.local_search 
    = sym_local_search_duration + asym_local_search_duration;

  // Deal with open tour cases requiring adaptation.
  if(!cl_args.force_start and cl_args.force_end){
    // The tour has been listed starting with the "forced" end. This
    // index has to be popped and put back, the next element being the
    // chosen start resulting from the optimization.
    current_sol.push_back(current_sol.front());
    current_sol.pop_front();
  }

  logger log (cl_args);
  log.write_solution(asymmetric_tsp, current_sol, computing_times);
}
