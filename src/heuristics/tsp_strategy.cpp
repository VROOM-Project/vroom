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

#include "tsp_strategy.h"

void solve_atsp(std::string request, std::string out_file){
  // Building problem object with embedded table request.
  auto start_problem_build = std::chrono::high_resolution_clock::now();

  tsp asymmetric_tsp (request);

  auto end_problem_build = std::chrono::high_resolution_clock::now();

  double problem_build_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_problem_build - start_problem_build).count();

  std::cout << "Built with OSRM: "
            << problem_build_duration / 1000 << "s\n";

  // Using symmetrized problem to apply heuristics and local searches.
  tsp_sym symmetrized_tsp (asymmetric_tsp.get_symmetrized_matrix());

  // Applying Christofides heuristic.
  auto start_christo = std::chrono::high_resolution_clock::now();
  heuristic* christo_h = new christo_heuristic();
  std::list<index_t> christo_sol
    = christo_h->build_solution(symmetrized_tsp);
  delete christo_h;

  auto end_christo = std::chrono::high_resolution_clock::now();

  double christo_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_christo - start_christo).count();

  std::cout << "Christofides heuristic applied: "
            << christo_duration / 1000 << "s\n";

  distance_t christo_cost = symmetrized_tsp.cost(christo_sol);

  // Applying deterministic, fast local search to improve the current
  // solution in a small amount of time. All possible moves for the
  // different neighbourhoods are performed, stopping when reaching a
  // local minima.
  local_search ls (&symmetrized_tsp, christo_sol);
  auto start_local_search = std::chrono::high_resolution_clock::now();
  
  distance_t two_opt_gain = 0;
  distance_t relocate_gain = 0;
  distance_t or_opt_gain = 0;
  
  do{
    // All possible 2-opt moves.
    auto start_two_opt = std::chrono::high_resolution_clock::now();
    two_opt_gain = ls.perform_all_two_opt_steps();

    std::cout << " ("
              << (double) two_opt_gain * 100 / christo_cost
              << "%)"
              << std::endl;
  
    auto end_two_opt = std::chrono::high_resolution_clock::now();

    double two_opt_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_two_opt - start_two_opt).count();

    std::cout << "Two-opt steps applied in: "
              << two_opt_duration / 1000 << "s\n";

    // All relocate moves.
    auto start_relocate = std::chrono::high_resolution_clock::now();
    relocate_gain = ls.perform_all_relocate_steps();

    std::cout << " ("
              << (double) relocate_gain * 100 / christo_cost
              << "%)"
              << std::endl;
    auto end_relocate = std::chrono::high_resolution_clock::now();

    double relocate_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_relocate - start_relocate).count();

    std::cout << "Relocate steps applied in: "
              << relocate_duration / 1000 << "s\n";

    // All or-opt moves.
    auto start_or_opt = std::chrono::high_resolution_clock::now();
    or_opt_gain = ls.perform_all_or_opt_steps();

    std::cout << " ("
              << (double) or_opt_gain * 100 / christo_cost
              << "%)"
              << std::endl;

    auto end_or_opt = std::chrono::high_resolution_clock::now();

    double or_opt_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_or_opt - start_or_opt).count();

    std::cout << "Or-Opt steps applied in: "
              << or_opt_duration / 1000 << "s\n";
  }while((two_opt_gain > 0) or (relocate_gain > 0) or (or_opt_gain > 0));

  std::list<index_t> local_search_sol = ls.get_tour(0);

  auto end_local_search = std::chrono::high_resolution_clock::now();

  double local_search_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_local_search - start_local_search).count();
  std::cout << "Local search: "
            << local_search_duration / 1000 << "s\n";

  // Back to the asymmetric problem, picking the best way.
  std::list<index_t> reverse_sol;
  for(auto step = local_search_sol.begin();
      step != local_search_sol.end();
      ++step){
    reverse_sol.push_front(*step);
  }

  distance_t direct_cost = asymmetric_tsp.cost(local_search_sol);
  distance_t reverse_cost = asymmetric_tsp.cost(reverse_sol);

  double total_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_local_search - start_problem_build).count();
  
  std::cout << "Total: "
            << total_duration / 1000 << "s\n";

  logger log (out_file);
  if(direct_cost < reverse_cost){
    log.tour_to_file(asymmetric_tsp, local_search_sol, total_duration / 1000);
  }
  else{
    log.tour_to_file(asymmetric_tsp, reverse_sol, total_duration / 1000);
  }
}
