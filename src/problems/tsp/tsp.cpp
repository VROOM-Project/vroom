/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "tsp.h"
#include "../../structures/vroom/input/input.h"

tsp::tsp(const input& input,
         index_t vehicle_rank):
  vrp(input),
  _vehicle_rank(vehicle_rank),
  _matrix(_input._matrix),      // TODO avoid this copy!
  _symmetrized_matrix(_input._matrix.size()),
  _is_symmetric(true),
  _force_start(_input._vehicles[_vehicle_rank].has_start()),
  _force_end(_input._vehicles[_vehicle_rank].has_end()) {
  if(_force_start){
    if( _input._vehicles[_vehicle_rank].start_id != boost::none ){
      _start = _input._vehicles[_vehicle_rank].start_id.get();
    }else{
      _start = _input._vehicles[_vehicle_rank].start.get().index;
    }
    assert(_start < _matrix.size());
  }
  if(_force_end){
    if( _input._vehicles[_vehicle_rank].end_id != boost::none ){
      _end = _input._vehicles[_vehicle_rank].end_id.get();
    }else{
      _end = _input._vehicles[_vehicle_rank].end.get().index;
    }
    assert(_end < _matrix.size());
  }

  // Dealing with open tour cases. At most one of the following
  // occurs.
  if (_force_start and !_force_end) {
    // Forcing first location as start, end location decided during
    // optimization.
    for (index_t i = 0; i < _matrix.size(); ++i) {
      if (i != _start) {
        _matrix[i][_start] = 0;
      }
    }
  }
  if (!_force_start and _force_end) {
    // Forcing last location as end, start location decided during
    // optimization.
    for (index_t j = 0; j < _matrix.size(); ++j) {
      if (j != _end) {
        _matrix[_end][j] = 0;
      }
    }
  }
  if (_force_start and _force_end) {
    // Forcing first location as start, last location as end to
    // produce an open tour.
    assert(_start != _end);
    _matrix[_end][_start] = 0;
    for (index_t j = 0; j < _matrix.size(); ++j) {
      if ((j != _start) and (j != _end)) {
        _matrix[_end][j] = INFINITE_DISTANCE;
      }
    }
  }

  // Compute symmetrized matrix and update _is_symmetric flag.
  const distance_t& (*sym_f) (const distance_t&, const distance_t&) =
    std::min<distance_t>;
  if ((_force_start and !_force_end) or (!_force_start and _force_end)) {
    // Using symmetrization with max as when only start or only end is
    // forced, the matrix has a line or a column filled with zeros.
    sym_f = std::max<distance_t>;
  }
  for (index_t i = 0; i < _matrix.size(); ++i) {
    _symmetrized_matrix[i][i] = _matrix[i][i];
    for (index_t j = i + 1; j < _matrix.size(); ++j) {
      _is_symmetric &= (_matrix[i][j] == _matrix[j][i]);
      distance_t val = sym_f(_matrix[i][j], _matrix[j][i]);
      _symmetrized_matrix[i][j] = val;
      _symmetrized_matrix[j][i] = val;
    }
  }
}

distance_t tsp::cost(const std::list<index_t>& tour) const {
  distance_t cost = 0;
  index_t init_step = 0;        // Initialization actually never used.

  auto step = tour.cbegin();
  if (tour.size() > 0) {
    init_step = *step;
  }

  index_t previous_step = init_step;
  ++step;
  for (; step != tour.cend(); ++step) {
    cost += _matrix[previous_step][*step];
    previous_step = *step;
  }
  if (tour.size() > 0) {
    cost += _matrix[previous_step][init_step];
  }
  return cost;
}

distance_t tsp::symmetrized_cost(const std::list<index_t>& tour) const {
  distance_t cost = 0;
  index_t init_step = 0;        // Initialization actually never used.

  auto step = tour.cbegin();
  if (tour.size() > 0) {
    init_step = *step;
  }

  index_t previous_step = init_step;
  ++step;
  for (; step != tour.cend(); ++step) {
    cost += _symmetrized_matrix[previous_step][*step];
    previous_step = *step;
  }
  if (tour.size() > 0) {
    cost += _symmetrized_matrix[previous_step][init_step];
  }
  return cost;
}

solution tsp::solve(unsigned nb_threads) const {
  // Applying heuristic.
  auto start_heuristic = std::chrono::high_resolution_clock::now();
  BOOST_LOG_TRIVIAL(info)
    << "[Heuristic] Start heuristic on symmetrized problem.";

  std::list<index_t> christo_sol = christofides(_symmetrized_matrix);
  distance_t christo_cost = this->symmetrized_cost(christo_sol);

  auto end_heuristic = std::chrono::high_resolution_clock::now();

  auto heuristic_computing_time =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_heuristic -
                                                          start_heuristic)
    .count();

  BOOST_LOG_TRIVIAL(info) << "[Heuristic] Done, took "
                          << heuristic_computing_time << " ms.";

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
    << "[Local search] Using " << nb_threads << " thread(s).";

  local_search sym_ls(_symmetrized_matrix,
                      true,    // Symmetrized problem.
                      christo_sol,
                      nb_threads);

  distance_t sym_two_opt_gain = 0;
  distance_t sym_relocate_gain = 0;
  distance_t sym_or_opt_gain = 0;

  do {
    // All possible 2-opt moves.
    sym_two_opt_gain = sym_ls.perform_all_two_opt_steps();

    // All relocate moves.
    sym_relocate_gain = sym_ls.perform_all_relocate_steps();

    // All or-opt moves.
    sym_or_opt_gain = sym_ls.perform_all_or_opt_steps();
  } while ((sym_two_opt_gain > 0)
           or (sym_relocate_gain > 0)
           or (sym_or_opt_gain > 0));

  index_t first_loc_index;
  if (_force_start) {
    // Use start value set in constructor from vehicle input.
    first_loc_index = _start;
  }
  else {
    assert(_force_end);
    // Requiring the tour to be described from the "forced" end
    // location.
    first_loc_index = _end;
  }

  std::list<index_t> current_sol = sym_ls.get_tour(first_loc_index);
  auto current_cost = this->symmetrized_cost(current_sol);

  auto end_sym_local_search = std::chrono::high_resolution_clock::now();

  auto sym_local_search_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_sym_local_search - start_sym_local_search)
    .count();
  BOOST_LOG_TRIVIAL(info) << "[Local search] Done, took "
                          << sym_local_search_duration << " ms.";

  BOOST_LOG_TRIVIAL(info) << "[Local search] Symmetric solution cost is now "
                          << current_cost
                          << " ("
                          << std::fixed << std::setprecision(2)
                          << 100 *(((double) current_cost) / christo_cost - 1)
                          << "%).";

  auto asym_local_search_duration = 0;

  if (!_is_symmetric) {
    auto start_asym_local_search = std::chrono::high_resolution_clock::now();

    // Back to the asymmetric problem, picking the best way.
    std::list<index_t> reverse_current_sol(current_sol);
    reverse_current_sol.reverse();
    distance_t direct_cost = this->cost(current_sol);
    distance_t reverse_cost = this->cost(reverse_current_sol);

    // Cost reference after symmetric local search.
    distance_t sym_ls_cost = std::min(direct_cost, reverse_cost);

    // Local search on asymmetric problem.
    local_search asym_ls(_matrix,
                         false, // Not the symmetrized problem.
                         (direct_cost <= reverse_cost) ?
                         current_sol: reverse_current_sol,
                         nb_threads);

    BOOST_LOG_TRIVIAL(info)
      << "[Asym. local search] Back to asymmetric problem, initial solution cost is "
      << sym_ls_cost << ".";

    BOOST_LOG_TRIVIAL(info)
      << "[Asym. local search] Start local search on asymmetric problem.";

    BOOST_LOG_TRIVIAL(info)
      << "[Asym. local search] Using " << nb_threads << " thread(s).";

    distance_t asym_two_opt_gain = 0;
    distance_t asym_relocate_gain = 0;
    distance_t asym_or_opt_gain = 0;
    distance_t asym_avoid_loops_gain = 0;

    do {
      // All avoid-loops moves.
      asym_avoid_loops_gain = asym_ls.perform_all_avoid_loop_steps();

      // All possible 2-opt moves.
      asym_two_opt_gain = asym_ls.perform_all_asym_two_opt_steps();

      // All relocate moves.
      asym_relocate_gain = asym_ls.perform_all_relocate_steps();

      // All or-opt moves.
      asym_or_opt_gain = asym_ls.perform_all_or_opt_steps();
    } while ((asym_two_opt_gain > 0)
             or (asym_relocate_gain > 0)
             or (asym_or_opt_gain > 0)
             or (asym_avoid_loops_gain > 0));


    current_sol = asym_ls.get_tour(first_loc_index);
    current_cost = this->cost(current_sol);

    auto end_asym_local_search = std::chrono::high_resolution_clock::now();

    asym_local_search_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_asym_local_search - start_asym_local_search)
      .count();
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

  // Deal with open tour cases requiring adaptation.
  if (!_force_start and _force_end) {
    // The tour has been listed starting with the "forced" end. This
    // index has to be popped and put back, the next element being the
    // chosen start resulting from the optimization.
    current_sol.push_back(current_sol.front());
    current_sol.pop_front();
  }

  // current_sol;

  // Steps for the one route.
  std::vector<step> steps;

  // Handle start.
  auto job_start = current_sol.cbegin();
  if (_force_start) {
    // Add start step.
    if(_input._json_matrix_provided){
      index_t start_id = _input._vehicles[_vehicle_rank].start_id.get();
      steps.emplace_back(TYPE::START,
                       _input._jobs[start_id],
                       start_id
                      );
    }else{
      steps.emplace_back(TYPE::START,
                       _input.get_location_at(current_sol.front()));
    }
    
    // Remember that jobs start further away in the list.
    ++job_start;
  }
  // Determine where to stop for last job.
  auto job_end = current_sol.cend();
  if (_force_end) {
    --job_end;
  }
  // Handle jobs.
  for (auto job = job_start; job != job_end; ++job) {
    auto current_rank = _input.get_job_rank_from_index(*job);
    steps.emplace_back(TYPE::JOB,
                       _input._jobs[current_rank],
                       _input._jobs[current_rank].id);
  }
  // Handle end.
  if (_force_end) {
    // Add end step.
    if(_input._json_matrix_provided){
      index_t end_id = _input._vehicles[_vehicle_rank].end_id.get();
      steps.emplace_back(TYPE::END,
                       _input._jobs[end_id],
                       end_id
                      );
    }else{
      steps.emplace_back(TYPE::END,
                         _input.get_location_at(current_sol.back()));
    }
  }


  // Route.
  std::vector<route_t> routes;
  routes.emplace_back(_input._vehicles[_vehicle_rank].id,
                      steps,
                      current_cost);

  solution sol(0, std::move(routes), current_cost);

  return sol;
}
