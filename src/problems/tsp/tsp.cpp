/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "tsp.h"
#include "../../structures/vroom/input/input.h"

tsp::tsp(const input& input,
         std::vector<index_t> problem_indices,
         index_t vehicle_rank)
  : vrp(input),
    _vehicle_rank(vehicle_rank),
    _tsp_index_to_global(std::move(problem_indices)),
    _matrix(_input._matrix.get_sub_matrix(_tsp_index_to_global)),
    _symmetrized_matrix(_matrix.size()),
    _is_symmetric(true),
    _has_start(_input._vehicles[_vehicle_rank].has_start()),
    _has_end(_input._vehicles[_vehicle_rank].has_end()) {

  if (_has_start) {
    // Use index in _matrix for start.
    auto search = std::find(_tsp_index_to_global.begin(),
                            _tsp_index_to_global.end(),
                            _input._vehicles[_vehicle_rank].start.get().index);
    assert(search != _tsp_index_to_global.end());
    _start = std::distance(_tsp_index_to_global.begin(), search);
    assert(_start < _matrix.size());
  }
  if (_has_end) {
    // Use index in _matrix for start.
    auto search = std::find(_tsp_index_to_global.begin(),
                            _tsp_index_to_global.end(),
                            _input._vehicles[_vehicle_rank].end.get().index);
    assert(search != _tsp_index_to_global.end());
    _end = std::distance(_tsp_index_to_global.begin(), search);
    assert(_end < _matrix.size());
  }

  _round_trip = _has_start and _has_end and (_start == _end);

  if (!_round_trip) {
    // Dealing with open tour cases. Exactly one of the following
    // happens.
    if (_has_start and !_has_end) {
      // Forcing first location as start, end location decided during
      // optimization.
      for (index_t i = 0; i < _matrix.size(); ++i) {
        if (i != _start) {
          _matrix[i][_start] = 0;
        }
      }
    }
    if (!_has_start and _has_end) {
      // Forcing last location as end, start location decided during
      // optimization.
      for (index_t j = 0; j < _matrix.size(); ++j) {
        if (j != _end) {
          _matrix[_end][j] = 0;
        }
      }
    }
    if (_has_start and _has_end) {
      // Forcing first location as start, last location as end to
      // produce an open tour.
      assert(_start != _end);
      _matrix[_end][_start] = 0;
      for (index_t j = 0; j < _matrix.size(); ++j) {
        if ((j != _start) and (j != _end)) {
          _matrix[_end][j] = INFINITE_COST;
        }
      }
    }
  }

  // Compute symmetrized matrix and update _is_symmetric flag.
  const cost_t& (*sym_f)(const cost_t&, const cost_t&) = std::min<cost_t>;
  if ((_has_start and !_has_end) or (!_has_start and _has_end)) {
    // Using symmetrization with max as when only start or only end is
    // forced, the matrix has a line or a column filled with zeros.
    sym_f = std::max<cost_t>;
  }
  for (index_t i = 0; i < _matrix.size(); ++i) {
    _symmetrized_matrix[i][i] = _matrix[i][i];
    for (index_t j = i + 1; j < _matrix.size(); ++j) {
      _is_symmetric &= (_matrix[i][j] == _matrix[j][i]);
      cost_t val = sym_f(_matrix[i][j], _matrix[j][i]);
      _symmetrized_matrix[i][j] = val;
      _symmetrized_matrix[j][i] = val;
    }
  }
}

cost_t tsp::cost(const std::list<index_t>& tour) const {
  cost_t cost = 0;
  index_t init_step = 0; // Initialization actually never used.

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

cost_t tsp::symmetrized_cost(const std::list<index_t>& tour) const {
  cost_t cost = 0;
  index_t init_step = 0; // Initialization actually never used.

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
  BOOST_LOG_TRIVIAL(info) << "[Heuristic] Start heuristic on symmetrized "
                             "problem.";

  std::list<index_t> christo_sol = christofides(_symmetrized_matrix);
  cost_t christo_cost = this->symmetrized_cost(christo_sol);

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
  BOOST_LOG_TRIVIAL(info) << "[Local search] Start local search on symmetrized "
                             "problem.";
  BOOST_LOG_TRIVIAL(info) << "[Local search] Using " << nb_threads
                          << " thread(s).";

  local_search sym_ls(_symmetrized_matrix,
                      true, // Symmetrized problem.
                      christo_sol,
                      nb_threads);

  cost_t sym_two_opt_gain = 0;
  cost_t sym_relocate_gain = 0;
  cost_t sym_or_opt_gain = 0;

  do {
    // All possible 2-opt moves.
    sym_two_opt_gain = sym_ls.perform_all_two_opt_steps();

    // All relocate moves.
    sym_relocate_gain = sym_ls.perform_all_relocate_steps();

    // All or-opt moves.
    sym_or_opt_gain = sym_ls.perform_all_or_opt_steps();
  } while ((sym_two_opt_gain > 0) or (sym_relocate_gain > 0) or
           (sym_or_opt_gain > 0));

  index_t first_loc_index;
  if (_has_start) {
    // Use start value set in constructor from vehicle input.
    first_loc_index = _start;
  } else {
    assert(_has_end);
    // Requiring the tour to be described from the "forced" end
    // location.
    first_loc_index = _end;
  }

  std::list<index_t> current_sol = sym_ls.get_tour(first_loc_index);
  auto current_cost = this->symmetrized_cost(current_sol);

  auto end_sym_local_search = std::chrono::high_resolution_clock::now();

  auto sym_local_search_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      end_sym_local_search - start_sym_local_search)
      .count();
  BOOST_LOG_TRIVIAL(info) << "[Local search] Done, took "
                          << sym_local_search_duration << " ms.";

  BOOST_LOG_TRIVIAL(info) << "[Local search] Symmetric solution cost is now "
                          << current_cost << " (" << std::fixed
                          << std::setprecision(2)
                          << 100 * (((double)current_cost) / christo_cost - 1)
                          << "%).";

  auto asym_local_search_duration = 0;

  if (!_is_symmetric) {
    auto start_asym_local_search = std::chrono::high_resolution_clock::now();

    // Back to the asymmetric problem, picking the best way.
    std::list<index_t> reverse_current_sol(current_sol);
    reverse_current_sol.reverse();
    cost_t direct_cost = this->cost(current_sol);
    cost_t reverse_cost = this->cost(reverse_current_sol);

    // Cost reference after symmetric local search.
    cost_t sym_ls_cost = std::min(direct_cost, reverse_cost);

    // Local search on asymmetric problem.
    local_search asym_ls(_matrix,
                         false, // Not the symmetrized problem.
                         (direct_cost <= reverse_cost) ? current_sol
                                                       : reverse_current_sol,
                         nb_threads);

    BOOST_LOG_TRIVIAL(info) << "[Asym. local search] Back to asymmetric "
                               "problem, initial solution cost is "
                            << sym_ls_cost << ".";

    BOOST_LOG_TRIVIAL(info)
      << "[Asym. local search] Start local search on asymmetric problem.";

    BOOST_LOG_TRIVIAL(info) << "[Asym. local search] Using " << nb_threads
                            << " thread(s).";

    cost_t asym_two_opt_gain = 0;
    cost_t asym_relocate_gain = 0;
    cost_t asym_or_opt_gain = 0;
    cost_t asym_avoid_loops_gain = 0;

    do {
      // All avoid-loops moves.
      asym_avoid_loops_gain = asym_ls.perform_all_avoid_loop_steps();

      // All possible 2-opt moves.
      asym_two_opt_gain = asym_ls.perform_all_asym_two_opt_steps();

      // All relocate moves.
      asym_relocate_gain = asym_ls.perform_all_relocate_steps();

      // All or-opt moves.
      asym_or_opt_gain = asym_ls.perform_all_or_opt_steps();
    } while ((asym_two_opt_gain > 0) or (asym_relocate_gain > 0) or
             (asym_or_opt_gain > 0) or (asym_avoid_loops_gain > 0));

    current_sol = asym_ls.get_tour(first_loc_index);
    current_cost = this->cost(current_sol);

    auto end_asym_local_search = std::chrono::high_resolution_clock::now();

    asym_local_search_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        end_asym_local_search - start_asym_local_search)
        .count();
    BOOST_LOG_TRIVIAL(info) << "[Asym. local search] Done, took "
                            << asym_local_search_duration << " ms.";

    BOOST_LOG_TRIVIAL(info)
      << "[Asym. local search] Asymmetric solution cost is now " << current_cost
      << " (" << std::fixed << std::setprecision(2)
      << 100 * (((double)current_cost) / sym_ls_cost - 1) << "%).";
  }

  // Deal with open tour cases requiring adaptation.
  if (!_has_start and _has_end) {
    // The tour has been listed starting with the "forced" end. This
    // index has to be popped and put back, the next element being the
    // chosen start resulting from the optimization.
    current_sol.push_back(current_sol.front());
    current_sol.pop_front();
  }

  // Steps for the one route.
  std::vector<step> steps;

  // Handle start.
  auto job_start = current_sol.cbegin();
  if (_has_start) {
    // Add start step.
    assert(current_sol.front() == _start);
    auto rank = _input.get_location_rank_from_index(
      _tsp_index_to_global[current_sol.front()]);
    steps.emplace_back(TYPE::START, _input._locations[rank]);
    // Remember that jobs start further away in the list.
    ++job_start;
  }
  // Determine end index and where to stop for last job.
  auto job_end = current_sol.cend();
  index_t end_index = current_sol.back();

  if (_round_trip) {
    end_index = _start;
  } else {
    if (_has_end) {
      assert(current_sol.back() == end_index);
      --job_end;
    }
  }

  // Handle jobs.
  for (auto job = job_start; job != job_end; ++job) {
    auto current_rank =
      _input.get_job_rank_from_index(_tsp_index_to_global[*job]);
    steps.emplace_back(TYPE::JOB,
                       _input._jobs[current_rank],
                       _input._jobs[current_rank].id);
  }
  // Handle end.
  if (_has_end) {
    // Add end step.
    auto rank =
      _input.get_location_rank_from_index(_tsp_index_to_global[end_index]);
    steps.emplace_back(TYPE::END, _input._locations[rank]);
  }

  // Route.
  std::vector<route_t> routes;
  routes.emplace_back(_input._vehicles[_vehicle_rank].id, steps, current_cost);

  solution sol(0, std::move(routes), current_cost);

  return sol;
}
