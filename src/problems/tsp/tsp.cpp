/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "problems/tsp/heuristics/christofides.h"
#include "problems/tsp/heuristics/local_search.h"
#include "problems/tsp/tsp.h"
#include "utils/helpers.h"

namespace vroom {

UserCost compute_cost(const std::list<Index>& tour,
                      const Matrix<UserCost>& matrix) {
  UserCost cost = 0;
  Index init_step = 0; // Initialization actually never used.

  auto step = tour.cbegin();
  if (!tour.empty()) {
    init_step = *step;
  }

  Index previous_step = init_step;
  ++step;
  for (; step != tour.cend(); ++step) {
    cost += matrix[previous_step][*step];
    previous_step = *step;
  }
  if (!tour.empty()) {
    cost += matrix[previous_step][init_step];
  }
  return cost;
}

TSP::TSP(const Input& input, std::vector<Index>&& job_ranks, Index vehicle_rank)
  : VRP(input),
    _vehicle_rank(vehicle_rank),
    _job_ranks(std::move(job_ranks)),
    _has_start(_input.vehicles[_vehicle_rank].has_start()),
    _has_end(_input.vehicles[_vehicle_rank].has_end()) {

  assert(!_job_ranks.empty());

  // Pick ranks to select from input matrix.
  std::vector<Index> matrix_ranks;
  matrix_ranks.reserve(_job_ranks.size() + 2);

  std::ranges::transform(_job_ranks,
                         std::back_inserter(matrix_ranks),
                         [&](const auto& r) { return _input.jobs[r].index(); });

  if (_has_start) {
    // Add start and remember rank in _matrix.
    _start = matrix_ranks.size();
    matrix_ranks.push_back(
      _input.vehicles[_vehicle_rank].start.value().index());
  }
  if (_has_end) {
    // Add end and remember rank in _matrix.
    if (_has_start && (_input.vehicles[_vehicle_rank].start.value().index() ==
                       _input.vehicles[_vehicle_rank].end.value().index())) {
      // Avoiding duplicate for identical ranks.
      _end = _start;
    } else {
      _end = matrix_ranks.size();
      matrix_ranks.push_back(
        _input.vehicles[_vehicle_rank].end.value().index());
    }
  }

  // Populate TSP-solving matrix.
  _matrix = Matrix<UserCost>(matrix_ranks.size());

  const auto& v = _input.vehicles[vehicle_rank];
  for (Index i = 0; i < matrix_ranks.size(); ++i) {
    for (Index j = 0; j < matrix_ranks.size(); ++j) {
      // The TSP solving cost stays in the UserCost unsigned world so
      // we need to undo the default scaling occurring in
      // Vehicle::cost.
      _matrix[i][j] =
        utils::scale_to_user_cost(v.cost(matrix_ranks[i], matrix_ranks[j]));
    }
  }

  // Distances on the diagonal are never used except in the minimum
  // weight perfect matching (munkres call during the heuristic). This
  // makes sure no node will be matched with itself at that time.
  for (Index i = 0; i < _matrix.size(); ++i) {
    _matrix[i][i] = INFINITE_USER_COST;
  }

  _round_trip = _has_start && _has_end && (_start == _end);

  if (!_round_trip) {
    // Dealing with open tour cases. Exactly one of the following
    // happens.
    if (_has_start && !_has_end) {
      // Forcing first location as start, end location decided during
      // optimization.
      for (Index i = 0; i < _matrix.size(); ++i) {
        if (i != _start) {
          _matrix[i][_start] = 0;
        }
      }
    }
    if (!_has_start && _has_end) {
      // Forcing last location as end, start location decided during
      // optimization.
      for (Index j = 0; j < _matrix.size(); ++j) {
        if (j != _end) {
          _matrix[_end][j] = 0;
        }
      }
    }
    if (_has_start && _has_end) {
      // Forcing first location as start, last location as end to
      // produce an open tour.
      assert(_start != _end);
      _matrix[_end][_start] = 0;
      for (Index j = 0; j < _matrix.size(); ++j) {
        if ((j != _start) && (j != _end)) {
          _matrix[_end][j] = INFINITE_USER_COST;
        }
      }
    }
  }

  // Compute symmetrized matrix and update _is_symmetric flag.
  _symmetrized_matrix = Matrix<UserCost>(_matrix.size());

  const UserCost& (*sym_f)(const UserCost&, const UserCost&) =
    std::min<UserCost>;
  if ((_has_start && !_has_end) || (!_has_start && _has_end)) {
    // Using symmetrization with max as when only start or only end is
    // forced, the matrix has a line or a column filled with zeros.
    sym_f = std::max<UserCost>;
  }
  for (Index i = 0; i < _matrix.size(); ++i) {
    _symmetrized_matrix[i][i] = _matrix[i][i];
    for (Index j = i + 1; j < _matrix.size(); ++j) {
      _is_symmetric = _is_symmetric && (_matrix[i][j] == _matrix[j][i]);
      UserCost val = sym_f(_matrix[i][j], _matrix[j][i]);
      _symmetrized_matrix[i][j] = val;
      _symmetrized_matrix[j][i] = val;
    }
  }
}

UserCost TSP::cost(const std::list<Index>& tour) const {
  return compute_cost(tour, _matrix);
}

UserCost TSP::symmetrized_cost(const std::list<Index>& tour) const {
  return compute_cost(tour, _symmetrized_matrix);
}

std::vector<Index> TSP::raw_solve(unsigned nb_threads,
                                  const Timeout& timeout) const {
  // Compute deadline including heuristic computing time.
  const Deadline deadline =
    timeout.has_value() ? utils::now() + timeout.value() : Deadline();

  // Applying heuristic.
  std::list<Index> christo_sol = tsp::christofides(_symmetrized_matrix);

  Deadline sym_deadline = deadline;
  if (deadline.has_value() && !_is_symmetric) {
    // Rule of thumb if problem is asymmetric: dedicate 70% of the
    // remaining available solving time to the symmetric local search,
    // then the rest to the asymmetric version.
    constexpr double sym_ls_ratio = 0.7;
    const auto after_heuristic = utils::now();
    const auto remaining_ms =
      (after_heuristic < deadline.value())
        ? std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline.value() - after_heuristic)
            .count()
        : 0;
    sym_deadline =
      after_heuristic + std::chrono::milliseconds(
                          static_cast<unsigned>(sym_ls_ratio * remaining_ms));
  }

  // Local search on symmetric problem.
  // Applying deterministic, fast local search to improve the current
  // solution in a small amount of time. All possible moves for the
  // different neighbourhoods are performed, stopping when reaching a
  // local minima.
  tsp::LocalSearch sym_ls(_symmetrized_matrix,
                          std::make_pair(!_round_trip && _has_start && _has_end,
                                         _start),
                          christo_sol,
                          nb_threads);

  UserCost sym_two_opt_gain = 0;
  UserCost sym_relocate_gain = 0;
  UserCost sym_or_opt_gain = 0;

  do {
    // All possible 2-opt moves.
    sym_two_opt_gain = sym_ls.perform_all_two_opt_steps(sym_deadline);

    // All relocate moves.
    sym_relocate_gain = sym_ls.perform_all_relocate_steps(sym_deadline);

    // All or-opt moves.
    sym_or_opt_gain = sym_ls.perform_all_or_opt_steps(sym_deadline);
  } while ((sym_two_opt_gain > 0) || (sym_relocate_gain > 0) ||
           (sym_or_opt_gain > 0));

  Index first_loc_index;
  if (_has_start) {
    // Use start value set in constructor from vehicle input.
    first_loc_index = _start;
  } else {
    assert(_has_end);
    // Requiring the tour to be described from the "forced" end
    // location.
    first_loc_index = _end;
  }

  std::list<Index> current_sol = sym_ls.get_tour(first_loc_index);

  if (!_is_symmetric) {
    // Back to the asymmetric problem, picking the best way.
    std::list<Index> reverse_current_sol(current_sol);
    reverse_current_sol.reverse();
    UserCost direct_cost = this->cost(current_sol);
    UserCost reverse_cost = this->cost(reverse_current_sol);

    // Local search on asymmetric problem.
    tsp::LocalSearch
      asym_ls(_matrix,
              std::make_pair(!_round_trip && _has_start && _has_end, _start),
              (direct_cost <= reverse_cost) ? current_sol : reverse_current_sol,
              nb_threads);

    UserCost asym_two_opt_gain = 0;
    UserCost asym_relocate_gain = 0;
    UserCost asym_or_opt_gain = 0;
    UserCost asym_avoid_loops_gain = 0;

    do {
      // All avoid-loops moves.
      asym_avoid_loops_gain = asym_ls.perform_all_avoid_loop_steps(deadline);

      // All possible 2-opt moves.
      asym_two_opt_gain = asym_ls.perform_all_asym_two_opt_steps(deadline);

      // All relocate moves.
      asym_relocate_gain = asym_ls.perform_all_relocate_steps(deadline);

      // All or-opt moves.
      asym_or_opt_gain = asym_ls.perform_all_or_opt_steps(deadline);
    } while ((asym_two_opt_gain > 0) || (asym_relocate_gain > 0) ||
             (asym_or_opt_gain > 0) || (asym_avoid_loops_gain > 0));

    current_sol = asym_ls.get_tour(first_loc_index);
  }

  // Deal with open tour cases requiring adaptation.
  if (!_has_start && _has_end) {
    // The tour has been listed starting with the "forced" end. This
    // index has to be popped and put back, the next element being the
    // chosen start resulting from the optimization.
    current_sol.push_back(current_sol.front());
    current_sol.pop_front();
  }

  // Handle start and end removal as output list should only contain
  // jobs.
  if (_has_start) {
    // Jobs start further away in the list.
    current_sol.pop_front();
  }
  if (!_round_trip && _has_end) {
    current_sol.pop_back();
  }

  // Back to ranks in input::_jobs.
  std::vector<Index> init_ranks_sol;
  std::ranges::transform(current_sol,
                         std::back_inserter(init_ranks_sol),
                         [&](const auto& i) { return _job_ranks[i]; });

  return init_ranks_sol;
}

Solution TSP::solve(unsigned,
                    unsigned,
                    unsigned nb_threads,
                    const Timeout& timeout,
                    const std::vector<HeuristicParameters>&) const {
  RawRoute r(_input, 0, 0);
  r.set_route(_input, raw_solve(nb_threads, timeout));
  return utils::format_solution(_input, {r});
}

} // namespace vroom
