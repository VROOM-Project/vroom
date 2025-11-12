/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>

#include "algorithms/local_search/insertion_search.h"
#include "algorithms/local_search/local_search.h"
#include "problems/vrptw/operators/cross_exchange.h"
#include "problems/vrptw/operators/intra_cross_exchange.h"
#include "problems/vrptw/operators/intra_exchange.h"
#include "problems/vrptw/operators/intra_mixed_exchange.h"
#include "problems/vrptw/operators/intra_or_opt.h"
#include "problems/vrptw/operators/intra_relocate.h"
#include "problems/vrptw/operators/intra_two_opt.h"
#include "problems/vrptw/operators/mixed_exchange.h"
#include "problems/vrptw/operators/or_opt.h"
#include "problems/vrptw/operators/pd_shift.h"
#include "problems/vrptw/operators/priority_replace.h"
#include "problems/vrptw/operators/relocate.h"
#include "problems/vrptw/operators/reverse_two_opt.h"
#include "problems/vrptw/operators/route_exchange.h"
#include "problems/vrptw/operators/route_split.h"
#include "problems/vrptw/operators/swap_star.h"
#include "problems/vrptw/operators/tsp_fix.h"
#include "problems/vrptw/operators/two_opt.h"
#include "problems/vrptw/operators/unassigned_exchange.h"
#include "utils/helpers.h"

namespace vroom::ls {

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
LocalSearch<Route,
            UnassignedExchange,
            CrossExchange,
            MixedExchange,
            TwoOpt,
            ReverseTwoOpt,
            Relocate,
            OrOpt,
            IntraExchange,
            IntraCrossExchange,
            IntraMixedExchange,
            IntraRelocate,
            IntraOrOpt,
            IntraTwoOpt,
            PDShift,
            RouteExchange,
            SwapStar,
            RouteSplit,
            PriorityReplace,
            TSPFix>::LocalSearch(const Input& input,
                                 std::vector<Route>& sol,
                                 unsigned depth,
                                 const Timeout& timeout)
  : _input(input),
    _nb_vehicles(_input.vehicles.size()),
    _depth(depth),
    _deadline(timeout.has_value() ? utils::now() + timeout.value()
                                  : Deadline()),
    _all_routes(_nb_vehicles),
    _sol_state(input),
    _sol(sol),
    _best_sol(sol),
    _best_sol_indicators(_input, _sol) {
  // Initialize all route indices.
  std::iota(_all_routes.begin(), _all_routes.end(), 0);

  // Setup solution state.
  _sol_state.setup(_sol);
}

template <class Route>
RouteInsertion compute_best_insertion(const Input& input,
                                      const utils::SolutionState& sol_state,
                                      const Index j,
                                      Index v,
                                      const Route& route) {
  const auto& current_job = input.jobs[j];
  assert(current_job.type == JOB_TYPE::PICKUP ||
         current_job.type == JOB_TYPE::SINGLE);

  if (current_job.type == JOB_TYPE::SINGLE) {
    return compute_best_insertion_single(input, sol_state, j, v, route);
  }
  auto insert =
    compute_best_insertion_pd(input, sol_state, j, v, route, NO_EVAL);
  if (insert.eval != NO_EVAL) {
    // Normalize cost per job for consistency with single jobs.
    insert.eval.cost =
      static_cast<Cost>(static_cast<double>(insert.eval.cost) / 2);
  }
  return insert;
}

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
std::unordered_set<Index>
LocalSearch<Route,
            UnassignedExchange,
            CrossExchange,
            MixedExchange,
            TwoOpt,
            ReverseTwoOpt,
            Relocate,
            OrOpt,
            IntraExchange,
            IntraCrossExchange,
            IntraMixedExchange,
            IntraRelocate,
            IntraOrOpt,
            IntraTwoOpt,
            PDShift,
            RouteExchange,
            SwapStar,
            RouteSplit,
            PriorityReplace,
            TSPFix>::try_job_additions(const std::vector<Index>& routes,
                                       double regret_coeff) {
  bool job_added;

  std::vector<std::vector<RouteInsertion>> route_job_insertions;
  route_job_insertions.reserve(routes.size());

  for (std::size_t i = 0; i < routes.size(); ++i) {
    route_job_insertions.emplace_back(_input.jobs.size(),
                                      RouteInsertion(_input.get_amount_size()));

    const auto v = routes[i];
    const auto fixed_cost =
      _sol[v].empty() ? _input.vehicles[v].fixed_cost() : 0;

    for (const auto j : _sol_state.unassigned) {
      if (const auto& current_job = _input.jobs[j];
          current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }
      route_job_insertions[i][j] =
        compute_best_insertion(_input, _sol_state, j, v, _sol[v]);

      if (route_job_insertions[i][j].eval != NO_EVAL) {
        route_job_insertions[i][j].eval.cost += fixed_cost;
      }
    }
  }

  std::unordered_set<Index> modified_vehicles;

  do {
    Priority best_priority = 0;
    RouteInsertion best_insertion(_input.get_amount_size());
    double best_cost = std::numeric_limits<double>::max();
    Index best_job_rank = 0;
    Index best_route = 0;
    std::size_t best_route_idx = 0;

    for (const auto j : _sol_state.unassigned) {
      const auto& current_job = _input.jobs[j];
      if (current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }

      const auto job_priority = current_job.priority;

      if (job_priority < best_priority) {
        // Insert higher priority jobs first.
        continue;
      }

      auto smallest = _input.get_cost_upper_bound();
      auto second_smallest = _input.get_cost_upper_bound();
      std::size_t smallest_idx = std::numeric_limits<std::size_t>::max();

      for (std::size_t i = 0; i < routes.size(); ++i) {
        if (route_job_insertions[i][j].eval.cost < smallest) {
          smallest_idx = i;
          second_smallest = smallest;
          smallest = route_job_insertions[i][j].eval.cost;
        } else if (route_job_insertions[i][j].eval.cost < second_smallest) {
          second_smallest = route_job_insertions[i][j].eval.cost;
        }
      }

      // Find best route for current job based on cost of addition and
      // regret cost of not adding.
      for (std::size_t i = 0; i < routes.size(); ++i) {
        if (route_job_insertions[i][j].eval == NO_EVAL) {
          continue;
        }

        const auto& current_r = _sol[routes[i]];
        const auto& vehicle = _input.vehicles[routes[i]];

        if (const bool is_pickup = (_input.jobs[j].type == JOB_TYPE::PICKUP);
            current_r.size() + (is_pickup ? 2 : 1) > vehicle.max_tasks) {
          continue;
        }

        const auto regret_cost =
          (i == smallest_idx) ? second_smallest : smallest;

        const double current_cost =
          static_cast<double>(route_job_insertions[i][j].eval.cost) -
          regret_coeff * static_cast<double>(regret_cost);

        if ((job_priority > best_priority) ||
            (job_priority == best_priority && current_cost < best_cost)) {
          best_priority = job_priority;
          best_job_rank = j;
          best_route = routes[i];
          best_insertion = route_job_insertions[i][j];
          best_cost = current_cost;
          best_route_idx = i;
        }
      }
    }

    job_added = (best_cost < std::numeric_limits<double>::max());

    if (job_added) {
      _sol_state.unassigned.erase(best_job_rank);

      if (const auto& best_job = _input.jobs[best_job_rank];
          best_job.type == JOB_TYPE::SINGLE) {
        _sol[best_route].add(_input, best_job_rank, best_insertion.single_rank);
      } else {
        assert(best_job.type == JOB_TYPE::PICKUP);

        std::vector<Index> modified_with_pd;
        modified_with_pd.reserve(best_insertion.delivery_rank -
                                 best_insertion.pickup_rank + 2);
        modified_with_pd.push_back(best_job_rank);

        std::copy(_sol[best_route].route.begin() + best_insertion.pickup_rank,
                  _sol[best_route].route.begin() + best_insertion.delivery_rank,
                  std::back_inserter(modified_with_pd));
        modified_with_pd.push_back(best_job_rank + 1);

        _sol[best_route].replace(_input,
                                 best_insertion.delivery,
                                 modified_with_pd.begin(),
                                 modified_with_pd.end(),
                                 best_insertion.pickup_rank,
                                 best_insertion.delivery_rank);

        assert(_sol_state.unassigned.find(best_job_rank + 1) !=
               _sol_state.unassigned.end());
        _sol_state.unassigned.erase(best_job_rank + 1);
      }

      // Update best_route data required for consistency.
      modified_vehicles.insert(best_route);
      _sol_state.update_route_eval(_sol[best_route]);
      _sol_state.set_insertion_ranks(_sol[best_route]);

      const auto fixed_cost =
        _sol[best_route].empty() ? _input.vehicles[best_route].fixed_cost() : 0;

      for (const auto j : _sol_state.unassigned) {
        if (const auto& current_job = _input.jobs[j];
            current_job.type == JOB_TYPE::DELIVERY) {
          continue;
        }
        route_job_insertions[best_route_idx][j] =
          compute_best_insertion(_input,
                                 _sol_state,
                                 j,
                                 best_route,
                                 _sol[best_route]);

        if (route_job_insertions[best_route_idx][j].eval != NO_EVAL) {
          route_job_insertions[best_route_idx][j].eval.cost += fixed_cost;
        }
      }
    }
  } while (job_added);

  // Update stored data for consistency (except update_route_eval and
  // set_insertion_ranks done along the way).
  for (const auto v : modified_vehicles) {
    _sol_state.update_route_bbox(_sol[v]);
    _sol_state.update_costs(_sol[v]);
    _sol_state.update_skills(_sol[v]);
    _sol_state.update_priorities(_sol[v]);
    _sol_state.set_node_gains(_sol[v]);
    _sol_state.set_edge_gains(_sol[v]);
    _sol_state.set_pd_matching_ranks(_sol[v]);
    _sol_state.set_pd_gains(_sol[v]);
  }

  return modified_vehicles;
}

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
void LocalSearch<Route,
                 UnassignedExchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 IntraTwoOpt,
                 PDShift,
                 RouteExchange,
                 SwapStar,
                 RouteSplit,
                 PriorityReplace,
                 TSPFix>::run_ls_step() {
  // Store best move involving a pair of routes.
  std::vector<std::vector<std::unique_ptr<Operator>>> best_ops(_nb_vehicles);
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    best_ops[v] = std::vector<std::unique_ptr<Operator>>(_nb_vehicles);
  }

  // List of source/target pairs we need to test (all related vehicles
  // at first).
  std::vector<std::pair<Index, Index>> s_t_pairs;
  s_t_pairs.reserve(_nb_vehicles * _nb_vehicles);

  for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
    for (unsigned t_v = 0; t_v < _nb_vehicles; ++t_v) {
      if (_input.vehicle_ok_with_vehicle(s_v, t_v)) {
        s_t_pairs.emplace_back(s_v, t_v);
      }
    }
  }

  // Store best gain for matching move.
  std::vector<std::vector<Eval>> best_gains(_nb_vehicles,
                                            std::vector<Eval>(_nb_vehicles,
                                                              Eval()));

  // Store best priority increase and number of assigned tasks for use
  // with operators involving a single route and unassigned jobs
  // (UnassignedExchange and PriorityReplace).
  std::vector<Priority> best_priorities(_nb_vehicles, 0);
  std::vector<unsigned> best_removals(_nb_vehicles,
                                      std::numeric_limits<unsigned>::max());

  // Dummy init to enter first loop.
  Eval best_gain(static_cast<Cost>(1));
  Priority best_priority = 0;
  auto best_removal = std::numeric_limits<unsigned>::max();

  while (best_gain.cost > 0 || best_priority > 0) {
    if (_deadline.has_value() && _deadline.value() < utils::now()) {
      break;
    }

    if (_input.has_jobs()) {
      // Move(s) that don't make sense for shipment-only instances.

      // UnassignedExchange stuff
      for (const Index u : _sol_state.unassigned) {
        if (_input.jobs[u].type != JOB_TYPE::SINGLE) {
          continue;
        }

        const Priority u_priority = _input.jobs[u].priority;
        const auto& u_pickup = _input.jobs[u].pickup;
        const auto& u_delivery = _input.jobs[u].delivery;

        for (const auto& [source, target] : s_t_pairs) {
          if (source != target || !_input.vehicle_ok_with_job(source, u) ||
              _sol[source].empty()) {
            continue;
          }

          const auto& delivery_margin = _sol[source].delivery_margin();
          const auto& pickup_margin = _sol[source].pickup_margin();

          const auto begin_t_rank_candidate =
            _sol_state.insertion_ranks_begin[source][u];
          const auto begin_t_rank_weak_candidate =
            _sol_state.weak_insertion_ranks_begin[source][u];
          const auto end_t_rank_candidate =
            _sol_state.insertion_ranks_end[source][u];
          const auto end_t_rank_weak_candidate =
            _sol_state.weak_insertion_ranks_end[source][u];

          for (unsigned s_rank = 0; s_rank < _sol[source].size(); ++s_rank) {
            const auto& current_job = _input.jobs[_sol[source].route[s_rank]];
            if (current_job.type != JOB_TYPE::SINGLE ||
                u_priority < current_job.priority) {
              continue;
            }

            const Priority priority_gain = u_priority - current_job.priority;

            if (best_priorities[source] <= priority_gain) {
              if (!(u_delivery <= delivery_margin + current_job.delivery) ||
                  !(u_pickup <= pickup_margin + current_job.pickup)) {
                continue;
              }

              auto begin_t_rank = 0;
              if (s_rank + 1 != begin_t_rank_weak_candidate) {
                // Weak constraint is only invalidated when removing
                // job right before.
                begin_t_rank = begin_t_rank_weak_candidate;
              }

              if (s_rank + 1 < begin_t_rank_candidate) {
                // Strong constraint still holds when removing job at
                // s_rank.
                begin_t_rank = begin_t_rank_candidate;
              }

              Index end_t_rank = _sol[source].size();
              if (s_rank + 1 != end_t_rank_weak_candidate) {
                // Weak constraint is only invalidated when removing
                // job right before.
                end_t_rank = std::min(end_t_rank, end_t_rank_weak_candidate);
              }

              if (end_t_rank_candidate <= s_rank) {
                // Strong constraint still holds when removing job at
                // s_rank.
                end_t_rank = std::min(end_t_rank, end_t_rank_candidate);
              }

              for (unsigned t_rank = begin_t_rank; t_rank <= end_t_rank;
                   ++t_rank) {
                if (t_rank == s_rank + 1) {
                  // Same move as with t_rank == s_rank.
                  continue;
                }

                UnassignedExchange r(_input,
                                     _sol_state,
                                     _sol_state.unassigned,
                                     _sol[source],
                                     source,
                                     s_rank,
                                     t_rank,
                                     u);

                const bool better_if_valid =
                  (best_priorities[source] < priority_gain) ||
                  (best_priorities[source] == priority_gain &&
                   best_gains[source][source] < r.gain());

                if (better_if_valid && r.is_valid()) {
                  best_priorities[source] = priority_gain;
                  best_removals[source] = 0;
                  // This may potentially define a negative value as
                  // best gain in case priority_gain is non-zero.
                  best_gains[source][source] = r.gain();
                  best_ops[source][source] =
                    std::make_unique<UnassignedExchange>(r);
                }
              }
            }
          }
        }
      }

      // PriorityReplace stuff
      for (const Index u : _sol_state.unassigned) {
        if (_input.jobs[u].type != JOB_TYPE::SINGLE) {
          continue;
        }

        Priority u_priority = _input.jobs[u].priority;

        for (const auto& [source, target] : s_t_pairs) {
          if (source != target || !_input.vehicle_ok_with_job(source, u) ||
              _sol[source].empty() ||
              // We only search for net priority gains here.
              (u_priority <= _sol_state.fwd_priority[source].front() &&
               u_priority <= _sol_state.bwd_priority[source].back())) {
            continue;
          }

          // Find where to stop when replacing beginning of route in
          // order to generate a net priority gain.
          const auto fwd_over =
            std::ranges::find_if(_sol_state.fwd_priority[source],
                                 [u_priority](const auto p) {
                                   return u_priority <= p;
                                 });
          const Index fwd_over_rank =
            std::distance(_sol_state.fwd_priority[source].begin(), fwd_over);
          // A fwd_last_rank of zero will discard replacing the start
          // of the route.
          Index fwd_last_rank = (fwd_over_rank > 0) ? fwd_over_rank - 1 : 0;
          // Go back to find the biggest route beginning portion where
          // splitting is possible.
          while (fwd_last_rank > 0 &&
                 _sol[source].has_pending_delivery_after_rank(fwd_last_rank)) {
            --fwd_last_rank;
          }
          const Priority begin_priority_gain =
            u_priority - _sol_state.fwd_priority[source][fwd_last_rank];

          // Find where to stop when replacing end of route in order
          // to generate a net priority gain.
          const auto bwd_over =
            std::find_if(_sol_state.bwd_priority[source].crbegin(),
                         _sol_state.bwd_priority[source].crend(),
                         [u_priority](const auto p) {
                           return u_priority <= p;
                         });
          const Index bwd_over_rank =
            std::distance(_sol_state.bwd_priority[source].crbegin(), bwd_over);
          Index bwd_first_rank = _sol[source].size() - bwd_over_rank;
          if (bwd_first_rank == 0) {
            // Sum of priorities for whole route is lower than job
            // priority. We elude this case as it is covered by start
            // replacing (also whole route).
            assert(fwd_last_rank == _sol[source].size() - 1);
            ++bwd_first_rank;
          }
          while (
            bwd_first_rank < _sol[source].size() - 1 &&
            _sol[source].has_pending_delivery_after_rank(bwd_first_rank - 1)) {
            ++bwd_first_rank;
          }
          const Priority end_priority_gain =
            u_priority - _sol_state.bwd_priority[source][bwd_first_rank];

          assert(fwd_over_rank > 0 || bwd_over_rank > 0);

          const auto best_current_priority =
            std::max(begin_priority_gain, end_priority_gain);

          if (best_current_priority > 0 &&
              best_priorities[source] <= best_current_priority) {
            PriorityReplace r(_input,
                              _sol_state,
                              _sol_state.unassigned,
                              _sol[source],
                              source,
                              fwd_last_rank,
                              bwd_first_rank,
                              u,
                              best_priorities[source]);

            if (r.is_valid()) {
              const auto priority_gain = r.priority_gain();
              const unsigned removal = _sol[source].size() - r.assigned();
              const auto gain = r.gain();
              if (std::tie(best_priorities[source],
                           removal,
                           best_gains[source][source]) <
                  std::tie(priority_gain, best_removals[source], gain)) {
                best_priorities[source] = priority_gain;
                best_removals[source] = removal;
                // This may potentially define a negative value as best
                // gain.
                best_gains[source][source] = r.gain();
                best_ops[source][source] = std::make_unique<PriorityReplace>(r);
              }
            }
          }
        }
      }
    }

    // CrossExchange stuff
    for (const auto& [source, target] : s_t_pairs) {
      if (target <= source || // This operator is symmetric.
          best_priorities[source] > 0 || best_priorities[target] > 0 ||
          _sol[source].size() < 2 || _sol[target].size() < 2 ||
          (_input.all_locations_have_coords() &&
           _input.vehicles[source].has_same_profile(_input.vehicles[target]) &&
           !_sol_state.route_bbox[source].intersects(
             _sol_state.route_bbox[target]))) {
        continue;
      }

      const auto& s_delivery_margin = _sol[source].delivery_margin();
      const auto& s_pickup_margin = _sol[source].pickup_margin();
      const auto& t_delivery_margin = _sol[target].delivery_margin();
      const auto& t_pickup_margin = _sol[target].pickup_margin();

      for (unsigned s_rank = 0; s_rank < _sol[source].size() - 1; ++s_rank) {
        const auto s_job_rank = _sol[source].route[s_rank];
        const auto s_next_job_rank = _sol[source].route[s_rank + 1];

        if (!_input.vehicle_ok_with_job(target, s_job_rank) ||
            !_input.vehicle_ok_with_job(target, s_next_job_rank)) {
          continue;
        }

        const auto& job_s_type = _input.jobs[s_job_rank].type;

        const bool both_s_single =
          (job_s_type == JOB_TYPE::SINGLE) &&
          (_input.jobs[s_next_job_rank].type == JOB_TYPE::SINGLE);

        const bool is_s_pickup =
          (job_s_type == JOB_TYPE::PICKUP) &&
          (_sol_state.matching_delivery_rank[source][s_rank] == s_rank + 1);

        if (!both_s_single && !is_s_pickup) {
          continue;
        }

        const auto s_delivery = _input.jobs[s_job_rank].delivery +
                                _input.jobs[s_next_job_rank].delivery;
        const auto s_pickup =
          _input.jobs[s_job_rank].pickup + _input.jobs[s_next_job_rank].pickup;

        Index end_t_rank = _sol[target].size() - 1;
        const auto end_s = _sol_state.insertion_ranks_end[target][s_job_rank];
        const auto end_s_next =
          _sol_state.insertion_ranks_end[target][s_next_job_rank];
        end_t_rank = std::min(end_t_rank, end_s);
        end_t_rank = std::min(end_t_rank, end_s_next);

        Index begin_t_rank = 0;
        const auto begin_s =
          _sol_state.insertion_ranks_begin[target][s_job_rank];
        const auto begin_s_next =
          _sol_state.insertion_ranks_begin[target][s_next_job_rank];
        begin_t_rank = std::max(begin_t_rank, begin_s);
        begin_t_rank = std::max(begin_t_rank, begin_s_next);
        begin_t_rank = (begin_t_rank > 1) ? begin_t_rank - 2 : 0;

        for (unsigned t_rank = begin_t_rank; t_rank < end_t_rank; ++t_rank) {
          const auto t_job_rank = _sol[target].route[t_rank];
          const auto t_next_job_rank = _sol[target].route[t_rank + 1];

          if (!_input.vehicle_ok_with_job(source, t_job_rank) ||
              !_input.vehicle_ok_with_job(source, t_next_job_rank)) {
            continue;
          }

          const auto& job_t_type = _input.jobs[t_job_rank].type;

          const bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) &&
            (_input.jobs[t_next_job_rank].type == JOB_TYPE::SINGLE);

          const bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) &&
            (_sol_state.matching_delivery_rank[target][t_rank] == t_rank + 1);

          if (!both_t_single && !is_t_pickup) {
            continue;
          }

          if (s_rank >=
              std::min(_sol_state.insertion_ranks_end[source][t_job_rank],
                       _sol_state
                         .insertion_ranks_end[source][t_next_job_rank])) {
            continue;
          }

          Index begin_s_rank = 0;
          const auto begin_t =
            _sol_state.insertion_ranks_begin[source][t_job_rank];
          const auto begin_t_next =
            _sol_state.insertion_ranks_begin[source][t_next_job_rank];
          begin_s_rank = std::max(begin_s_rank, begin_t);
          begin_s_rank = std::max(begin_s_rank, begin_t_next);
          begin_s_rank = (begin_s_rank > 1) ? begin_s_rank - 2 : 0;
          if (s_rank < begin_s_rank) {
            continue;
          }

          const auto t_delivery = _input.jobs[t_job_rank].delivery +
                                  _input.jobs[t_next_job_rank].delivery;

          if (const auto t_pickup = _input.jobs[t_job_rank].pickup +
                                    _input.jobs[t_next_job_rank].pickup;
              !(t_delivery <= s_delivery_margin + s_delivery) ||
              !(t_pickup <= s_pickup_margin + s_pickup) ||
              !(s_delivery <= t_delivery_margin + t_delivery) ||
              !(s_pickup <= t_pickup_margin + t_pickup)) {
            continue;
          }

          CrossExchange r(_input,
                          _sol_state,
                          _sol[source],
                          source,
                          s_rank,
                          _sol[target],
                          target,
                          t_rank,
                          !is_s_pickup,
                          !is_t_pickup);

          auto& current_best = best_gains[source][target];
          if (current_best < r.gain_upper_bound() && r.is_valid() &&
              current_best < r.gain()) {
            current_best = r.gain();
            best_ops[source][target] = std::make_unique<CrossExchange>(r);
          }
        }
      }
    }

    if (_input.has_jobs()) {
      // MixedExchange stuff
      for (const auto& [source, target] : s_t_pairs) {
        if (source == target || best_priorities[source] > 0 ||
            best_priorities[target] > 0 || _sol[source].size() == 0 ||
            _sol[target].size() < 2 ||
            (_input.all_locations_have_coords() &&
             _input.vehicles[source].has_same_profile(
               _input.vehicles[target]) &&
             !_sol_state.route_bbox[source].intersects(
               _sol_state.route_bbox[target]))) {
          continue;
        }

        if (_sol[source].size() + 1 > _input.vehicles[source].max_tasks) {
          continue;
        }

        const auto& s_delivery_margin = _sol[source].delivery_margin();
        const auto& s_pickup_margin = _sol[source].pickup_margin();
        const auto& t_delivery_margin = _sol[target].delivery_margin();
        const auto& t_pickup_margin = _sol[target].pickup_margin();

        for (unsigned s_rank = 0; s_rank < _sol[source].size(); ++s_rank) {
          const auto s_job_rank = _sol[source].route[s_rank];
          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE ||
              !_input.vehicle_ok_with_job(target, s_job_rank)) {
            // Don't try moving part of a shipment or an incompatible
            // job.
            continue;
          }

          const auto& s_delivery = _input.jobs[s_job_rank].delivery;
          const auto& s_pickup = _input.jobs[s_job_rank].pickup;

          auto end_t_rank =
            std::min(static_cast<Index>(_sol[target].size() - 1),
                     _sol_state.insertion_ranks_end[target][s_job_rank]);

          auto begin_t_rank =
            _sol_state.insertion_ranks_begin[target][s_job_rank];
          begin_t_rank = (begin_t_rank > 1) ? begin_t_rank - 2 : 0;

          for (unsigned t_rank = begin_t_rank; t_rank < end_t_rank; ++t_rank) {
            if (!_input.vehicle_ok_with_job(source,
                                            _sol[target].route[t_rank]) ||
                !_input.vehicle_ok_with_job(source,
                                            _sol[target].route[t_rank + 1])) {
              continue;
            }

            const auto t_job_rank = _sol[target].route[t_rank];
            const auto t_next_job_rank = _sol[target].route[t_rank + 1];
            const auto& job_t_type = _input.jobs[t_job_rank].type;

            const bool both_t_single =
              (job_t_type == JOB_TYPE::SINGLE) &&
              (_input.jobs[t_next_job_rank].type == JOB_TYPE::SINGLE);

            const bool is_t_pickup =
              (job_t_type == JOB_TYPE::PICKUP) &&
              (_sol_state.matching_delivery_rank[target][t_rank] == t_rank + 1);

            if (!both_t_single && !is_t_pickup) {
              continue;
            }

            if (s_rank >=
                std::min(_sol_state.insertion_ranks_end[source][t_job_rank],
                         _sol_state
                           .insertion_ranks_end[source][t_next_job_rank])) {
              continue;
            }

            if (const auto source_begin =
                  std::min(_sol_state.insertion_ranks_begin[source][t_job_rank],
                           _sol_state
                             .insertion_ranks_begin[source][t_next_job_rank]);
                source_begin > s_rank + 1) {
              continue;
            }

            const auto t_delivery = _input.jobs[t_job_rank].delivery +
                                    _input.jobs[t_next_job_rank].delivery;
            if (const auto t_pickup = _input.jobs[t_job_rank].pickup +
                                      _input.jobs[t_next_job_rank].pickup;
                !(t_delivery <= s_delivery_margin + s_delivery) ||
                !(t_pickup <= s_pickup_margin + s_pickup) ||
                !(s_delivery <= t_delivery_margin + t_delivery) ||
                !(s_pickup <= t_pickup_margin + t_pickup)) {
              continue;
            }

            MixedExchange r(_input,
                            _sol_state,
                            _sol[source],
                            source,
                            s_rank,
                            _sol[target],
                            target,
                            t_rank,
                            !is_t_pickup);

            auto& current_best = best_gains[source][target];
            if (current_best < r.gain_upper_bound() && r.is_valid() &&
                current_best < r.gain()) {
              current_best = r.gain();
              best_ops[source][target] = std::make_unique<MixedExchange>(r);
            }
          }
        }
      }
    }

    // TwoOpt stuff
    for (const auto& [source, target] : s_t_pairs) {
      if (target <= source || // This operator is symmetric.
          best_priorities[source] > 0 || best_priorities[target] > 0 ||
          (_input.all_locations_have_coords() &&
           _input.vehicles[source].has_same_profile(_input.vehicles[target]) &&
           !_sol_state.route_bbox[source].intersects(
             _sol_state.route_bbox[target]))) {
        continue;
      }

      const auto& s_v = _input.vehicles[source];
      const auto& t_v = _input.vehicles[target];

      // Determine first ranks for inner loops based on vehicles/jobs
      // compatibility along the routes.
      unsigned first_s_rank = 0;
      if (const auto first_s_candidate =
            _sol_state.bwd_skill_rank[source][target];
          first_s_candidate > 0) {
        first_s_rank = first_s_candidate - 1;
      }

      int first_t_rank = 0;
      if (const auto first_t_candidate =
            _sol_state.bwd_skill_rank[target][source];
          first_t_candidate > 0) {
        first_t_rank = first_t_candidate - 1;
      }

      for (unsigned s_rank = first_s_rank; s_rank < _sol[source].size();
           ++s_rank) {
        if (_sol[source].has_pending_delivery_after_rank(s_rank)) {
          continue;
        }

        const unsigned jobs_moved_from_source =
          _sol[source].size() - s_rank - 1;

        const auto& s_fwd_delivery = _sol[source].fwd_deliveries(s_rank);
        const auto& s_fwd_pickup = _sol[source].fwd_pickups(s_rank);
        const auto& s_bwd_delivery = _sol[source].bwd_deliveries(s_rank);
        const auto& s_bwd_pickup = _sol[source].bwd_pickups(s_rank);

        Index end_t_rank = _sol[target].size();
        if (jobs_moved_from_source > 0) {
          // There is a route end after s_rank in source route.
          const auto s_next_job_rank = _sol[source].route[s_rank + 1];
          end_t_rank =
            std::min(end_t_rank,
                     _sol_state
                       .weak_insertion_ranks_end[target][s_next_job_rank]);
        }

        for (int t_rank = end_t_rank - 1; t_rank >= first_t_rank; --t_rank) {
          if (_sol[target].has_pending_delivery_after_rank(t_rank)) {
            continue;
          }

          assert(static_cast<int>(_sol[target].size()) - t_rank - 1 >= 0);
          if (const unsigned jobs_moved_from_target =
                _sol[target].size() - static_cast<unsigned>(t_rank) - 1;
              jobs_moved_from_source <= 2 && jobs_moved_from_target <= 2) {
            // One of Relocate, OrOpt, SwapStar, MixedExchange, or no-opt.
            continue;
          }

          if (t_rank + 1 < static_cast<int>(_sol[target].size())) {
            // There is a route end after t_rank in target route.
            const auto t_next_job_rank = _sol[target].route[t_rank + 1];
            if (_sol_state.weak_insertion_ranks_end[source][t_next_job_rank] <=
                s_rank) {
              // Job right after t_rank won't fit after job at s_rank
              // in source route.
              continue;
            }
          }

          if (s_rank + _sol[target].size() - t_rank > s_v.max_tasks ||
              t_rank + _sol[source].size() - s_rank > t_v.max_tasks) {
            continue;
          }

          const auto& t_bwd_delivery = _sol[target].bwd_deliveries(t_rank);

          if (const auto& t_bwd_pickup = _sol[target].bwd_pickups(t_rank);
              !(s_fwd_delivery + t_bwd_delivery <= s_v.capacity) ||
              !(s_fwd_pickup + t_bwd_pickup <= s_v.capacity)) {
            // Stop current loop since we're going backward with
            // t_rank.
            break;
          }

          const auto& t_fwd_delivery = _sol[target].fwd_deliveries(t_rank);

          if (const auto& t_fwd_pickup = _sol[target].fwd_pickups(t_rank);
              !(t_fwd_delivery + s_bwd_delivery <= t_v.capacity) ||
              !(t_fwd_pickup + s_bwd_pickup <= t_v.capacity)) {
            continue;
          }

          TwoOpt r(_input,
                   _sol_state,
                   _sol[source],
                   source,
                   s_rank,
                   _sol[target],
                   target,
                   t_rank);

          if (best_gains[source][target] < r.gain() && r.is_valid()) {
            best_gains[source][target] = r.gain();
            best_ops[source][target] = std::make_unique<TwoOpt>(r);
          }
        }
      }
    }

    // ReverseTwoOpt stuff
    for (const auto& [source, target] : s_t_pairs) {
      if (source == target || best_priorities[source] > 0 ||
          best_priorities[target] > 0 ||
          (_input.all_locations_have_coords() &&
           _input.vehicles[source].has_same_profile(_input.vehicles[target]) &&
           !_sol_state.route_bbox[source].intersects(
             _sol_state.route_bbox[target]))) {
        continue;
      }

      const auto& s_v = _input.vehicles[source];
      const auto& t_v = _input.vehicles[target];

      // Determine first rank for inner loop based on vehicles/jobs
      // compatibility along the routes.
      unsigned first_s_rank = 0;
      if (const auto first_s_candidate =
            _sol_state.bwd_skill_rank[source][target];
          first_s_candidate > 0) {
        first_s_rank = first_s_candidate - 1;
      }

      for (unsigned s_rank = first_s_rank; s_rank < _sol[source].size();
           ++s_rank) {
        if (_sol[source].has_delivery_after_rank(s_rank)) {
          continue;
        }

        const auto& s_fwd_delivery = _sol[source].fwd_deliveries(s_rank);
        const auto& s_fwd_pickup = _sol[source].fwd_pickups(s_rank);
        const auto& s_bwd_delivery = _sol[source].bwd_deliveries(s_rank);
        const auto& s_bwd_pickup = _sol[source].bwd_pickups(s_rank);

        Index begin_t_rank = 0;
        if (s_rank + 1 < _sol[source].size()) {
          // There is a route end after s_rank in source route.
          const auto s_next_job_rank = _sol[source].route[s_rank + 1];
          const auto unmodified_begin =
            _sol_state.weak_insertion_ranks_begin[target][s_next_job_rank];
          if (unmodified_begin > 0) {
            begin_t_rank = unmodified_begin - 1;
          }
        }

        for (unsigned t_rank = begin_t_rank;
             t_rank < _sol_state.fwd_skill_rank[target][source];
             ++t_rank) {
          if (_sol[target].has_pickup_up_to_rank(t_rank)) {
            continue;
          }

          if (const auto t_job_rank = _sol[target].route[t_rank];
              _sol_state.weak_insertion_ranks_end[source][t_job_rank] <=
              s_rank) {
            // Job at t_rank won't fit after job at s_rank in source
            // route.
            continue;
          }

          if (s_rank + t_rank + 2 > s_v.max_tasks ||
              (_sol[source].size() - s_rank - 1) +
                  (_sol[target].size() - t_rank - 1) >
                t_v.max_tasks) {
            continue;
          }

          const auto& t_fwd_delivery = _sol[target].fwd_deliveries(t_rank);

          if (const auto& t_fwd_pickup = _sol[target].fwd_pickups(t_rank);
              !(s_fwd_delivery + t_fwd_delivery <= s_v.capacity) ||
              !(s_fwd_pickup + t_fwd_pickup <= s_v.capacity)) {
            break;
          }

          const auto& t_bwd_delivery = _sol[target].bwd_deliveries(t_rank);

          if (const auto& t_bwd_pickup = _sol[target].bwd_pickups(t_rank);
              !(t_bwd_delivery + s_bwd_delivery <= t_v.capacity) ||
              !(t_bwd_pickup + s_bwd_pickup <= t_v.capacity)) {
            continue;
          }

          ReverseTwoOpt r(_input,
                          _sol_state,
                          _sol[source],
                          source,
                          s_rank,
                          _sol[target],
                          target,
                          t_rank);

          if (best_gains[source][target] < r.gain() && r.is_valid()) {
            best_gains[source][target] = r.gain();
            best_ops[source][target] = std::make_unique<ReverseTwoOpt>(r);
          }
        }
      }
    }

    if (_input.has_jobs()) {
      // Move(s) that don't make sense for shipment-only instances.

      // Relocate stuff
      for (const auto& [source, target] : s_t_pairs) {
        if (source == target || best_priorities[source] > 0 ||
            best_priorities[target] > 0 || _sol[source].size() == 0) {
          continue;
        }

        if (_sol[target].size() + 1 > _input.vehicles[target].max_tasks) {
          continue;
        }

        const auto& t_delivery_margin = _sol[target].delivery_margin();
        const auto& t_pickup_margin = _sol[target].pickup_margin();

        for (unsigned s_rank = 0; s_rank < _sol[source].size(); ++s_rank) {
          if (_sol_state.node_gains[source][s_rank] <=
              best_gains[source][target]) {
            // Except if addition cost in target route is negative
            // (!!), overall gain can't exceed current known best
            // gain.
            continue;
          }

          const auto s_job_rank = _sol[source].route[s_rank];
          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE ||
              !_input.vehicle_ok_with_job(target, s_job_rank)) {
            // Don't try moving (part of) a shipment or an
            // incompatible job.
            continue;
          }

          const auto& s_pickup = _input.jobs[s_job_rank].pickup;

          if (const auto& s_delivery = _input.jobs[s_job_rank].delivery;
              !(s_delivery <= t_delivery_margin) ||
              !(s_pickup <= t_pickup_margin)) {
            continue;
          }

          for (unsigned t_rank =
                 _sol_state.insertion_ranks_begin[target][s_job_rank];
               t_rank < _sol_state.insertion_ranks_end[target][s_job_rank];
               ++t_rank) {
            Relocate r(_input,
                       _sol_state,
                       _sol[source],
                       source,
                       s_rank,
                       _sol[target],
                       target,
                       t_rank);

            if (best_gains[source][target] < r.gain() && r.is_valid()) {
              best_gains[source][target] = r.gain();
              best_ops[source][target] = std::make_unique<Relocate>(r);
            }
          }
        }
      }

      // OrOpt stuff
      for (const auto& [source, target] : s_t_pairs) {
        if (source == target || best_priorities[source] > 0 ||
            best_priorities[target] > 0 || _sol[source].size() < 2) {
          continue;
        }

        if (_sol[target].size() + 2 > _input.vehicles[target].max_tasks) {
          continue;
        }

        const auto& t_delivery_margin = _sol[target].delivery_margin();
        const auto& t_pickup_margin = _sol[target].pickup_margin();

        for (unsigned s_rank = 0; s_rank < _sol[source].size() - 1; ++s_rank) {
          if (_sol_state.edge_gains[source][s_rank] <=
              best_gains[source][target]) {
            // Except if addition cost in route target is negative
            // (!!), overall gain can't exceed current known best gain.
            continue;
          }

          const auto s_job_rank = _sol[source].route[s_rank];
          const auto s_next_job_rank = _sol[source].route[s_rank + 1];

          if (!_input.vehicle_ok_with_job(target, s_job_rank) ||
              !_input.vehicle_ok_with_job(target, s_next_job_rank)) {
            continue;
          }

          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE ||
              _input.jobs[s_next_job_rank].type != JOB_TYPE::SINGLE) {
            // Don't try moving part of a shipment. Moving a full
            // shipment as an edge is not tested because it's a
            // special case of PDShift.
            continue;
          }

          const auto s_pickup = _input.jobs[s_job_rank].pickup +
                                _input.jobs[s_next_job_rank].pickup;

          if (const auto s_delivery = _input.jobs[s_job_rank].delivery +
                                      _input.jobs[s_next_job_rank].delivery;
              !(s_delivery <= t_delivery_margin) ||
              !(s_pickup <= t_pickup_margin)) {
            continue;
          }

          const auto insertion_start =
            std::max(_sol_state.insertion_ranks_begin[target][s_job_rank],
                     _sol_state.insertion_ranks_begin[target][s_next_job_rank]);
          const auto insertion_end =
            std::min(_sol_state.insertion_ranks_end[target][s_job_rank],
                     _sol_state.insertion_ranks_end[target][s_next_job_rank]);
          for (unsigned t_rank = insertion_start; t_rank < insertion_end;
               ++t_rank) {
            OrOpt r(_input,
                    _sol_state,
                    _sol[source],
                    source,
                    s_rank,
                    _sol[target],
                    target,
                    t_rank);

            auto& current_best = best_gains[source][target];
            if (current_best < r.gain_upper_bound() && r.is_valid() &&
                current_best < r.gain()) {
              current_best = r.gain();
              best_ops[source][target] = std::make_unique<OrOpt>(r);
            }
          }
        }
      }
    }

    // TSPFix stuff
    if (_input.apply_TSPFix() && !_input.has_shipments()) {
      for (const auto& [source, target] : s_t_pairs) {
        if (target != source || best_priorities[source] > 0 ||
            _sol[source].size() < 2) {
          continue;
        }

        TSPFix op(_input, _sol_state, _sol[source], source);

        if (best_gains[source][target] < op.gain() && op.is_valid()) {
          best_gains[source][target] = op.gain();
          best_ops[source][target] = std::make_unique<TSPFix>(op);
        }
      }
    }

    // IntraExchange stuff
    for (const auto& [source, target] : s_t_pairs) {
      if (source != target || best_priorities[source] > 0 ||
          _sol[source].size() < 3) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[source].size() - 2; ++s_rank) {
        const auto s_job_rank = _sol[source].route[s_rank];

        Index end_t_rank = _sol[source].size();
        if (_input.jobs[s_job_rank].type == JOB_TYPE::PICKUP) {
          // Don't move a pickup past its matching delivery.
          end_t_rank = _sol_state.matching_delivery_rank[source][s_rank];
        }

        const auto end_s =
          _sol_state.weak_insertion_ranks_end[source][s_job_rank];
        assert(end_s != 0);
        end_t_rank = std::min(end_t_rank, static_cast<Index>(end_s - 1));

        for (Index t_rank = s_rank + 2; t_rank < end_t_rank; ++t_rank) {
          if (_input.jobs[_sol[source].route[t_rank]].type ==
                JOB_TYPE::DELIVERY &&
              s_rank <= _sol_state.matching_pickup_rank[source][t_rank]) {
            // Don't move a delivery before its matching pickup.
            continue;
          }

          if (const auto t_job_rank = _sol[source].route[t_rank];
              _sol_state.weak_insertion_ranks_begin[source][t_job_rank] >
              s_rank + 1) {
            continue;
          }

          IntraExchange r(_input,
                          _sol_state,
                          _sol[source],
                          source,
                          s_rank,
                          t_rank);

          if (best_gains[source][source] < r.gain() && r.is_valid()) {
            best_gains[source][source] = r.gain();
            best_ops[source][source] = std::make_unique<IntraExchange>(r);
          }
        }
      }
    }

    // IntraCrossExchange stuff
    constexpr unsigned min_intra_cross_exchange_size = 5;
    for (const auto& [source, target] : s_t_pairs) {
      if (source != target || best_priorities[source] > 0 ||
          _sol[source].size() < min_intra_cross_exchange_size) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank <= _sol[source].size() - 4; ++s_rank) {
        const auto job_s_type = _input.jobs[_sol[source].route[s_rank]].type;
        const auto s_next_job_rank = _sol[source].route[s_rank + 1];

        const bool both_s_single =
          (job_s_type == JOB_TYPE::SINGLE) &&
          (_input.jobs[s_next_job_rank].type == JOB_TYPE::SINGLE);

        const bool is_s_pickup =
          (job_s_type == JOB_TYPE::PICKUP) &&
          (_sol_state.matching_delivery_rank[source][s_rank] == s_rank + 1);

        if (!both_s_single && !is_s_pickup) {
          continue;
        }

        Index end_t_rank = _sol[source].size() - 1;
        const auto end_s_next =
          _sol_state.weak_insertion_ranks_end[source][s_next_job_rank];
        assert(end_s_next > 1);
        end_t_rank = std::min(end_t_rank, static_cast<Index>(end_s_next - 2));

        for (unsigned t_rank = s_rank + 3; t_rank < end_t_rank; ++t_rank) {
          const auto& job_t_type = _input.jobs[_sol[target].route[t_rank]].type;

          const bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) &&
            (_input.jobs[_sol[target].route[t_rank + 1]].type ==
             JOB_TYPE::SINGLE);

          const bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) &&
            (_sol_state.matching_delivery_rank[target][t_rank] == t_rank + 1);

          if (!both_t_single && !is_t_pickup) {
            continue;
          }

          if (const auto t_job_rank = _sol[source].route[t_rank];
              _sol_state.weak_insertion_ranks_begin[source][t_job_rank] >
              s_rank + 2) {
            continue;
          }

          IntraCrossExchange r(_input,
                               _sol_state,
                               _sol[source],
                               source,
                               s_rank,
                               t_rank,
                               !is_s_pickup,
                               !is_t_pickup);

          auto& current_best = best_gains[source][target];
          if (current_best < r.gain_upper_bound() && r.is_valid() &&
              current_best < r.gain()) {
            current_best = r.gain();
            best_ops[source][source] = std::make_unique<IntraCrossExchange>(r);
          }
        }
      }
    }

    // IntraMixedExchange stuff
    for (const auto& [source, target] : s_t_pairs) {
      if (source != target || best_priorities[source] > 0 ||
          _sol[source].size() < 4) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[source].size(); ++s_rank) {
        const auto s_job_rank = _sol[source].route[s_rank];

        if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE) {
          // Don't try moving part of a shipment.
          continue;
        }

        Index end_t_rank = _sol[source].size() - 1;
        if (const auto end_s =
              _sol_state.weak_insertion_ranks_end[source][s_job_rank];
            end_s > 1) {
          end_t_rank = std::min(end_t_rank, static_cast<Index>(end_s - 2));
        } else {
          end_t_rank = std::min(end_t_rank, end_s);
        }

        for (unsigned t_rank = 0; t_rank < end_t_rank; ++t_rank) {
          if (t_rank <= s_rank + 1 && s_rank <= t_rank + 2) {
            continue;
          }

          const auto& job_t_type = _input.jobs[_sol[target].route[t_rank]].type;

          const bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) &&
            (_input.jobs[_sol[source].route[t_rank + 1]].type ==
             JOB_TYPE::SINGLE);

          const bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) &&
            (_sol_state.matching_delivery_rank[target][t_rank] == t_rank + 1);

          if (!both_t_single && !is_t_pickup) {
            continue;
          }

          if (const auto t_job_rank = _sol[source].route[t_rank];
              _sol_state.weak_insertion_ranks_begin[source][t_job_rank] >
              s_rank + 1) {
            continue;
          }

          IntraMixedExchange r(_input,
                               _sol_state,
                               _sol[source],
                               source,
                               s_rank,
                               t_rank,
                               !is_t_pickup);
          auto& current_best = best_gains[source][target];
          if (current_best < r.gain_upper_bound() && r.is_valid() &&
              current_best < r.gain()) {
            current_best = r.gain();
            best_ops[source][source] = std::make_unique<IntraMixedExchange>(r);
          }
        }
      }
    }

    // IntraRelocate stuff
    for (const auto& [source, target] : s_t_pairs) {
      if (source != target || best_priorities[source] > 0 ||
          _sol[source].size() < 2) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[source].size(); ++s_rank) {
        if (_sol_state.node_gains[source][s_rank] <=
            best_gains[source][source]) {
          // Except if addition cost in route is negative (!!),
          // overall gain can't exceed current known best gain.
          continue;
        }

        const auto s_job_rank = _sol[source].route[s_rank];
        auto begin_t_rank =
          _sol_state.weak_insertion_ranks_begin[source][s_job_rank];
        if (_input.jobs[s_job_rank].type == JOB_TYPE::DELIVERY) {
          // Don't move a delivery before its matching pickup.
          const Index begin_candidate =
            _sol_state.matching_pickup_rank[source][s_rank] + 1;
          begin_t_rank = std::max(begin_t_rank, begin_candidate);
        }

        auto end_t_rank = _sol[source].size();
        if (_input.jobs[s_job_rank].type == JOB_TYPE::PICKUP) {
          // Don't move a pickup past its matching delivery.
          end_t_rank = _sol_state.matching_delivery_rank[source][s_rank];
        }

        for (unsigned t_rank = begin_t_rank; t_rank < end_t_rank; ++t_rank) {
          if (t_rank == s_rank) {
            continue;
          }
          if (t_rank > s_rank &&
              _sol_state.weak_insertion_ranks_end[source][s_job_rank] <=
                t_rank + 1) {
            // Relocating past t_rank (new rank *after* removal) won't
            // work.
            break;
          }

          IntraRelocate r(_input,
                          _sol_state,
                          _sol[source],
                          source,
                          s_rank,
                          t_rank);

          if (best_gains[source][source] < r.gain() && r.is_valid()) {
            best_gains[source][source] = r.gain();
            best_ops[source][source] = std::make_unique<IntraRelocate>(r);
          }
        }
      }
    }

    // IntraOrOpt stuff
    for (const auto& [source, target] : s_t_pairs) {
      if (source != target || best_priorities[source] > 0 ||
          _sol[source].size() < 4) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[source].size() - 1; ++s_rank) {
        const auto& job_type = _input.jobs[_sol[source].route[s_rank]].type;

        const bool both_single =
          (job_type == JOB_TYPE::SINGLE) &&
          (_input.jobs[_sol[source].route[s_rank + 1]].type ==
           JOB_TYPE::SINGLE);

        const bool is_pickup =
          (job_type == JOB_TYPE::PICKUP) &&
          (_sol_state.matching_delivery_rank[source][s_rank] == s_rank + 1);

        if (!both_single && !is_pickup) {
          continue;
        }

        if (is_pickup) {
          if (_sol_state.pd_gains[source][s_rank] <=
              best_gains[source][source]) {
            // Except if addition cost in route is negative (!!),
            // overall gain can't exceed current known best gain.
            continue;
          }
        } else {
          // Regular single job.
          if (_sol_state.edge_gains[source][s_rank] <=
              best_gains[source][source]) {
            // Except if addition cost in route is negative (!!),
            // overall gain can't exceed current known best gain.
            continue;
          }
        }

        const auto s_job_rank = _sol[source].route[s_rank];
        const auto s_next_job_rank = _sol[source].route[s_rank + 1];
        const auto begin_t_rank =
          _sol_state.weak_insertion_ranks_begin[source][s_job_rank];

        for (unsigned t_rank = begin_t_rank; t_rank <= _sol[source].size() - 2;
             ++t_rank) {
          if (t_rank == s_rank) {
            continue;
          }
          if (t_rank > s_rank &&
              _sol_state.weak_insertion_ranks_end[source][s_next_job_rank] <=
                t_rank + 2) {
            // Relocating past t_rank (new rank *after* removal) won't
            // work.
            break;
          }

          IntraOrOpt r(_input,
                       _sol_state,
                       _sol[source],
                       source,
                       s_rank,
                       t_rank,
                       !is_pickup);
          auto& current_best = best_gains[source][target];
          if (current_best < r.gain_upper_bound() && r.is_valid() &&
              current_best < r.gain()) {
            current_best = r.gain();
            best_ops[source][source] = std::make_unique<IntraOrOpt>(r);
          }
        }
      }
    }

    // IntraTwoOpt stuff
    for (const auto& [source, target] : s_t_pairs) {
      if (source != target || best_priorities[source] > 0 ||
          _sol[source].size() < 4) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[source].size() - 2; ++s_rank) {
        const auto s_job_rank = _sol[source].route[s_rank];
        const auto end_s =
          _sol_state.weak_insertion_ranks_end[source][s_job_rank];
        assert(end_s != 0);
        auto end_t_rank = std::min(static_cast<Index>(_sol[source].size()),
                                   static_cast<Index>(end_s - 1));

        for (unsigned t_rank = s_rank + 2; t_rank < end_t_rank; ++t_rank) {
          IntraTwoOpt r(_input,
                        _sol_state,
                        _sol[source],
                        source,
                        s_rank,
                        t_rank);
          auto& current_best = best_gains[source][target];
          if (current_best < r.gain() && r.is_valid()) {
            current_best = r.gain();
            best_ops[source][source] = std::make_unique<IntraTwoOpt>(r);
          }
        }
      }
    }

    if (_input.has_shipments()) {
      // Move(s) that don't make sense for job-only instances.

      // PDShift stuff
      for (const auto& [source, target] : s_t_pairs) {
        if (source == target || best_priorities[source] > 0 ||
            best_priorities[target] > 0 || _sol[source].size() == 0) {
          // Don't try to put things from an empty vehicle.
          continue;
        }

        if (_sol[target].size() + 2 > _input.vehicles[target].max_tasks) {
          continue;
        }

        for (unsigned s_p_rank = 0; s_p_rank < _sol[source].size();
             ++s_p_rank) {
          if (_input.jobs[_sol[source].route[s_p_rank]].type !=
              JOB_TYPE::PICKUP) {
            continue;
          }

          // Matching delivery rank in source route.
          const Index s_d_rank =
            _sol_state.matching_delivery_rank[source][s_p_rank];

          if (!_input.vehicle_ok_with_job(target,
                                          _sol[source].route[s_p_rank]) ||
              !_input.vehicle_ok_with_job(target,
                                          _sol[source].route[s_d_rank])) {
            continue;
          }

          if (_sol_state.pd_gains[source][s_p_rank] <=
              best_gains[source][target]) {
            // Except if addition cost in target route is negative
            // (!!), overall gain can't exceed current known best
            // gain.
            continue;
          }

          if (const auto& v_s = _input.vehicles[source];
              !v_s.ok_for_range_bounds(_sol_state.route_evals[source] -
                                       _sol_state.pd_gains[source][s_p_rank])) {
            // Removing shipment from source route actually breaks
            // vehicle range constraints in source.
            continue;
          }

          PDShift pdr(_input,
                      _sol_state,
                      _sol[source],
                      source,
                      s_p_rank,
                      s_d_rank,
                      _sol[target],
                      target,
                      best_gains[source][target]);

          if (best_gains[source][target] < pdr.gain() && pdr.is_valid()) {
            best_gains[source][target] = pdr.gain();
            best_ops[source][target] = std::make_unique<PDShift>(pdr);
          }
        }
      }
    }

    if (!_input.has_homogeneous_locations() ||
        !_input.has_homogeneous_profiles() || !_input.has_homogeneous_costs()) {
      // RouteExchange stuff
      for (const auto& [source, target] : s_t_pairs) {
        if (target <= source || best_priorities[source] > 0 ||
            best_priorities[target] > 0 ||
            (_sol[source].size() == 0 && _sol[target].size() == 0) ||
            _sol_state.bwd_skill_rank[source][target] > 0 ||
            _sol_state.bwd_skill_rank[target][source] > 0) {
          // Different routes (and operator is symmetric), at least
          // one non-empty and valid wrt vehicle/job compatibility.
          continue;
        }

        const auto& s_v = _input.vehicles[source];
        const auto& t_v = _input.vehicles[target];

        if (_sol[source].size() > t_v.max_tasks ||
            _sol[target].size() > s_v.max_tasks) {
          continue;
        }

        const auto& s_deliveries_sum = _sol[source].job_deliveries_sum();
        const auto& s_pickups_sum = _sol[source].job_pickups_sum();
        const auto& t_deliveries_sum = _sol[target].job_deliveries_sum();

        if (const auto& t_pickups_sum = _sol[target].job_pickups_sum();
            !(t_deliveries_sum <= s_v.capacity) ||
            !(t_pickups_sum <= s_v.capacity) ||
            !(s_deliveries_sum <= t_v.capacity) ||
            !(s_pickups_sum <= t_v.capacity)) {
          continue;
        }

        RouteExchange re(_input,
                         _sol_state,
                         _sol[source],
                         source,
                         _sol[target],
                         target);

        if (best_gains[source][target] < re.gain() && re.is_valid()) {
          best_gains[source][target] = re.gain();
          best_ops[source][target] = std::make_unique<RouteExchange>(re);
        }
      }
    }

    if (_input.has_jobs()) {
      // SwapStar stuff
      for (const auto& [source, target] : s_t_pairs) {
        if (target <= source || // This operator is symmetric.
            best_priorities[source] > 0 || best_priorities[target] > 0 ||
            _sol[source].size() == 0 || _sol[target].size() == 0 ||
            !_input.vehicle_ok_with_vehicle(source, target) ||
            (_input.all_locations_have_coords() &&
             _input.vehicles[source].has_same_profile(
               _input.vehicles[target]) &&
             !_sol_state.route_bbox[source].intersects(
               _sol_state.route_bbox[target]))) {
          continue;
        }

        SwapStar r(_input,
                   _sol_state,
                   _sol[source],
                   source,
                   _sol[target],
                   target,
                   best_gains[source][target]);

        if (best_gains[source][target] < r.gain()) {
          best_gains[source][target] = r.gain();
          best_ops[source][target] = std::make_unique<SwapStar>(r);
        }
      }
    }

    if (!_input.has_homogeneous_locations() ||
        !_input.has_homogeneous_profiles() || !_input.has_homogeneous_costs()) {
      // RouteSplit stuff
      std::vector<Index> empty_route_ranks;
      empty_route_ranks.reserve(_input.vehicles.size());

      for (Index v = 0; v < _input.vehicles.size(); ++v) {
        if (_sol[v].empty()) {
          empty_route_ranks.push_back(v);
        }
      }

      if (empty_route_ranks.size() >= 2) {
        for (const auto& [source, target] : s_t_pairs) {
          if (target != source || best_priorities[source] > 0 ||
              _sol[source].size() < 2) {
            continue;
          }

          // RouteSplit stores a const& to empty_route_ranks, which
          // will be invalid in the unique_ptr created below after
          // empty_route_ranks goes out of scope. This is fine because
          // that ref is only stored to compute gain right below, not
          // to apply the operator later on.
          RouteSplit r(_input,
                       _sol_state,
                       _sol[source],
                       source,
                       empty_route_ranks,
                       _sol,
                       best_gains[source][target]);

          if (best_gains[source][target] < r.gain()) {
            best_gains[source][target] = r.gain();
            best_ops[source][target] = std::make_unique<RouteSplit>(r);
          }
        }
      }
    }

    // Find best overall move, first checking priority increase then
    // best gain if no priority increase is available.
    best_priority = 0;
    best_removal = std::numeric_limits<unsigned>::max();
    best_gain = Eval();
    Index best_source = 0;
    Index best_target = 0;

    for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
      if (std::tie(best_priority, best_removals[s_v], best_gain) <
          std::tie(best_priorities[s_v], best_removal, best_gains[s_v][s_v])) {
        best_priority = best_priorities[s_v];
        best_removal = best_removals[s_v];
        best_gain = best_gains[s_v][s_v];
        best_source = s_v;
        best_target = s_v;
      }
    }

    if (best_priority == 0) {
      for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
        for (unsigned t_v = 0; t_v < _nb_vehicles; ++t_v) {
          if (best_gain < best_gains[s_v][t_v]) {
            best_gain = best_gains[s_v][t_v];
            best_source = s_v;
            best_target = t_v;
          }
        }
      }
    }

    // Apply matching operator.
    if (best_priority > 0 || best_gain.cost > 0) {
      assert(best_ops[best_source][best_target] != nullptr);

      best_ops[best_source][best_target]->apply();

      auto update_candidates =
        best_ops[best_source][best_target]->update_candidates();

#ifndef NDEBUG
      // Update route costs.
      const auto previous_eval =
        std::accumulate(update_candidates.begin(),
                        update_candidates.end(),
                        Eval(),
                        [&](auto sum, auto c) {
                          return sum + _sol_state.route_evals[c];
                        });
#endif

      for (auto v_rank : update_candidates) {
        _sol_state.update_route_eval(_sol[v_rank]);
        _sol_state.update_route_bbox(_sol[v_rank]);
        _sol_state.update_costs(_sol[v_rank]);
        _sol_state.update_skills(_sol[v_rank]);
        _sol_state.update_priorities(_sol[v_rank]);
        _sol_state.set_insertion_ranks(_sol[v_rank]);
        _sol_state.set_node_gains(_sol[v_rank]);
        _sol_state.set_edge_gains(_sol[v_rank]);
        _sol_state.set_pd_matching_ranks(_sol[v_rank]);
        _sol_state.set_pd_gains(_sol[v_rank]);

        assert(_sol[v_rank].size() <= _input.vehicles[v_rank].max_tasks);
        assert(_input.vehicles[v_rank].ok_for_range_bounds(
          _sol_state.route_evals[v_rank]));
      }

#ifndef NDEBUG
      const auto new_eval =
        std::accumulate(update_candidates.begin(),
                        update_candidates.end(),
                        Eval(),
                        [&](auto sum, auto c) {
                          return sum + _sol_state.route_evals[c];
                        });
      assert(new_eval + best_gain == previous_eval);
#endif

      auto modified_vehicles =
        try_job_additions(best_ops[best_source][best_target]
                            ->addition_candidates(),
                          0);

      // Extend update_candidates in case a vehicle was not modified
      // by the operator itself but afterward by
      // try_job_additions. Can happen e.g. with UnassignedExchange.
      for (const auto v : update_candidates) {
        modified_vehicles.erase(v);
      }
      for (const auto v : modified_vehicles) {
        update_candidates.push_back(v);
      }

      // Set gains to zero for what needs to be recomputed in the next
      // round and set route pairs accordingly.
      s_t_pairs.clear();
      for (auto v_rank : update_candidates) {
        best_gains[v_rank].assign(_nb_vehicles, Eval());
        best_priorities[v_rank] = 0;
        best_removals[v_rank] = std::numeric_limits<unsigned>::max();
        best_ops[v_rank] = std::vector<std::unique_ptr<Operator>>(_nb_vehicles);
      }

      for (unsigned v = 0; v < _nb_vehicles; ++v) {
        for (auto v_rank : update_candidates) {
          if (_input.vehicle_ok_with_vehicle(v, v_rank)) {
            best_gains[v][v_rank] = Eval();
            best_ops[v][v_rank] = std::unique_ptr<Operator>();

            s_t_pairs.emplace_back(v, v_rank);
            if (v != v_rank) {
              s_t_pairs.emplace_back(v_rank, v);
            }
          }
        }
      }

      for (unsigned v = 0; v < _nb_vehicles; ++v) {
        if (best_ops[v][v] == nullptr) {
          continue;
        }

        bool invalidate_move = false;

        for (auto req_u : best_ops[v][v]->required_unassigned()) {
          if (!_sol_state.unassigned.contains(req_u)) {
            // This move should be invalidated because a required
            // unassigned job has been added by try_job_additions in
            // the meantime.
            invalidate_move = true;
            break;
          }
        }

        for (auto v_rank : update_candidates) {
          invalidate_move =
            invalidate_move || best_ops[v][v]->invalidated_by(v_rank);
        }

        if (invalidate_move) {
          best_gains[v][v] = Eval();
          best_priorities[v] = 0;
          best_removals[v] = std::numeric_limits<unsigned>::max();
          best_ops[v][v] = std::unique_ptr<Operator>();
          s_t_pairs.emplace_back(v, v);
        }
      }
    }
  }
}

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
void LocalSearch<Route,
                 UnassignedExchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 IntraTwoOpt,
                 PDShift,
                 RouteExchange,
                 SwapStar,
                 RouteSplit,
                 PriorityReplace,
                 TSPFix>::run() {
  bool try_ls_step = true;

  while (try_ls_step) {
    // A round of local search.
    run_ls_step();

    // Comparison with indicators for current solution.
    if (const utils::SolutionIndicators current_sol_indicators(_input, _sol);
        current_sol_indicators < _best_sol_indicators) {
      _best_sol_indicators = current_sol_indicators;
      _best_sol = _sol;
    } else {
      // No improvement so back to previous best known for further
      // steps.
      if (_best_sol_indicators < current_sol_indicators) {
        _sol = _best_sol;
        _sol_state.setup(_sol);
      }

      if (_completed_depth.has_value()) {
        // Rule out situation with first descent not yielding a better
        // solution.
        ++_completed_depth.value();
      }
    }

    if (!_completed_depth.has_value()) {
      // End of first descent.
      _completed_depth = 0;
    }

    // Try again on each improvement until we reach last job removal
    // level or deadline is met.
    assert(_completed_depth.has_value());
    auto nb_removal = _completed_depth.value() + 1;
    try_ls_step = (nb_removal <= _depth) &&
                  (!_deadline.has_value() || utils::now() < _deadline.value());

    if (try_ls_step) {
      // Get a looser situation by removing jobs.
      for (unsigned i = 0; i < nb_removal; ++i) {
        remove_from_routes();
        for (std::size_t v = 0; v < _sol.size(); ++v) {
          // Update what is required for consistency in
          // remove_from_route.
          _sol_state.update_costs(_sol[v]);
          _sol_state.update_route_eval(_sol[v]);
          _sol_state.update_route_bbox(_sol[v]);
          _sol_state.set_node_gains(_sol[v]);
          _sol_state.set_pd_matching_ranks(_sol[v]);
          _sol_state.set_pd_gains(_sol[v]);
        }
      }

      // Update stored data that has not been maintained while
      // removing.
      for (std::size_t v = 0; v < _sol.size(); ++v) {
        _sol_state.update_skills(_sol[v]);
        _sol_state.update_priorities(_sol[v]);
        _sol_state.set_insertion_ranks(_sol[v]);
        _sol_state.set_edge_gains(_sol[v]);
      }

      // Refill jobs.
      constexpr double refill_regret = 1.5;
      try_job_additions(_all_routes, refill_regret);
    }
  }
}

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
Eval LocalSearch<Route,
                 UnassignedExchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 IntraTwoOpt,
                 PDShift,
                 RouteExchange,
                 SwapStar,
                 RouteSplit,
                 PriorityReplace,
                 TSPFix>::job_route_cost(Index v_target, Index v, Index r) {
  assert(v != v_target);

  Eval eval = NO_EVAL;
  const auto job_index = _input.jobs[_sol[v].route[r]].index();

  const auto& vehicle = _input.vehicles[v_target];
  if (vehicle.has_start()) {
    const auto start_index = vehicle.start.value().index();
    const auto start_eval = vehicle.eval(start_index, job_index);
    eval = std::min(eval, start_eval);
  }
  if (vehicle.has_end()) {
    const auto end_index = vehicle.end.value().index();
    const auto end_eval = vehicle.eval(job_index, end_index);
    eval = std::min(eval, end_eval);
  }
  if (_sol[v_target].size() != 0) {
    const auto cheapest_from_rank =
      _sol_state.cheapest_job_rank_in_routes_from[v][v_target][r];
    const auto cheapest_from_index =
      _input.jobs[_sol[v_target].route[cheapest_from_rank]].index();
    const auto eval_from = vehicle.eval(cheapest_from_index, job_index);
    eval = std::min(eval, eval_from);

    const auto cheapest_to_rank =
      _sol_state.cheapest_job_rank_in_routes_to[v][v_target][r];
    const auto cheapest_to_index =
      _input.jobs[_sol[v_target].route[cheapest_to_rank]].index();
    const auto eval_to = vehicle.eval(job_index, cheapest_to_index);
    eval = std::min(eval, eval_to);
  }

  return eval;
}

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
Eval LocalSearch<Route,
                 UnassignedExchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 IntraTwoOpt,
                 PDShift,
                 RouteExchange,
                 SwapStar,
                 RouteSplit,
                 PriorityReplace,
                 TSPFix>::relocate_cost_lower_bound(Index v, Index r) {
  Eval best_bound = NO_EVAL;

  for (std::size_t other_v = 0; other_v < _sol.size(); ++other_v) {
    if (other_v == v ||
        !_input.vehicle_ok_with_job(other_v, _sol[v].route[r])) {
      continue;
    }

    best_bound = std::min(best_bound, job_route_cost(other_v, v, r));
  }

  return best_bound;
}

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
Eval LocalSearch<Route,
                 UnassignedExchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 IntraTwoOpt,
                 PDShift,
                 RouteExchange,
                 SwapStar,
                 RouteSplit,
                 PriorityReplace,
                 TSPFix>::relocate_cost_lower_bound(Index v,
                                                    Index r1,
                                                    Index r2) {
  Eval best_bound = NO_EVAL;

  for (std::size_t other_v = 0; other_v < _sol.size(); ++other_v) {
    if (other_v == v ||
        !_input.vehicle_ok_with_job(other_v, _sol[v].route[r1])) {
      continue;
    }

    best_bound =
      std::min(best_bound,
               job_route_cost(other_v, v, r1) + job_route_cost(other_v, v, r2));
  }

  return best_bound;
}

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
void LocalSearch<Route,
                 UnassignedExchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 IntraTwoOpt,
                 PDShift,
                 RouteExchange,
                 SwapStar,
                 RouteSplit,
                 PriorityReplace,
                 TSPFix>::remove_from_routes() {
  // Store nearest job from and to any job in any route for constant
  // time access down the line.
  for (std::size_t v1 = 0; v1 < _nb_vehicles; ++v1) {
    for (std::size_t v2 = 0; v2 < _nb_vehicles; ++v2) {
      if (v2 == v1) {
        continue;
      }
      _sol_state.update_cheapest_job_rank_in_routes(_sol[v1].route,
                                                    _sol[v2].route,
                                                    v1,
                                                    v2);
    }
  }

  // Remove best node candidate from all routes.
  std::vector<std::pair<Index, Index>> routes_and_ranks;
  routes_and_ranks.reserve(_sol.size());

  for (std::size_t v = 0; v < _sol.size(); ++v) {
    if (_sol[v].empty()) {
      continue;
    }

    // Try removing the best node (good gain on current route and
    // small cost to closest node in another compatible route).
    Index best_rank = 0;
    Eval best_gain = NO_GAIN;

    const auto& route_eval = _sol_state.route_evals[v];

    for (std::size_t r = 0; r < _sol[v].size(); ++r) {
      const auto& current_job = _input.jobs[_sol[v].route[r]];
      if (current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }

      Eval current_gain;
      bool valid_removal = false;

      if (current_job.type == JOB_TYPE::SINGLE) {
        const auto& removal_gain = _sol_state.node_gains[v][r];
        current_gain = removal_gain - relocate_cost_lower_bound(v, r);

        if (best_gain < current_gain) {
          // Only check validity if required.
          valid_removal =
            _input.vehicles[v].ok_for_range_bounds(route_eval - removal_gain) &&
            _sol[v].is_valid_removal(_input, r, 1);
        }
      } else {
        assert(current_job.type == JOB_TYPE::PICKUP);
        const auto delivery_r = _sol_state.matching_delivery_rank[v][r];
        const auto& removal_gain = _sol_state.pd_gains[v][r];
        current_gain =
          removal_gain - relocate_cost_lower_bound(v, r, delivery_r);

        if (best_gain < current_gain &&
            _input.vehicles[v].ok_for_range_bounds(route_eval - removal_gain)) {
          // Only check validity if required.
          if (delivery_r == r + 1) {
            valid_removal = _sol[v].is_valid_removal(_input, r, 2);
          } else {
            std::vector<Index> between_pd(_sol[v].route.begin() + r + 1,
                                          _sol[v].route.begin() + delivery_r);

            const auto delivery_between_pd =
              _sol[v].delivery_in_range(r + 1, delivery_r);

            valid_removal =
              _sol[v].is_valid_addition_for_tw(_input,
                                               delivery_between_pd,
                                               between_pd.begin(),
                                               between_pd.end(),
                                               r,
                                               delivery_r + 1);
          }
        }
      }

      if (best_gain < current_gain && valid_removal) {
        best_gain = current_gain;
        best_rank = r;
      }
    }

    if (best_gain != NO_GAIN) {
      routes_and_ranks.emplace_back(v, best_rank);
    }
  }

  for (const auto& [v, r] : routes_and_ranks) {
    _sol_state.unassigned.insert(_sol[v].route[r]);

    const auto& current_job = _input.jobs[_sol[v].route[r]];
    if (current_job.type == JOB_TYPE::SINGLE) {
      _sol[v].remove(_input, r, 1);
    } else {
      assert(current_job.type == JOB_TYPE::PICKUP);
      const auto delivery_r = _sol_state.matching_delivery_rank[v][r];
      _sol_state.unassigned.insert(_sol[v].route[delivery_r]);

      if (delivery_r == r + 1) {
        _sol[v].remove(_input, r, 2);
      } else {
        std::vector<Index> between_pd(_sol[v].route.begin() + r + 1,
                                      _sol[v].route.begin() + delivery_r);

        _sol[v].replace(_input,
                        _sol[v].delivery_in_range(r + 1, delivery_r),
                        between_pd.begin(),
                        between_pd.end(),
                        r,
                        delivery_r + 1);
      }
    }
  }
}

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
utils::SolutionIndicators LocalSearch<Route,
                                      UnassignedExchange,
                                      CrossExchange,
                                      MixedExchange,
                                      TwoOpt,
                                      ReverseTwoOpt,
                                      Relocate,
                                      OrOpt,
                                      IntraExchange,
                                      IntraCrossExchange,
                                      IntraMixedExchange,
                                      IntraRelocate,
                                      IntraOrOpt,
                                      IntraTwoOpt,
                                      PDShift,
                                      RouteExchange,
                                      SwapStar,
                                      RouteSplit,
                                      PriorityReplace,
                                      TSPFix>::indicators() const {
  return _best_sol_indicators;
}

template class LocalSearch<TWRoute,
                           vrptw::UnassignedExchange,
                           vrptw::CrossExchange,
                           vrptw::MixedExchange,
                           vrptw::TwoOpt,
                           vrptw::ReverseTwoOpt,
                           vrptw::Relocate,
                           vrptw::OrOpt,
                           vrptw::IntraExchange,
                           vrptw::IntraCrossExchange,
                           vrptw::IntraMixedExchange,
                           vrptw::IntraRelocate,
                           vrptw::IntraOrOpt,
                           vrptw::IntraTwoOpt,
                           vrptw::PDShift,
                           vrptw::RouteExchange,
                           vrptw::SwapStar,
                           vrptw::RouteSplit,
                           vrptw::PriorityReplace,
                           vrptw::TSPFix>;

template class LocalSearch<RawRoute,
                           cvrp::UnassignedExchange,
                           cvrp::CrossExchange,
                           cvrp::MixedExchange,
                           cvrp::TwoOpt,
                           cvrp::ReverseTwoOpt,
                           cvrp::Relocate,
                           cvrp::OrOpt,
                           cvrp::IntraExchange,
                           cvrp::IntraCrossExchange,
                           cvrp::IntraMixedExchange,
                           cvrp::IntraRelocate,
                           cvrp::IntraOrOpt,
                           cvrp::IntraTwoOpt,
                           cvrp::PDShift,
                           cvrp::RouteExchange,
                           cvrp::SwapStar,
                           cvrp::RouteSplit,
                           cvrp::PriorityReplace,
                           cvrp::TSPFix>;

} // namespace vroom::ls
