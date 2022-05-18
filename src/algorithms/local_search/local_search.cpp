/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/local_search.h"
#include "algorithms/local_search/insertion_search.h"
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
#include "problems/vrptw/operators/relocate.h"
#include "problems/vrptw/operators/reverse_two_opt.h"
#include "problems/vrptw/operators/route_exchange.h"
#include "problems/vrptw/operators/swap_star.h"
#include "problems/vrptw/operators/two_opt.h"
#include "problems/vrptw/operators/unassigned_exchange.h"
#include "utils/helpers.h"

namespace vroom {
namespace ls {

template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class RouteExchange>
LocalSearch<Route,
            UnassignedExchange,
            SwapStar,
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
            RouteExchange>::LocalSearch(const Input& input,
                                        std::vector<Route>& sol,
                                        unsigned max_nb_jobs_removal,
                                        const Timeout& timeout)
  : _input(input),
    _nb_vehicles(_input.vehicles.size()),
    _max_nb_jobs_removal(max_nb_jobs_removal),
    _deadline(timeout.has_value()
                ? utils::now() + std::chrono::milliseconds(timeout.value())
                : Deadline()),
    _all_routes(_nb_vehicles),
    _sol_state(input),
    _sol(sol),
    _best_sol(sol) {
  // Initialize all route indices.
  std::iota(_all_routes.begin(), _all_routes.end(), 0);

  // Setup solution state.
  _sol_state.setup(_sol);

  _best_sol_indicators.priority_sum =
    std::accumulate(_sol.begin(), _sol.end(), 0, [&](auto sum, const auto& r) {
      return sum + utils::priority_sum_for_route(_input, r.route);
    });

  _best_sol_indicators.unassigned = _sol_state.unassigned.size();

  Index v_rank = 0;
  _best_sol_indicators.cost =
    std::accumulate(_sol.begin(), _sol.end(), 0, [&](auto sum, const auto& r) {
      return sum + utils::route_cost_for_vehicle(_input, v_rank++, r.route);
    });

  _best_sol_indicators.used_vehicles =
    std::count_if(_sol.begin(), _sol.end(), [](const auto& r) {
      return !r.empty();
    });

#ifdef LOG_LS_OPERATORS
  tried_moves.fill(0);
  applied_moves.fill(0);
#endif
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
  } else {
    auto insert = compute_best_insertion_pd(input,
                                            sol_state,
                                            j,
                                            v,
                                            route,
                                            std::numeric_limits<Gain>::max());
    if (insert.cost < std::numeric_limits<Gain>::max()) {
      // Normalize cost per job for consistency with single jobs.
      insert.cost = static_cast<Gain>(static_cast<double>(insert.cost) / 2);
    }
    return insert;
  }
}

template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class RouteExchange>
void LocalSearch<Route,
                 UnassignedExchange,
                 SwapStar,
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
                 RouteExchange>::try_job_additions(const std::vector<Index>&
                                                     routes,
                                                   double regret_coeff) {

  bool job_added;

  std::vector<std::vector<RouteInsertion>> route_job_insertions;

  for (std::size_t i = 0; i < routes.size(); ++i) {
    route_job_insertions.push_back(
      std::vector<RouteInsertion>(_input.jobs.size(), empty_insert));

    const auto v = routes[i];
    for (const auto j : _sol_state.unassigned) {
      const auto& current_job = _input.jobs[j];
      if (current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }
      route_job_insertions[i][j] =
        compute_best_insertion(_input, _sol_state, j, v, _sol[v]);
    }
  }

  do {

    Priority best_priority = 0;
    RouteInsertion best_insertion = empty_insert;
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
        if (route_job_insertions[i][j].cost < smallest) {
          smallest_idx = i;
          second_smallest = smallest;
          smallest = route_job_insertions[i][j].cost;
        } else if (route_job_insertions[i][j].cost < second_smallest) {
          second_smallest = route_job_insertions[i][j].cost;
        }
      }

      // Find best route for current job based on cost of addition and
      // regret cost of not adding.
      for (std::size_t i = 0; i < routes.size(); ++i) {
        const auto addition_cost = route_job_insertions[i][j].cost;
        if (addition_cost == std::numeric_limits<Gain>::max()) {
          continue;
        }

        const auto& current_r = _sol[routes[i]];
        const auto& vehicle = _input.vehicles[routes[i]];
        bool is_pickup = (_input.jobs[j].type == JOB_TYPE::PICKUP);

        if (current_r.size() + (is_pickup ? 2 : 1) > vehicle.max_tasks) {
          continue;
        }

        const Gain regret_cost =
          (i == smallest_idx) ? second_smallest : smallest;

        const double eval = static_cast<double>(addition_cost) -
                            regret_coeff * static_cast<double>(regret_cost);

        if ((job_priority > best_priority) or
            (job_priority == best_priority and eval < best_cost)) {
          best_priority = job_priority;
          best_job_rank = j;
          best_route = routes[i];
          best_insertion = route_job_insertions[i][j];
          best_cost = eval;
          best_route_idx = i;
        }
      }
    }

    job_added = (best_cost < std::numeric_limits<double>::max());

    if (job_added) {
      _sol_state.unassigned.erase(best_job_rank);
      const auto& best_job = _input.jobs[best_job_rank];

      if (best_job.type == JOB_TYPE::SINGLE) {
        _sol[best_route].add(_input, best_job_rank, best_insertion.single_rank);
      } else {
        assert(best_job.type == JOB_TYPE::PICKUP);

        std::vector<Index> modified_with_pd({best_job_rank});
        std::copy(_sol[best_route].route.begin() + best_insertion.pickup_rank,
                  _sol[best_route].route.begin() + best_insertion.delivery_rank,
                  std::back_inserter(modified_with_pd));
        modified_with_pd.push_back(best_job_rank + 1);

        _sol[best_route].replace(_input,
                                 modified_with_pd.begin(),
                                 modified_with_pd.end(),
                                 best_insertion.pickup_rank,
                                 best_insertion.delivery_rank);

        assert(_sol_state.unassigned.find(best_job_rank + 1) !=
               _sol_state.unassigned.end());
        _sol_state.unassigned.erase(best_job_rank + 1);
      }

      // Update route/job insertions for best_route
      _sol_state.set_insertion_ranks(_sol[best_route], best_route);

      for (const auto j : _sol_state.unassigned) {
        const auto& current_job = _input.jobs[j];
        if (current_job.type == JOB_TYPE::DELIVERY) {
          continue;
        }
        route_job_insertions[best_route_idx][j] =
          compute_best_insertion(_input,
                                 _sol_state,
                                 j,
                                 best_route,
                                 _sol[best_route]);
      }
#ifndef NDEBUG
      // Update cost after addition.
      _sol_state.update_route_cost(_sol[best_route].route, best_route);
#endif
    }
  } while (job_added);
}

template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class RouteExchange>
void LocalSearch<Route,
                 UnassignedExchange,
                 SwapStar,
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
                 RouteExchange>::run_ls_step() {
  // Store best move involving a pair of routes.
  std::vector<std::vector<std::unique_ptr<Operator>>> best_ops(_nb_vehicles);
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    best_ops[v] = std::vector<std::unique_ptr<Operator>>(_nb_vehicles);
  }

  // List of source/target pairs we need to test (all related vehicles
  // at first).
  std::vector<std::pair<Index, Index>> s_t_pairs;
  for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
    for (unsigned t_v = 0; t_v < _nb_vehicles; ++t_v) {
      if (_input.vehicle_ok_with_vehicle(s_v, t_v)) {
        s_t_pairs.emplace_back(s_v, t_v);
      }
    }
  }

  // Store best gain for matching move.
  std::vector<std::vector<Gain>> best_gains(_nb_vehicles,
                                            std::vector<Gain>(_nb_vehicles, 0));

  // Store best priority increase for matching move. Only operators
  // involving a single route and unassigned jobs can change overall
  // priority (currently only UnassignedExchange).
  std::vector<Priority> best_priorities(_nb_vehicles, 0);

  Gain best_gain = 1;
  Priority best_priority = 0;

  while (best_gain > 0 or best_priority > 0) {
    if (_deadline.has_value() and _deadline.value() < utils::now()) {
      break;
    }

    // Operators applied to a pair of (different) routes.
    if (_input.has_jobs()) {
      // Move(s) that don't make sense for shipment-only instances.

      // UnassignedExchange stuff
      for (const Index u : _sol_state.unassigned) {
        if (_input.jobs[u].type != JOB_TYPE::SINGLE) {
          continue;
        }

        Priority u_priority = _input.jobs[u].priority;
        const auto& u_pickup = _input.jobs[u].pickup;
        const auto& u_delivery = _input.jobs[u].delivery;

        for (const auto& s_t : s_t_pairs) {
          if (s_t.first != s_t.second or
              !_input.vehicle_ok_with_job(s_t.first, u) or
              _sol[s_t.first].empty()) {
            continue;
          }

          const auto& delivery_margin = _sol[s_t.first].delivery_margin();
          const auto& pickup_margin = _sol[s_t.first].pickup_margin();

          const auto begin_t_rank_candidate =
            _sol_state.insertion_ranks_begin[s_t.first][u];
          const auto begin_t_rank_weak_candidate =
            _sol_state.weak_insertion_ranks_begin[s_t.first][u];
          const auto end_t_rank_candidate =
            _sol_state.insertion_ranks_end[s_t.first][u];
          const auto end_t_rank_weak_candidate =
            _sol_state.weak_insertion_ranks_end[s_t.first][u];

          for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
            const auto& current_job =
              _input.jobs[_sol[s_t.first].route[s_rank]];
            if (current_job.type != JOB_TYPE::SINGLE or
                u_priority < current_job.priority) {
              continue;
            }

            const Priority priority_gain = u_priority - current_job.priority;

            if (best_priorities[s_t.first] <= priority_gain) {
              if (!(u_delivery <= delivery_margin + current_job.delivery) or
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

              Index end_t_rank = _sol[s_t.first].size();
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
#ifdef LOG_LS_OPERATORS
                ++tried_moves[OperatorName::UnassignedExchange];
#endif
                UnassignedExchange r(_input,
                                     _sol_state,
                                     _sol_state.unassigned,
                                     _sol[s_t.first],
                                     s_t.first,
                                     s_rank,
                                     t_rank,
                                     u);

                bool better_if_valid =
                  (best_priorities[s_t.first] < priority_gain) or
                  (best_priorities[s_t.first] == priority_gain and
                   r.gain() > best_gains[s_t.first][s_t.first]);

                if (better_if_valid and r.is_valid()) {
                  best_priorities[s_t.first] = priority_gain;
                  // This may potentially define a negative value as
                  // best gain in case priority_gain is non-zero.
                  best_gains[s_t.first][s_t.first] = r.gain();
                  best_ops[s_t.first][s_t.first] =
                    std::make_unique<UnassignedExchange>(r);
                }
              }
            }
          }
        }
      }
    }

    // CrossExchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first or // This operator is symmetric.
          best_priorities[s_t.first] > 0 or best_priorities[s_t.second] > 0 or
          _sol[s_t.first].size() < 2 or _sol[s_t.second].size() < 2) {
        continue;
      }

      const auto& s_delivery_margin = _sol[s_t.first].delivery_margin();
      const auto& s_pickup_margin = _sol[s_t.first].pickup_margin();
      const auto& t_delivery_margin = _sol[s_t.second].delivery_margin();
      const auto& t_pickup_margin = _sol[s_t.second].pickup_margin();

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        const auto s_job_rank = _sol[s_t.first].route[s_rank];
        const auto s_next_job_rank = _sol[s_t.first].route[s_rank + 1];

        if (!_input.vehicle_ok_with_job(s_t.second, s_job_rank) or
            !_input.vehicle_ok_with_job(s_t.second, s_next_job_rank)) {
          continue;
        }

        const auto& job_s_type = _input.jobs[s_job_rank].type;

        bool both_s_single =
          (job_s_type == JOB_TYPE::SINGLE) and
          (_input.jobs[s_next_job_rank].type == JOB_TYPE::SINGLE);

        bool is_s_pickup =
          (job_s_type == JOB_TYPE::PICKUP) and
          (_sol_state.matching_delivery_rank[s_t.first][s_rank] == s_rank + 1);

        if (!both_s_single and !is_s_pickup) {
          continue;
        }

        const auto s_delivery = _input.jobs[s_job_rank].delivery +
                                _input.jobs[s_next_job_rank].delivery;
        const auto s_pickup =
          _input.jobs[s_job_rank].pickup + _input.jobs[s_next_job_rank].pickup;

        Index end_t_rank = _sol[s_t.second].size() - 1;
        const auto end_s =
          _sol_state.insertion_ranks_end[s_t.second][s_job_rank];
        const auto end_s_next =
          _sol_state.insertion_ranks_end[s_t.second][s_next_job_rank];
        end_t_rank = std::min(end_t_rank, end_s);
        end_t_rank = std::min(end_t_rank, end_s_next);

        Index begin_t_rank = 0;
        const auto begin_s =
          _sol_state.insertion_ranks_begin[s_t.second][s_job_rank];
        const auto begin_s_next =
          _sol_state.insertion_ranks_begin[s_t.second][s_next_job_rank];
        begin_t_rank = std::max(begin_t_rank, begin_s);
        begin_t_rank = std::max(begin_t_rank, begin_s_next);
        begin_t_rank = (begin_t_rank > 1) ? begin_t_rank - 2 : 0;

        for (unsigned t_rank = begin_t_rank; t_rank < end_t_rank; ++t_rank) {
          const auto t_job_rank = _sol[s_t.second].route[t_rank];
          const auto t_next_job_rank = _sol[s_t.second].route[t_rank + 1];

          if (!_input.vehicle_ok_with_job(s_t.first, t_job_rank) or
              !_input.vehicle_ok_with_job(s_t.first, t_next_job_rank)) {
            continue;
          }

          const auto& job_t_type = _input.jobs[t_job_rank].type;

          bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) and
            (_input.jobs[t_next_job_rank].type == JOB_TYPE::SINGLE);

          bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) and
            (_sol_state.matching_delivery_rank[s_t.second][t_rank] ==
             t_rank + 1);

          if (!both_t_single and !is_t_pickup) {
            continue;
          }

          if (s_rank >=
              std::min(_sol_state.insertion_ranks_end[s_t.first][t_job_rank],
                       _sol_state
                         .insertion_ranks_end[s_t.first][t_next_job_rank])) {
            continue;
          }

          Index begin_s_rank = 0;
          const auto begin_t =
            _sol_state.insertion_ranks_begin[s_t.first][t_job_rank];
          const auto begin_t_next =
            _sol_state.insertion_ranks_begin[s_t.first][t_next_job_rank];
          begin_s_rank = std::max(begin_s_rank, begin_t);
          begin_s_rank = std::max(begin_s_rank, begin_t_next);
          begin_s_rank = (begin_s_rank > 1) ? begin_s_rank - 2 : 0;
          if (s_rank < begin_s_rank) {
            continue;
          }

          const auto t_delivery = _input.jobs[t_job_rank].delivery +
                                  _input.jobs[t_next_job_rank].delivery;
          const auto t_pickup = _input.jobs[t_job_rank].pickup +
                                _input.jobs[t_next_job_rank].pickup;
          if (!(t_delivery <= s_delivery_margin + s_delivery) or
              !(t_pickup <= s_pickup_margin + s_pickup) or
              !(s_delivery <= t_delivery_margin + t_delivery) or
              !(s_pickup <= t_pickup_margin + t_pickup)) {
            continue;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::CrossExchange];
#endif
          CrossExchange r(_input,
                          _sol_state,
                          _sol[s_t.first],
                          s_t.first,
                          s_rank,
                          _sol[s_t.second],
                          s_t.second,
                          t_rank,
                          !is_s_pickup,
                          !is_t_pickup);

          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain_upper_bound() > current_best and r.is_valid() and
              r.gain() > current_best) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<CrossExchange>(r);
          }
        }
      }
    }

    if (_input.has_jobs()) {
      // MixedExchange stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or _sol[s_t.first].size() == 0 or
            _sol[s_t.second].size() < 2) {
          continue;
        }

        const auto& s_v = _input.vehicles[s_t.first];
        if (_sol[s_t.first].size() + 1 > s_v.max_tasks) {
          continue;
        }

        const auto& s_delivery_margin = _sol[s_t.first].delivery_margin();
        const auto& s_pickup_margin = _sol[s_t.first].pickup_margin();
        const auto& t_delivery_margin = _sol[s_t.second].delivery_margin();
        const auto& t_pickup_margin = _sol[s_t.second].pickup_margin();

        for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
          const auto s_job_rank = _sol[s_t.first].route[s_rank];
          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE or
              !_input.vehicle_ok_with_job(s_t.second, s_job_rank)) {
            // Don't try moving part of a shipment or an incompatible
            // job.
            continue;
          }

          const auto& s_delivery = _input.jobs[s_job_rank].delivery;
          const auto& s_pickup = _input.jobs[s_job_rank].pickup;

          auto end_t_rank =
            std::min(static_cast<Index>(_sol[s_t.second].size() - 1),
                     _sol_state.insertion_ranks_end[s_t.second][s_job_rank]);

          auto begin_t_rank =
            _sol_state.insertion_ranks_begin[s_t.second][s_job_rank];
          begin_t_rank = (begin_t_rank > 1) ? begin_t_rank - 2 : 0;

          for (unsigned t_rank = begin_t_rank; t_rank < end_t_rank; ++t_rank) {
            if (!_input.vehicle_ok_with_job(s_t.first,
                                            _sol[s_t.second].route[t_rank]) or
                !_input
                   .vehicle_ok_with_job(s_t.first,
                                        _sol[s_t.second].route[t_rank + 1])) {
              continue;
            }

            const auto t_job_rank = _sol[s_t.second].route[t_rank];
            const auto t_next_job_rank = _sol[s_t.second].route[t_rank + 1];
            const auto& job_t_type = _input.jobs[t_job_rank].type;

            bool both_t_single =
              (job_t_type == JOB_TYPE::SINGLE) and
              (_input.jobs[t_next_job_rank].type == JOB_TYPE::SINGLE);

            bool is_t_pickup =
              (job_t_type == JOB_TYPE::PICKUP) and
              (_sol_state.matching_delivery_rank[s_t.second][t_rank] ==
               t_rank + 1);

            if (!both_t_single and !is_t_pickup) {
              continue;
            }

            if (s_rank >=
                std::min(_sol_state.insertion_ranks_end[s_t.first][t_job_rank],
                         _sol_state
                           .insertion_ranks_end[s_t.first][t_next_job_rank])) {
              continue;
            }

            const auto source_begin =
              std::min(_sol_state.insertion_ranks_begin[s_t.first][t_job_rank],
                       _sol_state
                         .insertion_ranks_begin[s_t.first][t_next_job_rank]);
            if (source_begin > s_rank + 1) {
              continue;
            }

            const auto t_delivery = _input.jobs[t_job_rank].delivery +
                                    _input.jobs[t_next_job_rank].delivery;
            const auto t_pickup = _input.jobs[t_job_rank].pickup +
                                  _input.jobs[t_next_job_rank].pickup;
            if (!(t_delivery <= s_delivery_margin + s_delivery) or
                !(t_pickup <= s_pickup_margin + s_pickup) or
                !(s_delivery <= t_delivery_margin + t_delivery) or
                !(s_pickup <= t_pickup_margin + t_pickup)) {
              continue;
            }

#ifdef LOG_LS_OPERATORS
            ++tried_moves[OperatorName::MixedExchange];
#endif
            MixedExchange r(_input,
                            _sol_state,
                            _sol[s_t.first],
                            s_t.first,
                            s_rank,
                            _sol[s_t.second],
                            s_t.second,
                            t_rank,
                            !is_t_pickup);

            auto& current_best = best_gains[s_t.first][s_t.second];
            if (r.gain_upper_bound() > current_best and r.is_valid() and
                r.gain() > current_best) {
              current_best = r.gain();
              best_ops[s_t.first][s_t.second] =
                std::make_unique<MixedExchange>(r);
            }
          }
        }
      }
    }

    // TwoOpt stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first or // This operator is symmetric.
          best_priorities[s_t.first] > 0 or best_priorities[s_t.second] > 0) {
        continue;
      }

      const auto& s_v = _input.vehicles[s_t.first];
      const auto& t_v = _input.vehicles[s_t.second];

      // Determine first ranks for inner loops based on vehicles/jobs
      // compatibility along the routes.
      unsigned first_s_rank = 0;
      const auto first_s_candidate =
        _sol_state.bwd_skill_rank[s_t.first][s_t.second];
      if (first_s_candidate > 0) {
        first_s_rank = first_s_candidate - 1;
      }

      int first_t_rank = 0;
      const auto first_t_candidate =
        _sol_state.bwd_skill_rank[s_t.second][s_t.first];
      if (first_t_candidate > 0) {
        first_t_rank = first_t_candidate - 1;
      }

      for (unsigned s_rank = first_s_rank; s_rank < _sol[s_t.first].size();
           ++s_rank) {
        if (_sol[s_t.first].has_pending_delivery_after_rank(s_rank)) {
          continue;
        }

        const auto& s_fwd_delivery = _sol[s_t.first].fwd_deliveries(s_rank);
        const auto& s_fwd_pickup = _sol[s_t.first].fwd_pickups(s_rank);
        const auto& s_bwd_delivery = _sol[s_t.first].bwd_deliveries(s_rank);
        const auto& s_bwd_pickup = _sol[s_t.first].bwd_pickups(s_rank);

        Index end_t_rank = _sol[s_t.second].size();
        if (s_rank + 1 < _sol[s_t.first].size()) {
          // There is a route end after s_rank in source route.
          const auto s_next_job_rank = _sol[s_t.first].route[s_rank + 1];
          end_t_rank =
            std::min(end_t_rank,
                     _sol_state
                       .weak_insertion_ranks_end[s_t.second][s_next_job_rank]);
        }

        for (int t_rank = end_t_rank - 1; t_rank >= first_t_rank; --t_rank) {
          if (_sol[s_t.second].has_pending_delivery_after_rank(t_rank)) {
            continue;
          }

          if (t_rank + 1 < static_cast<int>(_sol[s_t.second].size())) {
            // There is a route end after t_rank in target route.
            const auto t_next_job_rank = _sol[s_t.second].route[t_rank + 1];
            if (_sol_state
                  .weak_insertion_ranks_end[s_t.first][t_next_job_rank] <=
                s_rank) {
              // Job right after t_rank won't fit after job at s_rank
              // in source route.
              continue;
            }
          }

          if (s_rank + _sol[s_t.second].size() - t_rank > s_v.max_tasks or
              t_rank + _sol[s_t.first].size() - s_rank > t_v.max_tasks) {
            continue;
          }

          const auto& t_bwd_delivery = _sol[s_t.second].bwd_deliveries(t_rank);
          const auto& t_bwd_pickup = _sol[s_t.second].bwd_pickups(t_rank);

          if (!(s_fwd_delivery + t_bwd_delivery <= s_v.capacity) or
              !(s_fwd_pickup + t_bwd_pickup <= s_v.capacity)) {
            // Stop current loop since we're going backward with
            // t_rank.
            break;
          }

          const auto& t_fwd_delivery = _sol[s_t.second].fwd_deliveries(t_rank);
          const auto& t_fwd_pickup = _sol[s_t.second].fwd_pickups(t_rank);

          if (!(t_fwd_delivery + s_bwd_delivery <= t_v.capacity) or
              !(t_fwd_pickup + s_bwd_pickup <= t_v.capacity)) {
            continue;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::TwoOpt];
#endif
          TwoOpt r(_input,
                   _sol_state,
                   _sol[s_t.first],
                   s_t.first,
                   s_rank,
                   _sol[s_t.second],
                   s_t.second,
                   t_rank);

          if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<TwoOpt>(r);
          }
        }
      }
    }

    // ReverseTwoOpt stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
          best_priorities[s_t.second] > 0) {
        continue;
      }

      const auto& s_v = _input.vehicles[s_t.first];
      const auto& t_v = _input.vehicles[s_t.second];

      // Determine first rank for inner loop based on vehicles/jobs
      // compatibility along the routes.
      unsigned first_s_rank = 0;
      const auto first_s_candidate =
        _sol_state.bwd_skill_rank[s_t.first][s_t.second];
      if (first_s_candidate > 0) {
        first_s_rank = first_s_candidate - 1;
      }

      for (unsigned s_rank = first_s_rank; s_rank < _sol[s_t.first].size();
           ++s_rank) {
        if (_sol[s_t.first].has_delivery_after_rank(s_rank)) {
          continue;
        }

        const auto& s_fwd_delivery = _sol[s_t.first].fwd_deliveries(s_rank);
        const auto& s_fwd_pickup = _sol[s_t.first].fwd_pickups(s_rank);
        const auto& s_bwd_delivery = _sol[s_t.first].bwd_deliveries(s_rank);
        const auto& s_bwd_pickup = _sol[s_t.first].bwd_pickups(s_rank);

        Index begin_t_rank = 0;
        if (s_rank + 1 < _sol[s_t.first].size()) {
          // There is a route end after s_rank in source route.
          const auto s_next_job_rank = _sol[s_t.first].route[s_rank + 1];
          const auto unmodified_begin =
            _sol_state.weak_insertion_ranks_begin[s_t.second][s_next_job_rank];
          if (unmodified_begin > 0) {
            begin_t_rank = unmodified_begin - 1;
          }
        }

        for (unsigned t_rank = begin_t_rank;
             t_rank < _sol_state.fwd_skill_rank[s_t.second][s_t.first];
             ++t_rank) {
          if (_sol[s_t.second].has_pickup_up_to_rank(t_rank)) {
            continue;
          }

          const auto t_job_rank = _sol[s_t.second].route[t_rank];
          if (_sol_state.weak_insertion_ranks_end[s_t.first][t_job_rank] <=
              s_rank) {
            // Job at t_rank won't fit after job at s_rank in source
            // route.
            continue;
          }

          if (s_rank + t_rank + 2 > s_v.max_tasks or
              (_sol[s_t.first].size() - s_rank - 1) +
                  (_sol[s_t.second].size() - t_rank - 1) >
                t_v.max_tasks) {
            continue;
          }

          const auto& t_fwd_delivery = _sol[s_t.second].fwd_deliveries(t_rank);
          const auto& t_fwd_pickup = _sol[s_t.second].fwd_pickups(t_rank);

          if (!(s_fwd_delivery + t_fwd_delivery <= s_v.capacity) or
              !(s_fwd_pickup + t_fwd_pickup <= s_v.capacity)) {
            break;
          }

          const auto& t_bwd_delivery = _sol[s_t.second].bwd_deliveries(t_rank);
          const auto& t_bwd_pickup = _sol[s_t.second].bwd_pickups(t_rank);

          if (!(t_bwd_delivery + s_bwd_delivery <= t_v.capacity) or
              !(t_bwd_pickup + s_bwd_pickup <= t_v.capacity)) {
            continue;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::ReverseTwoOpt];
#endif
          ReverseTwoOpt r(_input,
                          _sol_state,
                          _sol[s_t.first],
                          s_t.first,
                          s_rank,
                          _sol[s_t.second],
                          s_t.second,
                          t_rank);

          if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<ReverseTwoOpt>(r);
          }
        }
      }
    }

    if (_input.has_jobs()) {
      // Move(s) that don't make sense for shipment-only instances.

      // Relocate stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or _sol[s_t.first].size() == 0) {
          continue;
        }

        const auto& v_t = _input.vehicles[s_t.second];
        if (_sol[s_t.second].size() + 1 > v_t.max_tasks) {
          continue;
        }

        const auto& t_delivery_margin = _sol[s_t.second].delivery_margin();
        const auto& t_pickup_margin = _sol[s_t.second].pickup_margin();

        for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
          if (_sol_state.node_gains[s_t.first][s_rank] <=
              best_gains[s_t.first][s_t.second]) {
            // Except if addition cost in route s_t.second is negative
            // (!!), overall gain can't exceed current known best gain.
            continue;
          }

          const auto s_job_rank = _sol[s_t.first].route[s_rank];
          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE or
              !_input.vehicle_ok_with_job(s_t.second, s_job_rank)) {
            // Don't try moving (part of) a shipment or an
            // incompatible job.
            continue;
          }

          const auto& s_pickup = _input.jobs[s_job_rank].pickup;
          const auto& s_delivery = _input.jobs[s_job_rank].delivery;

          if (!(s_delivery <= t_delivery_margin) or
              !(s_pickup <= t_pickup_margin)) {
            continue;
          }

          for (unsigned t_rank =
                 _sol_state.insertion_ranks_begin[s_t.second][s_job_rank];
               t_rank < _sol_state.insertion_ranks_end[s_t.second][s_job_rank];
               ++t_rank) {
#ifdef LOG_LS_OPERATORS
            ++tried_moves[OperatorName::Relocate];
#endif
            Relocate r(_input,
                       _sol_state,
                       _sol[s_t.first],
                       s_t.first,
                       s_rank,
                       _sol[s_t.second],
                       s_t.second,
                       t_rank);

            if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
              best_gains[s_t.first][s_t.second] = r.gain();
              best_ops[s_t.first][s_t.second] = std::make_unique<Relocate>(r);
            }
          }
        }
      }

      // OrOpt stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or _sol[s_t.first].size() < 2) {
          continue;
        }

        const auto& v_t = _input.vehicles[s_t.second];
        if (_sol[s_t.second].size() + 2 > v_t.max_tasks) {
          continue;
        }

        const auto& t_delivery_margin = _sol[s_t.second].delivery_margin();
        const auto& t_pickup_margin = _sol[s_t.second].pickup_margin();

        for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1;
             ++s_rank) {
          if (_sol_state.edge_gains[s_t.first][s_rank] <=
              best_gains[s_t.first][s_t.second]) {
            // Except if addition cost in route s_t.second is negative
            // (!!), overall gain can't exceed current known best gain.
            continue;
          }

          const auto s_job_rank = _sol[s_t.first].route[s_rank];
          const auto s_next_job_rank = _sol[s_t.first].route[s_rank + 1];

          if (!_input.vehicle_ok_with_job(s_t.second, s_job_rank) or
              !_input.vehicle_ok_with_job(s_t.second, s_next_job_rank)) {
            continue;
          }

          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE or
              _input.jobs[s_next_job_rank].type != JOB_TYPE::SINGLE) {
            // Don't try moving part of a shipment. Moving a full
            // shipment as an edge is not tested because it's a
            // special case of PDShift.
            continue;
          }

          const auto s_pickup = _input.jobs[s_job_rank].pickup +
                                _input.jobs[s_next_job_rank].pickup;
          const auto s_delivery = _input.jobs[s_job_rank].delivery +
                                  _input.jobs[s_next_job_rank].delivery;

          if (!(s_delivery <= t_delivery_margin) or
              !(s_pickup <= t_pickup_margin)) {
            continue;
          }

          const auto insertion_start =
            std::max(_sol_state.insertion_ranks_begin[s_t.second][s_job_rank],
                     _sol_state
                       .insertion_ranks_begin[s_t.second][s_next_job_rank]);
          const auto insertion_end =
            std::min(_sol_state.insertion_ranks_end[s_t.second][s_job_rank],
                     _sol_state
                       .insertion_ranks_end[s_t.second][s_next_job_rank]);
          for (unsigned t_rank = insertion_start; t_rank < insertion_end;
               ++t_rank) {
#ifdef LOG_LS_OPERATORS
            ++tried_moves[OperatorName::OrOpt];
#endif
            OrOpt r(_input,
                    _sol_state,
                    _sol[s_t.first],
                    s_t.first,
                    s_rank,
                    _sol[s_t.second],
                    s_t.second,
                    t_rank);

            auto& current_best = best_gains[s_t.first][s_t.second];
            if (r.gain_upper_bound() > current_best and r.is_valid() and
                r.gain() > current_best) {
              current_best = r.gain();
              best_ops[s_t.first][s_t.second] = std::make_unique<OrOpt>(r);
            }
          }
        }
      }
    }

    // IntraExchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 3) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 2; ++s_rank) {
        const auto s_job_rank = _sol[s_t.first].route[s_rank];

        Index end_t_rank = _sol[s_t.first].size();
        if (_input.jobs[s_job_rank].type == JOB_TYPE::PICKUP) {
          // Don't move a pickup past its matching delivery.
          end_t_rank = _sol_state.matching_delivery_rank[s_t.first][s_rank];
        }

        const auto end_s =
          _sol_state.weak_insertion_ranks_end[s_t.first][s_job_rank];
        assert(end_s != 0);
        end_t_rank = std::min(end_t_rank, static_cast<Index>(end_s - 1));

        for (Index t_rank = s_rank + 2; t_rank < end_t_rank; ++t_rank) {
          if (_input.jobs[_sol[s_t.first].route[t_rank]].type ==
                JOB_TYPE::DELIVERY and
              s_rank <= _sol_state.matching_pickup_rank[s_t.first][t_rank]) {
            // Don't move a delivery before its matching pickup.
            continue;
          }

          const auto t_job_rank = _sol[s_t.first].route[t_rank];
          if (_sol_state.weak_insertion_ranks_begin[s_t.first][t_job_rank] >
              s_rank + 1) {
            continue;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::IntraExchange];
#endif
          IntraExchange r(_input,
                          _sol_state,
                          _sol[s_t.first],
                          s_t.first,
                          s_rank,
                          t_rank);

          if (r.gain() > best_gains[s_t.first][s_t.first] and r.is_valid()) {
            best_gains[s_t.first][s_t.first] = r.gain();
            best_ops[s_t.first][s_t.first] = std::make_unique<IntraExchange>(r);
          }
        }
      }
    }

    // IntraCrossExchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 5) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank <= _sol[s_t.first].size() - 4;
           ++s_rank) {
        const auto job_s_type = _input.jobs[_sol[s_t.first].route[s_rank]].type;
        const auto s_next_job_rank = _sol[s_t.first].route[s_rank + 1];

        bool both_s_single =
          (job_s_type == JOB_TYPE::SINGLE) and
          (_input.jobs[s_next_job_rank].type == JOB_TYPE::SINGLE);

        bool is_s_pickup =
          (job_s_type == JOB_TYPE::PICKUP) and
          (_sol_state.matching_delivery_rank[s_t.first][s_rank] == s_rank + 1);

        if (!both_s_single and !is_s_pickup) {
          continue;
        }

        Index end_t_rank = _sol[s_t.first].size() - 1;
        const auto end_s_next =
          _sol_state.weak_insertion_ranks_end[s_t.first][s_next_job_rank];
        assert(end_s_next > 1);
        end_t_rank = std::min(end_t_rank, static_cast<Index>(end_s_next - 2));

        for (unsigned t_rank = s_rank + 3; t_rank < end_t_rank; ++t_rank) {
          const auto& job_t_type =
            _input.jobs[_sol[s_t.second].route[t_rank]].type;

          bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) and
            (_input.jobs[_sol[s_t.second].route[t_rank + 1]].type ==
             JOB_TYPE::SINGLE);

          bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) and
            (_sol_state.matching_delivery_rank[s_t.second][t_rank] ==
             t_rank + 1);

          if (!both_t_single and !is_t_pickup) {
            continue;
          }

          const auto t_job_rank = _sol[s_t.first].route[t_rank];
          if (_sol_state.weak_insertion_ranks_begin[s_t.first][t_job_rank] >
              s_rank + 2) {
            continue;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::IntraCrossExchange];
#endif
          IntraCrossExchange r(_input,
                               _sol_state,
                               _sol[s_t.first],
                               s_t.first,
                               s_rank,
                               t_rank,
                               !is_s_pickup,
                               !is_t_pickup);

          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain_upper_bound() > current_best and r.is_valid() and
              r.gain() > current_best) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.first] =
              std::make_unique<IntraCrossExchange>(r);
          }
        }
      }
    }

    // IntraMixedExchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 4) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        const auto s_job_rank = _sol[s_t.first].route[s_rank];

        if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE) {
          // Don't try moving part of a shipment.
          continue;
        }

        Index end_t_rank = _sol[s_t.first].size() - 1;
        const auto end_s =
          _sol_state.weak_insertion_ranks_end[s_t.first][s_job_rank];
        if (end_s > 1) {
          end_t_rank = std::min(end_t_rank, static_cast<Index>(end_s - 2));
        } else {
          end_t_rank = std::min(end_t_rank, end_s);
        }

        for (unsigned t_rank = 0; t_rank < end_t_rank; ++t_rank) {
          if (t_rank <= s_rank + 1 and s_rank <= t_rank + 2) {
            continue;
          }

          const auto& job_t_type =
            _input.jobs[_sol[s_t.second].route[t_rank]].type;

          bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) and
            (_input.jobs[_sol[s_t.first].route[t_rank + 1]].type ==
             JOB_TYPE::SINGLE);

          bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) and
            (_sol_state.matching_delivery_rank[s_t.second][t_rank] ==
             t_rank + 1);

          if (!both_t_single and !is_t_pickup) {
            continue;
          }

          const auto t_job_rank = _sol[s_t.first].route[t_rank];
          if (_sol_state.weak_insertion_ranks_begin[s_t.first][t_job_rank] >
              s_rank + 1) {
            continue;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::IntraMixedExchange];
#endif
          IntraMixedExchange r(_input,
                               _sol_state,
                               _sol[s_t.first],
                               s_t.first,
                               s_rank,
                               t_rank,
                               !is_t_pickup);
          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain_upper_bound() > current_best and r.is_valid() and
              r.gain() > current_best) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.first] =
              std::make_unique<IntraMixedExchange>(r);
          }
        }
      }
    }

    // IntraRelocate stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 2) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        if (_sol_state.node_gains[s_t.first][s_rank] <=
            best_gains[s_t.first][s_t.first]) {
          // Except if addition cost in route is negative (!!),
          // overall gain can't exceed current known best gain.
          continue;
        }

        const auto s_job_rank = _sol[s_t.first].route[s_rank];
        auto begin_t_rank =
          _sol_state.weak_insertion_ranks_begin[s_t.first][s_job_rank];
        if (_input.jobs[s_job_rank].type == JOB_TYPE::DELIVERY) {
          // Don't move a delivery before its matching pickup.
          Index begin_candidate =
            _sol_state.matching_pickup_rank[s_t.first][s_rank] + 1;
          begin_t_rank = std::max(begin_t_rank, begin_candidate);
        }

        auto end_t_rank = _sol[s_t.first].size();
        if (_input.jobs[s_job_rank].type == JOB_TYPE::PICKUP) {
          // Don't move a pickup past its matching delivery.
          end_t_rank = _sol_state.matching_delivery_rank[s_t.first][s_rank];
        }

        for (unsigned t_rank = begin_t_rank; t_rank < end_t_rank; ++t_rank) {
          if (t_rank == s_rank) {
            continue;
          }
          if (t_rank > s_rank and
              _sol_state.weak_insertion_ranks_end[s_t.first][s_job_rank] <=
                t_rank + 1) {
            // Relocating past t_rank (new rank *after* removal) won't
            // work.
            break;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::IntraRelocate];
#endif
          IntraRelocate r(_input,
                          _sol_state,
                          _sol[s_t.first],
                          s_t.first,
                          s_rank,
                          t_rank);

          if (r.gain() > best_gains[s_t.first][s_t.first] and r.is_valid()) {
            best_gains[s_t.first][s_t.first] = r.gain();
            best_ops[s_t.first][s_t.first] = std::make_unique<IntraRelocate>(r);
          }
        }
      }
    }

    // IntraOrOpt stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 4) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        const auto& job_type = _input.jobs[_sol[s_t.first].route[s_rank]].type;

        bool both_single =
          (job_type == JOB_TYPE::SINGLE) and
          (_input.jobs[_sol[s_t.first].route[s_rank + 1]].type ==
           JOB_TYPE::SINGLE);

        bool is_pickup =
          (job_type == JOB_TYPE::PICKUP) and
          (_sol_state.matching_delivery_rank[s_t.first][s_rank] == s_rank + 1);

        if (!both_single and !is_pickup) {
          continue;
        }

        if (is_pickup) {
          if (_sol_state.pd_gains[s_t.first][s_rank] <=
              best_gains[s_t.first][s_t.first]) {
            // Except if addition cost in route is negative (!!),
            // overall gain can't exceed current known best gain.
            continue;
          }
        } else {
          // Regular single job.
          if (_sol_state.edge_gains[s_t.first][s_rank] <=
              best_gains[s_t.first][s_t.first]) {
            // Except if addition cost in route is negative (!!),
            // overall gain can't exceed current known best gain.
            continue;
          }
        }

        const auto s_job_rank = _sol[s_t.first].route[s_rank];
        const auto s_next_job_rank = _sol[s_t.first].route[s_rank + 1];
        const auto begin_t_rank =
          _sol_state.weak_insertion_ranks_begin[s_t.first][s_job_rank];

        for (unsigned t_rank = begin_t_rank;
             t_rank <= _sol[s_t.first].size() - 2;
             ++t_rank) {
          if (t_rank == s_rank) {
            continue;
          }
          if (t_rank > s_rank and
              _sol_state.weak_insertion_ranks_end[s_t.first][s_next_job_rank] <=
                t_rank + 2) {
            // Relocating past t_rank (new rank *after* removal) won't
            // work.
            break;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::IntraOrOpt];
#endif
          IntraOrOpt r(_input,
                       _sol_state,
                       _sol[s_t.first],
                       s_t.first,
                       s_rank,
                       t_rank,
                       !is_pickup);
          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain_upper_bound() > current_best and r.is_valid() and
              r.gain() > current_best) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.first] = std::make_unique<IntraOrOpt>(r);
          }
        }
      }
    }

    // IntraTwoOpt stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 4) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 2; ++s_rank) {
        const auto s_job_rank = _sol[s_t.first].route[s_rank];
        const auto end_s =
          _sol_state.weak_insertion_ranks_end[s_t.first][s_job_rank];
        assert(end_s != 0);
        auto end_t_rank = std::min(static_cast<Index>(_sol[s_t.first].size()),
                                   static_cast<Index>(end_s - 1));

        for (unsigned t_rank = s_rank + 2; t_rank < end_t_rank; ++t_rank) {
#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::IntraTwoOpt];
#endif
          IntraTwoOpt r(_input,
                        _sol_state,
                        _sol[s_t.first],
                        s_t.first,
                        s_rank,
                        t_rank);
          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain() > current_best and r.is_valid()) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.first] = std::make_unique<IntraTwoOpt>(r);
          }
        }
      }
    }

    if (_input.has_shipments()) {
      // Move(s) that don't make sense for job-only instances.

      // PDShift stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or _sol[s_t.first].size() == 0) {
          // Don't try to put things from an empty vehicle.
          continue;
        }

        const auto& v_t = _input.vehicles[s_t.second];
        if (_sol[s_t.second].size() + 2 > v_t.max_tasks) {
          continue;
        }

        for (unsigned s_p_rank = 0; s_p_rank < _sol[s_t.first].size();
             ++s_p_rank) {
          if (_input.jobs[_sol[s_t.first].route[s_p_rank]].type !=
              JOB_TYPE::PICKUP) {
            continue;
          }

          // Matching delivery rank in source route.
          unsigned s_d_rank =
            _sol_state.matching_delivery_rank[s_t.first][s_p_rank];

          if (!_input.vehicle_ok_with_job(s_t.second,
                                          _sol[s_t.first].route[s_p_rank]) or
              !_input.vehicle_ok_with_job(s_t.second,
                                          _sol[s_t.first].route[s_d_rank])) {
            continue;
          }

          if (_sol_state.pd_gains[s_t.first][s_p_rank] <=
              best_gains[s_t.first][s_t.second]) {
            // Except if addition cost in route s_t.second is negative
            // (!!), overall gain can't exceed current known best gain.
            continue;
          }

#ifdef LOG_LS_OPERATORS
          ++tried_moves[OperatorName::PDShift];
#endif
          PDShift pdr(_input,
                      _sol_state,
                      _sol[s_t.first],
                      s_t.first,
                      s_p_rank,
                      s_d_rank,
                      _sol[s_t.second],
                      s_t.second,
                      best_gains[s_t.first][s_t.second]);

          if (pdr.gain() > best_gains[s_t.first][s_t.second] and
              pdr.is_valid()) {
            best_gains[s_t.first][s_t.second] = pdr.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<PDShift>(pdr);
          }
        }
      }
    }

    if (!_input.has_homogeneous_locations() or
        !_input.has_homogeneous_profiles()) {
      // RouteExchange stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.second <= s_t.first or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or
            (_sol[s_t.first].size() == 0 and _sol[s_t.second].size() == 0) or
            _sol_state.bwd_skill_rank[s_t.first][s_t.second] > 0 or
            _sol_state.bwd_skill_rank[s_t.second][s_t.first] > 0) {
          // Different routes (and operator is symmetric), at least
          // one non-empty and valid wrt vehicle/job compatibility.
          continue;
        }

        const auto& s_v = _input.vehicles[s_t.first];
        const auto& t_v = _input.vehicles[s_t.second];

        if (_sol[s_t.first].size() > t_v.max_tasks or
            _sol[s_t.second].size() > s_v.max_tasks) {
          continue;
        }

        const auto& s_deliveries_sum = _sol[s_t.first].job_deliveries_sum();
        const auto& s_pickups_sum = _sol[s_t.first].job_pickups_sum();
        const auto& t_deliveries_sum = _sol[s_t.second].job_deliveries_sum();
        const auto& t_pickups_sum = _sol[s_t.second].job_pickups_sum();

        if (!(t_deliveries_sum <= s_v.capacity) or
            !(t_pickups_sum <= s_v.capacity) or
            !(s_deliveries_sum <= t_v.capacity) or
            !(s_pickups_sum <= t_v.capacity)) {
          continue;
        }

#ifdef LOG_LS_OPERATORS
        ++tried_moves[OperatorName::RouteExchange];
#endif
        RouteExchange re(_input,
                         _sol_state,
                         _sol[s_t.first],
                         s_t.first,
                         _sol[s_t.second],
                         s_t.second);

        if (re.gain() > best_gains[s_t.first][s_t.second] and re.is_valid()) {
          best_gains[s_t.first][s_t.second] = re.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<RouteExchange>(re);
        }
      }
    }

    if (_input.has_jobs()) {
      // SwapStar stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.second <= s_t.first or // This operator is symmetric.
            best_priorities[s_t.first] > 0 or best_priorities[s_t.second] > 0 or
            _sol[s_t.first].size() == 0 or _sol[s_t.second].size() == 0 or
            !_input.vehicle_ok_with_vehicle(s_t.first, s_t.second)) {
          continue;
        }

#ifdef LOG_LS_OPERATORS
        ++tried_moves[OperatorName::SwapStar];
#endif
        SwapStar r(_input,
                   _sol_state,
                   _sol[s_t.first],
                   s_t.first,
                   _sol[s_t.second],
                   s_t.second,
                   best_gains[s_t.first][s_t.second]);

        if (r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<SwapStar>(r);
        }
      }
    }

    // Find best overall move, first checking priority increase then
    // best gain if no priority increase is available.
    best_priority = 0;
    best_gain = 0;
    Index best_source = 0;
    Index best_target = 0;

    for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
      if (best_priorities[s_v] > best_priority) {
        best_priority = best_priorities[s_v];
        best_gain = best_gains[s_v][s_v];
        best_source = s_v;
        best_target = s_v;
      }
    }

    if (best_priority == 0) {
      for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
        for (unsigned t_v = 0; t_v < _nb_vehicles; ++t_v) {
          if (best_gains[s_v][t_v] > best_gain) {
            best_gain = best_gains[s_v][t_v];
            best_source = s_v;
            best_target = t_v;
          }
        }
      }
    }

    // Apply matching operator.
    if (best_priority > 0 or best_gain > 0) {
      assert(best_ops[best_source][best_target] != nullptr);

      best_ops[best_source][best_target]->apply();

      auto update_candidates =
        best_ops[best_source][best_target]->update_candidates();

#ifdef LOG_LS_OPERATORS
      ++applied_moves.at(best_ops[best_source][best_target]->get_name());
#endif

#ifndef NDEBUG
      // Update route costs.
      const auto previous_cost =
        std::accumulate(update_candidates.begin(),
                        update_candidates.end(),
                        0,
                        [&](auto sum, auto c) {
                          return sum + _sol_state.route_costs[c];
                        });
      for (auto v_rank : update_candidates) {
        _sol_state.update_route_cost(_sol[v_rank].route, v_rank);
        assert(_sol[v_rank].size() <= _input.vehicles[v_rank].max_tasks);
      }
      const auto new_cost =
        std::accumulate(update_candidates.begin(),
                        update_candidates.end(),
                        0,
                        [&](auto sum, auto c) {
                          return sum + _sol_state.route_costs[c];
                        });
      assert(new_cost + best_gain == previous_cost);
#endif

      for (auto v_rank : update_candidates) {
        _sol_state.set_insertion_ranks(_sol[v_rank], v_rank);
      }

      try_job_additions(best_ops[best_source][best_target]
                          ->addition_candidates(),
                        0);

      for (auto v_rank : update_candidates) {
        // Running update_costs only after try_job_additions is fine.
        _sol_state.update_costs(_sol[v_rank].route, v_rank);

        _sol_state.update_skills(_sol[v_rank].route, v_rank);

        _sol_state.set_node_gains(_sol[v_rank].route, v_rank);
        _sol_state.set_edge_gains(_sol[v_rank].route, v_rank);
        _sol_state.set_pd_matching_ranks(_sol[v_rank].route, v_rank);
        _sol_state.set_pd_gains(_sol[v_rank].route, v_rank);
      }

      // Set gains to zero for what needs to be recomputed in the next
      // round and set route pairs accordingly.
      s_t_pairs.clear();
      for (auto v_rank : update_candidates) {
        best_gains[v_rank].assign(_nb_vehicles, 0);
        best_priorities[v_rank] = 0;
        best_ops[v_rank] = std::vector<std::unique_ptr<Operator>>(_nb_vehicles);
      }

      for (unsigned v = 0; v < _nb_vehicles; ++v) {
        for (auto v_rank : update_candidates) {
          if (_input.vehicle_ok_with_vehicle(v, v_rank)) {
            best_gains[v][v_rank] = 0;
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
        for (auto req_u : best_ops[v][v]->required_unassigned()) {
          if (_sol_state.unassigned.find(req_u) ==
              _sol_state.unassigned.end()) {
            // This move should be invalidated because a required
            // unassigned job has been added by try_job_additions in
            // the meantime.
            best_gains[v][v] = 0;
            best_priorities[v] = 0;
            best_ops[v][v] = std::unique_ptr<Operator>();
            s_t_pairs.emplace_back(v, v);
          }
        }
      }
    }
  }
}

template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class RouteExchange>
void LocalSearch<Route,
                 UnassignedExchange,
                 SwapStar,
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
                 RouteExchange>::run() {
  bool try_ls_step = true;
  bool first_step = true;

  unsigned current_nb_removal = 1;

  while (try_ls_step) {
    // A round of local search.
    run_ls_step();

    // Indicators for current solution.
    utils::SolutionIndicators current_sol_indicators;
    current_sol_indicators.priority_sum =
      std::accumulate(_sol.begin(),
                      _sol.end(),
                      0,
                      [&](auto sum, const auto& r) {
                        return sum +
                               utils::priority_sum_for_route(_input, r.route);
                      });

    current_sol_indicators.unassigned = _sol_state.unassigned.size();

    Index v_rank = 0;
    current_sol_indicators.cost =
      std::accumulate(_sol.begin(),
                      _sol.end(),
                      0,
                      [&](auto sum, const auto& r) {
                        return sum + utils::route_cost_for_vehicle(_input,
                                                                   v_rank++,
                                                                   r.route);
                      });

    current_sol_indicators.used_vehicles =
      std::count_if(_sol.begin(), _sol.end(), [](const auto& r) {
        return !r.empty();
      });

    if (current_sol_indicators < _best_sol_indicators) {
      _best_sol_indicators = current_sol_indicators;
      _best_sol = _sol;
    } else {
      if (!first_step) {
        ++current_nb_removal;
      }
      if (_best_sol_indicators < current_sol_indicators) {
        // Back to best known solution for further steps.
        _sol = _best_sol;
        _sol_state.setup(_sol);
      }
    }

    // Try again on each improvement until we reach last job removal
    // level or deadline is met.
    try_ls_step = (current_nb_removal <= _max_nb_jobs_removal) and
                  (!_deadline.has_value() or utils::now() < _deadline.value());

    if (try_ls_step) {
      // Get a looser situation by removing jobs.
      for (unsigned i = 0; i < current_nb_removal; ++i) {
        remove_from_routes();
        for (std::size_t v = 0; v < _sol.size(); ++v) {
          _sol_state.set_node_gains(_sol[v].route, v);
          _sol_state.set_pd_matching_ranks(_sol[v].route, v);
          _sol_state.set_pd_gains(_sol[v].route, v);
        }
      }

      // Update insertion ranks ranges.
      for (std::size_t v = 0; v < _sol.size(); ++v) {
        _sol_state.set_insertion_ranks(_sol[v], v);
      }

      // Refill jobs.
      try_job_additions(_all_routes, 1.5);

      // Reset what is needed in solution state.
      _sol_state.setup(_sol);
    }

    first_step = false;
  }
}

#ifdef LOG_LS_OPERATORS
template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class RouteExchange>
std::array<OperatorStats, OperatorName::MAX>
LocalSearch<Route,
            UnassignedExchange,
            SwapStar,
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
            RouteExchange>::get_stats() const {
  std::array<OperatorStats, OperatorName::MAX> stats;
  for (auto op = 0; op < OperatorName::MAX; ++op) {
    stats[op] = OperatorStats(tried_moves.at(op), applied_moves.at(op));
  }

  return stats;
}
#endif

template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class RouteExchange>
Gain LocalSearch<Route,
                 UnassignedExchange,
                 SwapStar,
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
                 RouteExchange>::job_route_cost(Index v_target,
                                                Index v,
                                                Index r) {
  assert(v != v_target);

  Gain cost = static_cast<Gain>(INFINITE_COST);
  const auto job_index = _input.jobs[_sol[v].route[r]].index();

  const auto& vehicle = _input.vehicles[v_target];
  if (vehicle.has_start()) {
    const auto start_index = vehicle.start.value().index();
    const Gain start_cost = vehicle.cost(start_index, job_index);
    cost = std::min(cost, start_cost);
  }
  if (vehicle.has_end()) {
    const auto end_index = vehicle.end.value().index();
    const Gain end_cost = vehicle.cost(job_index, end_index);
    cost = std::min(cost, end_cost);
  }
  if (_sol[v_target].size() != 0) {
    const auto cheapest_from_rank =
      _sol_state.cheapest_job_rank_in_routes_from[v][v_target][r];
    const auto cheapest_from_index =
      _input.jobs[_sol[v_target].route[cheapest_from_rank]].index();
    const Gain cost_from = vehicle.cost(cheapest_from_index, job_index);
    cost = std::min(cost, cost_from);

    const auto cheapest_to_rank =
      _sol_state.cheapest_job_rank_in_routes_to[v][v_target][r];
    const auto cheapest_to_index =
      _input.jobs[_sol[v_target].route[cheapest_to_rank]].index();
    const Gain cost_to = vehicle.cost(job_index, cheapest_to_index);
    cost = std::min(cost, cost_to);
  }

  return cost;
}

template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class RouteExchange>
Gain LocalSearch<Route,
                 UnassignedExchange,
                 SwapStar,
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
                 RouteExchange>::relocate_cost_lower_bound(Index v, Index r) {
  Gain best_bound = static_cast<Gain>(INFINITE_COST);

  for (std::size_t other_v = 0; other_v < _sol.size(); ++other_v) {
    if (other_v == v or
        !_input.vehicle_ok_with_job(other_v, _sol[v].route[r])) {
      continue;
    }

    best_bound = std::min(best_bound, job_route_cost(other_v, v, r));
  }

  return best_bound;
}

template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class RouteExchange>
Gain LocalSearch<Route,
                 UnassignedExchange,
                 SwapStar,
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
                 RouteExchange>::relocate_cost_lower_bound(Index v,
                                                           Index r1,
                                                           Index r2) {
  Gain best_bound = static_cast<Gain>(INFINITE_COST);

  for (std::size_t other_v = 0; other_v < _sol.size(); ++other_v) {
    if (other_v == v or
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
          class SwapStar,
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
          class RouteExchange>
void LocalSearch<Route,
                 UnassignedExchange,
                 SwapStar,
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
                 RouteExchange>::remove_from_routes() {
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

  for (std::size_t v = 0; v < _sol.size(); ++v) {
    if (_sol[v].empty()) {
      continue;
    }

    // Try removing the best node (good gain on current route and
    // small cost to closest node in another compatible route).
    Index best_rank = 0;
    Gain best_gain = std::numeric_limits<Gain>::min();

    for (std::size_t r = 0; r < _sol[v].size(); ++r) {
      const auto& current_job = _input.jobs[_sol[v].route[r]];
      if (current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }

      Gain current_gain;
      bool valid_removal;

      if (current_job.type == JOB_TYPE::SINGLE) {
        current_gain =
          _sol_state.node_gains[v][r] - relocate_cost_lower_bound(v, r);

        if (current_gain > best_gain) {
          // Only check validity if required.
          valid_removal = _sol[v].is_valid_removal(_input, r, 1);
        }
      } else {
        assert(current_job.type == JOB_TYPE::PICKUP);
        auto delivery_r = _sol_state.matching_delivery_rank[v][r];
        current_gain = _sol_state.pd_gains[v][r] -
                       relocate_cost_lower_bound(v, r, delivery_r);

        if (current_gain > best_gain) {
          // Only check validity if required.
          if (delivery_r == r + 1) {
            valid_removal = _sol[v].is_valid_removal(_input, r, 2);
          } else {
            std::vector<Index> between_pd(_sol[v].route.begin() + r + 1,
                                          _sol[v].route.begin() + delivery_r);

            valid_removal = _sol[v].is_valid_addition_for_tw(_input,
                                                             between_pd.begin(),
                                                             between_pd.end(),
                                                             r,
                                                             delivery_r + 1);
          }
        }
      }

      if (current_gain > best_gain and valid_removal) {
        best_gain = current_gain;
        best_rank = r;
      }
    }

    if (best_gain > std::numeric_limits<Gain>::min()) {
      routes_and_ranks.push_back(std::make_pair(v, best_rank));
    }
  }

  for (const auto& r_r : routes_and_ranks) {
    const auto v = r_r.first;
    const auto r = r_r.second;

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
          class SwapStar,
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
          class RouteExchange>
utils::SolutionIndicators LocalSearch<Route,
                                      UnassignedExchange,
                                      SwapStar,
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
                                      RouteExchange>::indicators() const {
  return _best_sol_indicators;
}

template class LocalSearch<TWRoute,
                           vrptw::UnassignedExchange,
                           vrptw::SwapStar,
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
                           vrptw::RouteExchange>;

template class LocalSearch<RawRoute,
                           cvrp::UnassignedExchange,
                           cvrp::SwapStar,
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
                           cvrp::RouteExchange>;

} // namespace ls
} // namespace vroom
