/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

#include "algorithms/heuristics/heuristics.h"
#include "utils/helpers.h"
#include "utils/soft_skills.h"

namespace vroom::heuristics {

namespace {

// Preference-aware insertion ranking support for inputs that contain
// soft-skills (Task 15). Jobs without soft-skills bypass every
// computation below, and inputs with no soft-skills at all never even
// reach the enclosing has_soft_skills() branches, so the upstream
// heuristics stay untouched on legacy inputs.

// On-demand reference metrics for a single soft-skills job: best
// feasible insertion cost and distance across the vehicles that do
// not match its soft-skills, plus a possible deferral target (a
// matching vehicle that is still to be processed and whose best
// feasible insertion stays within threshold of the non-matching
// reference). Computed per heuristic call from the current content of
// all routes, never cached beyond that scope.
struct SoftJobReference {
  bool nonmatch_feasible{false};
  Cost best_nonmatch_cost{std::numeric_limits<Cost>::max()};
  Distance best_nonmatch_distance{std::numeric_limits<Distance>::max()};

  bool defer_to_match{false};
  Index defer_target{0};
};

// Best feasible insertion metrics of a (single or pickup) job into the
// current content of one route, as seen from its own vehicle.
struct VehicleInsertionScan {
  bool feasible{false};
  Cost best_cost{std::numeric_limits<Cost>::max()};
  Distance best_distance{std::numeric_limits<Distance>::max()};
  // Eval of the lowest-cost feasible insertion.
  Eval best_eval;
};

template <class Route>
inline VehicleInsertionScan
scan_soft_feasible_insertions(const Input& input,
                              Index job_rank,
                              Index v_rank,
                              const Route& route) {
  VehicleInsertionScan scan;

  if (!input.vehicle_ok_with_job(v_rank, job_rank)) {
    return scan;
  }

  const auto& vehicle = input.vehicles[v_rank];
  const auto& job = input.jobs[job_rank];
  const auto route_eval =
    utils::route_eval_for_vehicle(input, v_rank, route.route);

  auto consider = [&](const Eval& e) {
    scan.feasible = true;
    if (e.cost < scan.best_cost) {
      scan.best_cost = e.cost;
      scan.best_eval = e;
    }
    if (e.distance < scan.best_distance) {
      scan.best_distance = e.distance;
    }
  };

  if (job.type == JOB_TYPE::SINGLE
      && route.size() + 1 <= vehicle.max_tasks) {
    for (Index r = 0; r <= route.size(); ++r) {
      const auto e =
        utils::addition_eval(input, job_rank, vehicle, route.route, r);
      if (vehicle.ok_for_range_bounds(route_eval + e)
          && route.is_valid_addition_for_capacity(input,
                                                  job.pickup,
                                                  job.delivery,
                                                  r)
          && route.is_valid_addition_for_tw(input, job_rank, r)) {
        consider(e);
      }
    }
  }

  if (job.type == JOB_TYPE::PICKUP
      && route.size() + 2 <= vehicle.max_tasks) {
    // Only considering pickup and delivery inserted next to each
    // other, which is enough as a preference reference point.
    std::vector<Index> p_d({job_rank, static_cast<Index>(job_rank + 1)});
    for (Index r = 0; r <= route.size(); ++r) {
      const auto e = utils::addition_eval(input,
                                          job_rank,
                                          vehicle,
                                          route.route,
                                          r,
                                          r + 1);
      if (vehicle.ok_for_range_bounds(route_eval + e)
          && route.is_valid_addition_for_capacity_inclusion(input,
                                                            input.zero_amount(),
                                                            p_d.begin(),
                                                            p_d.end(),
                                                            r,
                                                            r)
          && route.is_valid_addition_for_tw(input,
                                            input.zero_amount(),
                                            p_d.begin(),
                                            p_d.end(),
                                            r,
                                            r)) {
        consider(e);
      }
    }
  }

  return scan;
}

// Soft-skills threshold check for a candidate insertion against the
// non-matching reference of its job. With no feasible non-matching
// alternative at all, the comparison automatically passes (Q3).
inline bool soft_pref_gain(const Eval& candidate,
                           const SoftJobReference& reference,
                           const SoftSkillsOptions& opts,
                           bool distance_data_available) {
  if (!reference.nonmatch_feasible) {
    return true;
  }

  return threshold_passes(candidate.cost,
                          reference.best_nonmatch_cost,
                          opts.max_cost_factor,
                          candidate.distance,
                          reference.best_nonmatch_distance,
                          opts.max_distance_factor,
                          opts.threshold_mode,
                          opts.missing_distance_policy,
                          distance_data_available);
}

template <class Route>
inline std::vector<std::optional<SoftJobReference>>
build_soft_references(const Input& input,
                      const std::vector<Route>& routes,
                      const std::set<Index>& unassigned,
                      const std::vector<Index>& remaining_ranks,
                      const SoftSkillsOptions& opts,
                      bool distance_data_available) {
  std::vector<std::optional<SoftJobReference>> references(input.jobs.size());

  for (const auto job_rank : unassigned) {
    const auto& job = input.jobs[job_rank];
    if (job.soft_skills.empty() || job.type == JOB_TYPE::DELIVERY) {
      continue;
    }

    const auto mode = effective_match_mode(job, opts);
    SoftJobReference reference;

    std::optional<Index> cheapest_match_rank;
    Cost cheapest_match_cost = std::numeric_limits<Cost>::max();

    const auto nb_vehicles = input.vehicles.size();
    for (Index v_rank = 0; v_rank < nb_vehicles; ++v_rank) {
      const bool matches = soft_skill_match(job.soft_skills,
                                            input.vehicles[v_rank].soft_skills,
                                            mode);
      const auto scan =
        scan_soft_feasible_insertions(input, job_rank, v_rank, routes[v_rank]);

      if (!scan.feasible) {
        continue;
      }

      if (!matches) {
        reference.nonmatch_feasible = true;
        reference.best_nonmatch_cost =
          std::min(reference.best_nonmatch_cost, scan.best_cost);
        reference.best_nonmatch_distance =
          std::min(reference.best_nonmatch_distance, scan.best_distance);
      } else if(std::find(remaining_ranks.begin(),
                          remaining_ranks.end(),
                          v_rank)
                != remaining_ranks.end()
                && scan.best_cost < cheapest_match_cost) {
        // Cheapest matching vehicle still to be processed: candidate
        // deferral target.
        cheapest_match_rank = v_rank;
        cheapest_match_cost = scan.best_cost;
      }
    }

    if (cheapest_match_rank.has_value()) {
      reference.defer_target = *cheapest_match_rank;
      const auto target_scan = scan_soft_feasible_insertions(input,
                                                             job_rank,
                                                             reference.defer_target,
                                                             routes[reference.defer_target]);
      reference.defer_to_match =
        !reference.nonmatch_feasible
        || soft_pref_gain(target_scan.best_eval,
                          reference,
                          opts,
                          distance_data_available);
    }

    references[job_rank] = reference;
  }

  return references;
}

} // anonymous namespace

// Add seed job to route if required and return current cost of route
// without vehicle fixed cost.
template <class Route>
inline void seed_route(const Input& input,
                       Route& route,
                       const std::vector<Route>& routes,
                       const std::vector<Index>& remaining_ranks,
                       INIT init,
                       const std::vector<std::vector<Eval>>& evals,
                       std::set<Index>& unassigned,
                       auto job_not_ok) {
  assert(route.empty() && init != INIT::NONE);

  const auto v_rank = route.v_rank;
  const auto& vehicle = input.vehicles[v_rank];

  const auto& opts = input.get_soft_skills_options();
  const bool distance_data_available = input.has_distance_data();

  std::vector<std::optional<SoftJobReference>> soft_references;
  if (input.has_soft_skills()) {
    soft_references = build_soft_references(input,
                                           routes,
                                           unassigned,
                                           remaining_ranks,
                                           opts,
                                           distance_data_available);
  }

  // Initialize current route with the "best" valid job.
  bool init_ok = false;

  Amount higher_amount(input.zero_amount());
  Cost furthest_cost = 0;
  Cost nearest_cost = std::numeric_limits<Cost>::max();
  Duration earliest_deadline = std::numeric_limits<Duration>::max();
  Index best_job_rank = 0;
  for (const auto job_rank : unassigned) {
    const auto& current_job = input.jobs[job_rank];

    if (!input.vehicle_ok_with_job(v_rank, job_rank) ||
        current_job.type == JOB_TYPE::DELIVERY || job_not_ok(job_rank)) {
      continue;
    }

    if (input.has_soft_skills() && !current_job.soft_skills.empty()) {
      const auto& reference = *soft_references[job_rank];
      const auto matches = soft_skill_match(current_job.soft_skills,
                                            vehicle.soft_skills,
                                            effective_match_mode(current_job,
                                                                 opts));
      if (!matches && reference.defer_to_match &&
          reference.defer_target != v_rank) {
        // A remaining vehicle matches this job's soft-skills and can
        // insert it within threshold: the current vehicle yields the
        // seed slot to that vehicle.
        continue;
      }
    }

    const bool is_pickup = (current_job.type == JOB_TYPE::PICKUP);

    if (route.size() + (is_pickup ? 2 : 1) > vehicle.max_tasks) {
      continue;
    }

    bool try_validity = false;

    if (init == INIT::HIGHER_AMOUNT) {
      try_validity = (higher_amount < current_job.pickup ||
                      higher_amount < current_job.delivery);
    }
    if (init == INIT::EARLIEST_DEADLINE) {
      const Duration current_deadline =
        is_pickup ? input.jobs[job_rank + 1].tws.back().end
                  : current_job.tws.back().end;
      try_validity = (current_deadline < earliest_deadline);
    }
    if (init == INIT::FURTHEST) {
      try_validity = (furthest_cost < evals[job_rank][v_rank].cost);
    }
    if (init == INIT::NEAREST) {
      try_validity = (evals[job_rank][v_rank].cost < nearest_cost);
    }

    if (!try_validity) {
      continue;
    }

    bool is_valid = (vehicle.ok_for_range_bounds(evals[job_rank][v_rank])) &&
                    route.is_valid_addition_for_capacity(input,
                                                         current_job.pickup,
                                                         current_job.delivery,
                                                         0);
    if (is_pickup) {
      std::vector<Index> p_d({job_rank, static_cast<Index>(job_rank + 1)});
      is_valid = is_valid && route.is_valid_addition_for_tw(input,
                                                            input.zero_amount(),
                                                            p_d.begin(),
                                                            p_d.end(),
                                                            0,
                                                            0);
    } else {
      assert(current_job.type == JOB_TYPE::SINGLE);
      is_valid = is_valid && route.is_valid_addition_for_tw(input, job_rank, 0);
    }

    if (is_valid) {
      init_ok = true;
      best_job_rank = job_rank;

      switch (init) {
        using enum INIT;
      case NONE:
        assert(false);
        break;
      case HIGHER_AMOUNT:
        if (higher_amount < current_job.pickup) {
          higher_amount = current_job.pickup;
        }
        if (higher_amount < current_job.delivery) {
          higher_amount = current_job.delivery;
        }
        break;
      case EARLIEST_DEADLINE:
        earliest_deadline = is_pickup ? input.jobs[job_rank + 1].tws.back().end
                                      : current_job.tws.back().end;
        break;
      case FURTHEST:
        furthest_cost = evals[job_rank][v_rank].cost;
        break;
      case NEAREST:
        nearest_cost = evals[job_rank][v_rank].cost;
        break;
      }
    }
  }

  if (init_ok) {
    if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
      route.add(input, best_job_rank, 0);
      unassigned.erase(best_job_rank);
    }
    if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
      std::vector<Index> p_d(
        {best_job_rank, static_cast<Index>(best_job_rank + 1)});
      route.replace(input, input.zero_amount(), p_d.begin(), p_d.end(), 0, 0);
      unassigned.erase(best_job_rank);
      unassigned.erase(best_job_rank + 1);
    }
  }
}

template <class Route> struct UnassignedCosts {
  const Vehicle& vehicle;
  Cost max_edge_cost;
  std::vector<Cost> min_route_to_unassigned;
  std::vector<Cost> min_unassigned_to_route;

  UnassignedCosts(const Input& input,
                  Route& route,
                  const std::set<Index>& unassigned)
    : vehicle(input.vehicles[route.v_rank]),
      max_edge_cost(utils::max_edge_eval(input, vehicle, route.route).cost),
      min_route_to_unassigned(input.jobs.size(),
                              std::numeric_limits<Cost>::max()),
      min_unassigned_to_route(input.jobs.size(),
                              std::numeric_limits<Cost>::max()) {
    for (const auto job_rank : unassigned) {
      const auto& unassigned_job = input.jobs[job_rank];
      const auto unassigned_job_index = unassigned_job.index();

      // The purpose here is to generate insertion lower bounds so we
      // only account for service times (no setup) which are
      // independent of insertion rank.
      const auto added_service = unassigned_job.services[vehicle.type];
      const auto service_cost = vehicle.task_eval(added_service).cost;

      if (vehicle.has_start()) {
        const auto start_to_job =
          vehicle.eval(vehicle.start.value().index(), unassigned_job_index)
            .cost;
        min_route_to_unassigned[job_rank] = start_to_job + service_cost;
      }

      if (vehicle.has_end()) {
        const auto job_to_end =
          vehicle.eval(unassigned_job_index, vehicle.end.value().index()).cost;
        min_unassigned_to_route[job_rank] = job_to_end + service_cost;
      }

      for (const auto j : route.route) {
        const auto job_index = input.jobs[j].index();

        const auto job_to_unassigned =
          vehicle.eval(job_index, unassigned_job_index).cost + service_cost;
        min_route_to_unassigned[job_rank] =
          std::min(min_route_to_unassigned[job_rank], job_to_unassigned);

        const auto unassigned_to_job =
          vehicle.eval(unassigned_job_index, job_index).cost + service_cost;
        min_unassigned_to_route[job_rank] =
          std::min(min_unassigned_to_route[job_rank], unassigned_to_job);
      }
    }
  }

  double get_insertion_lower_bound(Index j) {
    return static_cast<double>(min_route_to_unassigned[j] +
                               min_unassigned_to_route[j] - max_edge_cost);
  }

  double get_pd_insertion_lower_bound(const Input& input, Index p) {
    assert(input.jobs[p].type == JOB_TYPE::PICKUP);

    // Situation where pickup and delivery are not inserted in a row.
    const auto apart_insertion = static_cast<double>(
      min_route_to_unassigned[p] + min_unassigned_to_route[p] +
      min_route_to_unassigned[p + 1] + min_unassigned_to_route[p + 1] -
      2 * max_edge_cost);

    // Situation where delivery is inserted next to the pickup.
    const auto next_insertion = static_cast<double>(
      min_route_to_unassigned[p] + min_unassigned_to_route[p + 1] +
      vehicle.eval(input.jobs[p].index(), input.jobs[p + 1].index()).cost -
      max_edge_cost);

    return std::min(apart_insertion, next_insertion);
  }

  void update_max_edge(const Input& input, Route& route) {
    max_edge_cost = utils::max_edge_eval(input, vehicle, route.route).cost;
  }

  void update_min_costs(const Input& input,
                        const std::set<Index>& unassigned,
                        Index inserted_index) {
    for (const auto j : unassigned) {
      const auto& unassigned_job = input.jobs[j];
      const auto unassigned_job_index = unassigned_job.index();

      const auto added_service = unassigned_job.services[vehicle.type];
      const auto service_cost = vehicle.task_eval(added_service).cost;

      const auto to_unassigned =
        vehicle.eval(inserted_index, unassigned_job_index).cost + service_cost;
      min_route_to_unassigned[j] =
        std::min(min_route_to_unassigned[j], to_unassigned);

      const auto from_unassigned =
        vehicle.eval(unassigned_job_index, inserted_index).cost + service_cost;
      min_unassigned_to_route[j] =
        std::min(min_unassigned_to_route[j], from_unassigned);
    }
  }
};

template <class Route>
inline Eval fill_route(const Input& input,
                       Route& route,
                       const std::vector<Route>& routes,
                       const std::vector<Index>& remaining_ranks,
                       std::set<Index>& unassigned,
                       const std::vector<Cost>& regrets,
                       double lambda) {
  const auto v_rank = route.v_rank;
  const auto& vehicle = input.vehicles[v_rank];

  const auto& opts = input.get_soft_skills_options();
  const bool distance_data_available = input.has_distance_data();

  std::vector<std::optional<SoftJobReference>> soft_references;
  if (input.has_soft_skills()) {
    soft_references = build_soft_references(input,
                                           routes,
                                           unassigned,
                                           remaining_ranks,
                                           opts,
                                           distance_data_available);
  }

  const bool init_route_is_empty = route.empty();
  Eval route_eval = utils::route_eval_for_vehicle(input, v_rank, route.route);

  // Store bounds to be able to cut out some loops.
  UnassignedCosts unassigned_costs(input, route, unassigned);

  bool keep_going = true;
  while (keep_going) {
    keep_going = false;
    double best_cost = std::numeric_limits<double>::max();
    bool best_pref = false;
    Index best_job_rank = 0;
    Index best_r = 0;
    Index best_pickup_r = 0;
    Index best_delivery_r = 0;
    Amount best_modified_delivery = input.zero_amount();
    Eval best_eval;

    for (const auto job_rank : unassigned) {
      if (!input.vehicle_ok_with_job(v_rank, job_rank)) {
        continue;
      }

      const auto& current_job = input.jobs[job_rank];

      if (current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }

      // Preference-aware ranking (Task 15): on soft-skills inputs,
      // insertions that gain a preference point rank above
      // cost-equivalent non-matching ones (lexicographic order:
      // existing priority semantics, then preference, then Eval).
      bool boost_candidates = false;
      if (input.has_soft_skills() && !current_job.soft_skills.empty()) {
        const auto& reference = *soft_references[job_rank];
        const auto matches =
          soft_skill_match(current_job.soft_skills,
                           vehicle.soft_skills,
                           effective_match_mode(current_job, opts));
        if (!matches && reference.defer_to_match &&
            reference.defer_target != v_rank) {
          // A remaining vehicle matches this job's soft-skills and can
          // insert it within threshold: the current vehicle defers to
          // that vehicle.
          continue;
        }
        boost_candidates = matches;
      }

      if (current_job.type == JOB_TYPE::SINGLE &&
          route.size() + 1 <= vehicle.max_tasks) {

        if ((!boost_candidates || best_pref) &&
            best_cost <
              unassigned_costs.get_insertion_lower_bound(job_rank) -
                lambda * static_cast<double>(regrets[job_rank])) {
          // Bypass going through whole route if we're sure insertion
          // cost is not good enough.
          continue;
        }

        for (Index r = 0; r <= route.size(); ++r) {
          const auto current_eval =
            utils::addition_eval(input, job_rank, vehicle, route.route, r);

          const double current_cost =
            static_cast<double>(current_eval.cost) -
            lambda * static_cast<double>(regrets[job_rank]);

          if ((current_cost < best_cost ||
               (boost_candidates && !best_pref)) &&
              (vehicle.ok_for_range_bounds(route_eval + current_eval)) &&
              route.is_valid_addition_for_capacity(input,
                                                   current_job.pickup,
                                                   current_job.delivery,
                                                   r) &&
              route.is_valid_addition_for_tw(input, job_rank, r)) {
            const bool current_pref =
              boost_candidates &&
              soft_pref_gain(current_eval,
                             *soft_references[job_rank],
                             opts,
                             distance_data_available);

            if ((current_pref && !best_pref) ||
                (current_pref == best_pref && current_cost < best_cost)) {
              best_cost = current_cost;
              best_pref = current_pref;
              best_job_rank = job_rank;
              best_r = r;
              best_eval = current_eval;
            }
          }
        }
      }

      if (current_job.type == JOB_TYPE::PICKUP &&
          route.size() + 2 <= vehicle.max_tasks) {

        if ((!boost_candidates || best_pref) &&
            best_cost <
              unassigned_costs.get_pd_insertion_lower_bound(input, job_rank) -
                lambda * static_cast<double>(regrets[job_rank])) {
          // Bypass going through whole route if we're sure insertion
          // cost is not good enough.
          continue;
        }

        // Pre-compute cost of addition for matching delivery.
        std::vector<Eval> d_adds(route.route.size() + 1);
        std::vector<unsigned char> valid_delivery_insertions(
          route.route.size() + 1);

        for (unsigned d_rank = 0; d_rank <= route.route.size(); ++d_rank) {
          d_adds[d_rank] = utils::addition_eval(input,
                                                job_rank + 1,
                                                vehicle,
                                                route.route,
                                                d_rank);
          valid_delivery_insertions[d_rank] =
            route.is_valid_addition_for_tw_without_max_load(input,
                                                            job_rank + 1,
                                                            d_rank);
        }

        for (Index pickup_r = 0; pickup_r <= route.size(); ++pickup_r) {
          const auto p_add = utils::addition_eval(input,
                                                  job_rank,
                                                  vehicle,
                                                  route.route,
                                                  pickup_r);

          if (!route.is_valid_addition_for_load(input,
                                                current_job.pickup,
                                                pickup_r) ||
              !route.is_valid_addition_for_tw_without_max_load(input,
                                                               job_rank,
                                                               pickup_r)) {
            continue;
          }

          // Build replacement sequence for current insertion.
          std::vector<Index> modified_with_pd;
          modified_with_pd.reserve(route.size() - pickup_r + 2);
          modified_with_pd.push_back(job_rank);

          Amount modified_delivery = input.zero_amount();

          for (Index delivery_r = pickup_r; delivery_r <= route.size();
               ++delivery_r) {
            // Update state variables along the way before potential
            // early abort.
            if (pickup_r < delivery_r) {
              modified_with_pd.push_back(route.route[delivery_r - 1]);
              const auto& new_modified_job =
                input.jobs[route.route[delivery_r - 1]];
              if (new_modified_job.type == JOB_TYPE::SINGLE) {
                modified_delivery += new_modified_job.delivery;
              }
            }

            if (!static_cast<bool>(valid_delivery_insertions[delivery_r])) {
              continue;
            }

            Eval current_eval;
            if (pickup_r == delivery_r) {
              current_eval = utils::addition_eval(input,
                                                  job_rank,
                                                  vehicle,
                                                  route.route,
                                                  pickup_r,
                                                  pickup_r + 1);
            } else {
              current_eval = p_add + d_adds[delivery_r];
            }

            const double current_cost =
              current_eval.cost -
              lambda * static_cast<double>(regrets[job_rank]);

            if (current_cost < best_cost ||
                (boost_candidates && !best_pref)) {
              modified_with_pd.push_back(job_rank + 1);

              // Update best cost depending on validity.
              const bool valid =
                (vehicle.ok_for_range_bounds(route_eval + current_eval)) &&
                route
                  .is_valid_addition_for_capacity_inclusion(input,
                                                            modified_delivery,
                                                            modified_with_pd
                                                              .begin(),
                                                            modified_with_pd
                                                              .end(),
                                                            pickup_r,
                                                            delivery_r) &&
                route.is_valid_addition_for_tw(input,
                                               modified_delivery,
                                               modified_with_pd.begin(),
                                               modified_with_pd.end(),
                                               pickup_r,
                                               delivery_r);

              modified_with_pd.pop_back();

              if (valid) {
                const bool current_pref =
                  boost_candidates &&
                  soft_pref_gain(current_eval,
                                 *soft_references[job_rank],
                                 opts,
                                 distance_data_available);

                if ((current_pref && !best_pref) ||
                    (current_pref == best_pref && current_cost < best_cost)) {
                  best_cost = current_cost;
                  best_pref = current_pref;
                  best_job_rank = job_rank;
                  best_pickup_r = pickup_r;
                  best_delivery_r = delivery_r;
                  best_modified_delivery = modified_delivery;
                  best_eval = current_eval;
                }
              }
            }
          }
        }
      }
    }

    if (best_cost < std::numeric_limits<double>::max()) {
      const auto& best_job = input.jobs[best_job_rank];
      if (best_job.type == JOB_TYPE::SINGLE) {
        route.add(input, best_job_rank, best_r);
        unassigned.erase(best_job_rank);
        keep_going = true;

        unassigned_costs.update_max_edge(input, route);
        unassigned_costs.update_min_costs(input, unassigned, best_job.index());
      }
      if (best_job.type == JOB_TYPE::PICKUP) {
        std::vector<Index> modified_with_pd;
        modified_with_pd.reserve(best_delivery_r - best_pickup_r + 2);
        modified_with_pd.push_back(best_job_rank);

        std::copy(route.route.begin() + best_pickup_r,
                  route.route.begin() + best_delivery_r,
                  std::back_inserter(modified_with_pd));
        modified_with_pd.push_back(best_job_rank + 1);

        route.replace(input,
                      best_modified_delivery,
                      modified_with_pd.begin(),
                      modified_with_pd.end(),
                      best_pickup_r,
                      best_delivery_r);
        unassigned.erase(best_job_rank);
        unassigned.erase(best_job_rank + 1);
        keep_going = true;

        unassigned_costs.update_max_edge(input, route);
        unassigned_costs.update_min_costs(input, unassigned, best_job.index());
        unassigned_costs
          .update_min_costs(input,
                            unassigned,
                            input.jobs[best_job_rank + 1].index());
      }

      route_eval += best_eval;
    }
  }

  if (init_route_is_empty && !route.empty()) {
    // Account for fixed cost if we actually filled an empty route.
    route_eval.cost += vehicle.fixed_cost();
  }

  return route_eval;
}

template <class Route>
Eval basic(const Input& input,
           std::vector<Route>& routes,
           std::set<Index> unassigned,
           std::vector<Index> vehicles_ranks,
           INIT init,
           double lambda,
           SORT sort) {
  // Ordering is based on vehicles description only so do not account
  // for initial routes if any.
  const auto nb_vehicles = vehicles_ranks.size();

  switch (sort) {
  case SORT::AVAILABILITY: {
    // Sort vehicles by decreasing "availability".
    std::ranges::stable_sort(vehicles_ranks,
                             [&](const auto lhs, const auto rhs) {
                               return input.vehicles[lhs] < input.vehicles[rhs];
                             });
    break;
  }
  case SORT::COST:
    // Sort vehicles by increasing fixed cost, then same as above.
    std::ranges::stable_sort(vehicles_ranks,
                             [&](const auto lhs, const auto rhs) {
                               const auto& v_lhs = input.vehicles[lhs];
                               const auto& v_rhs = input.vehicles[rhs];
                               return v_lhs.costs < v_rhs.costs ||
                                      (v_lhs.costs == v_rhs.costs &&
                                       input.vehicles[lhs] <
                                         input.vehicles[rhs]);
                             });
    break;
  }

  const auto& evals = input.jobs_vehicles_evals();

  // regrets[v][j] holds the min cost for reaching job j in an empty
  // route across all remaining vehicles **after** vehicle at rank v
  // in vehicles_ranks. Regrets are only computed for available
  // vehicles and unassigned jobs, but are based on empty routes
  // evaluations so do not account for initial routes if any.
  std::vector<std::vector<Cost>> regrets(nb_vehicles,
                                         std::vector<Cost>(input.jobs.size()));

  // Use own cost for last vehicle regret values.
  for (const auto j : unassigned) {
    regrets.back()[j] = evals[j][vehicles_ranks.back()].cost;
  }

  for (Index rev_v = 0; rev_v < nb_vehicles - 1; ++rev_v) {
    // Going trough vehicles backward from second to last.
    const auto v = nb_vehicles - 2 - rev_v;

    bool all_compatible_jobs_later_undoable = true;
    for (const auto j : unassigned) {
      regrets[v][j] =
        std::min(regrets[v + 1][j], (evals[j][vehicles_ranks[v + 1]]).cost);
      if (input.vehicle_ok_with_job(vehicles_ranks[v], j) &&
          regrets[v][j] < input.get_cost_upper_bound()) {
        all_compatible_jobs_later_undoable = false;
      }

      // Task 15 note: preference-aware handling happens at insertion
      // ranking time in seed_route/fill_route through on-demand soft
      // job references, so empty-route regrets stay cost-based.
    }

    if (all_compatible_jobs_later_undoable) {
      // We don't want to use all regrets equal to the cost upper
      // bound in this situation: it would defeat the purpose of using
      // regrets in the first place as all lambda values would yield
      // the same choices. Using the same approach as with last
      // vehicle.
      for (const auto j : unassigned) {
        regrets[v][j] = evals[j][vehicles_ranks[v]].cost;
      }
    }
  }

  Eval sol_eval;

  for (Index v = 0; v < nb_vehicles && !unassigned.empty(); ++v) {
    auto v_rank = vehicles_ranks[v];
    auto& current_r = routes[v_rank];

    const std::vector<Index> remaining_ranks(vehicles_ranks.begin() + v + 1,
                                             vehicles_ranks.end());

    if (current_r.empty() && init != INIT::NONE) {
      // Trivial lambda for no additional job validity constraint.
      constexpr auto job_not_ok = [](const Index) { return false; };

      seed_route(input,
                 current_r,
                 routes,
                 remaining_ranks,
                 init,
                 evals,
                 unassigned,
                 job_not_ok);
    }

    const auto current_eval = fill_route(input,
                                         current_r,
                                         routes,
                                         remaining_ranks,
                                         unassigned,
                                         regrets[v],
                                         lambda);
    sol_eval += current_eval;
  }

  return sol_eval;
}

template <class Route>
Eval dynamic_vehicle_choice(const Input& input,
                            std::vector<Route>& routes,
                            std::set<Index> unassigned,
                            std::vector<Index> vehicles_ranks,
                            INIT init,
                            double lambda,
                            SORT sort) {
  const auto& evals = input.jobs_vehicles_evals();

  Eval sol_eval;

  while (!vehicles_ranks.empty() && !unassigned.empty()) {
    // For any unassigned job at j, jobs_min_costs[j]
    // (resp. jobs_second_min_costs[j]) holds the min cost
    // (resp. second min cost) of picking the job in an empty route
    // for any remaining vehicle. Evaluation are based on empty routes
    // so do not account for initial routes if any.
    std::vector<Cost> jobs_min_costs(input.jobs.size(),
                                     input.get_cost_upper_bound());
    std::vector<Cost> jobs_second_min_costs(input.jobs.size(),
                                            input.get_cost_upper_bound());
    for (const auto j : unassigned) {
      for (const auto v : vehicles_ranks) {
        if (evals[j][v].cost <= jobs_min_costs[j]) {
          jobs_second_min_costs[j] = jobs_min_costs[j];
          jobs_min_costs[j] = evals[j][v].cost;
        } else {
          if (evals[j][v].cost < jobs_second_min_costs[j]) {
            jobs_second_min_costs[j] = evals[j][v].cost;
          }
        }
      }
    }

    // Pick vehicle that has the biggest number of compatible
    // unassigned jobs closest to him than to any other different
    // vehicle still available.
    std::vector<unsigned> closest_jobs_count(input.vehicles.size(), 0);
    for (const auto j : unassigned) {
      for (const auto v : vehicles_ranks) {
        if (evals[j][v].cost == jobs_min_costs[j]) {
          ++closest_jobs_count[v];
        }
      }
    }

    Index v_rank;

    if (sort == SORT::AVAILABILITY) {
      const auto chosen_vehicle =
        std::ranges::min_element(vehicles_ranks,
                                 [&](const auto lhs, const auto rhs) {
                                   return closest_jobs_count[lhs] >
                                            closest_jobs_count[rhs] ||
                                          (closest_jobs_count[lhs] ==
                                             closest_jobs_count[rhs] &&
                                           input.vehicles[lhs] <
                                             input.vehicles[rhs]);
                                 });
      v_rank = *chosen_vehicle;
      vehicles_ranks.erase(chosen_vehicle);
    } else {
      assert(sort == SORT::COST);

      const auto chosen_vehicle =
        std::ranges::min_element(vehicles_ranks,
                                 [&](const auto lhs, const auto rhs) {
                                   const auto& v_lhs = input.vehicles[lhs];
                                   const auto& v_rhs = input.vehicles[rhs];
                                   return closest_jobs_count[lhs] >
                                            closest_jobs_count[rhs] ||
                                          (closest_jobs_count[lhs] ==
                                             closest_jobs_count[rhs] &&
                                           (v_lhs.costs < v_rhs.costs ||
                                            (v_lhs.costs == v_rhs.costs &&
                                             v_lhs < v_rhs)));
                                 });
      v_rank = *chosen_vehicle;
      vehicles_ranks.erase(chosen_vehicle);
    }

    // Once current vehicle is decided, then for any unassigned job at
    // j, regrets[j] holds the min cost of picking the job in an empty
    // route for other remaining vehicles. Regrets are only computed
    // for available vehicles and unassigned jobs, but are based on
    // empty routes evaluations so do not account for initial routes
    // if any.
    std::vector<Cost> regrets(input.jobs.size(), input.get_cost_upper_bound());

    bool all_compatible_jobs_later_undoable = true;
    for (const auto j : unassigned) {
      if (jobs_min_costs[j] < evals[j][v_rank].cost) {
        regrets[j] = jobs_min_costs[j];
      } else {
        regrets[j] = jobs_second_min_costs[j];
      }

      if (input.vehicle_ok_with_job(v_rank, j) &&
          regrets[j] < input.get_cost_upper_bound()) {
        all_compatible_jobs_later_undoable = false;
      }

      // Task 15 note: preference-aware handling happens at insertion
      // ranking time in seed_route/fill_route through on-demand soft
      // job references, so empty-route regrets stay cost-based.
    }

    if (all_compatible_jobs_later_undoable) {
      // Same approach as for basic heuristic.
      for (const auto j : unassigned) {
        regrets[j] = evals[j][v_rank].cost;
      }
    }

    auto& current_r = routes[v_rank];

    if (current_r.empty() && init != INIT::NONE) {
      auto job_not_ok =
        [&jobs_min_costs, &evals, v_rank](const Index job_rank) {
          // One of the remaining vehicles is closest to that job.
          return jobs_min_costs[job_rank] < evals[job_rank][v_rank].cost;
        };

      seed_route(input,
                 current_r,
                 routes,
                 vehicles_ranks,
                 init,
                 evals,
                 unassigned,
                 job_not_ok);
    }

    const auto current_eval = fill_route(input,
                                         current_r,
                                         routes,
                                         vehicles_ranks,
                                         unassigned,
                                         regrets,
                                         lambda);
    sol_eval += current_eval;
  }

  return sol_eval;
}

template <class Route>
void set_route(const Input& input,
               Route& route,
               std::unordered_set<Index>& assigned) {
  assert(route.empty());
  const auto& vehicle = input.vehicles[route.v_rank];

  // Startup load is the sum of deliveries for (single) jobs.
  Amount single_jobs_deliveries(input.zero_amount());
  for (const auto& step : vehicle.steps) {
    if (step.type == STEP_TYPE::JOB) {
      assert(step.job_type.has_value());

      if (step.job_type.value() == JOB_TYPE::SINGLE) {
        single_jobs_deliveries += input.jobs[step.rank].delivery;
      }
    }
  }
  if (!(single_jobs_deliveries <= vehicle.capacity)) {
    throw InputException(
      std::format("Route over capacity for vehicle {}.", vehicle.id));
  }

  // Track load and travel time during the route for validity.
  Amount current_load = single_jobs_deliveries;
  Eval eval_sum;
  std::optional<Index> previous_index;
  if (vehicle.has_start()) {
    previous_index = vehicle.start.value().index();
  }

  std::vector<Index> job_ranks;
  job_ranks.reserve(vehicle.steps.size());
  std::unordered_set<Index> expected_delivery_ranks;
  for (const auto& step : vehicle.steps) {
    if (step.type != STEP_TYPE::JOB) {
      continue;
    }

    const auto job_rank = step.rank;
    const auto& job = input.jobs[job_rank];
    job_ranks.push_back(job_rank);

    assert(!assigned.contains(job_rank));
    assigned.insert(job_rank);

    if (!input.vehicle_ok_with_job(route.v_rank, job_rank)) {
      throw InputException(
        std::format("Missing skill or step out of reach for vehicle {} and "
                    "job {}.",
                    vehicle.id,
                    job.id));
    }

    // Update current travel time.
    if (previous_index.has_value()) {
      eval_sum += vehicle.eval(previous_index.value(), job.index());
    }
    previous_index = job.index();

    // Handle load.
    assert(step.job_type.has_value());
    switch (step.job_type.value()) {
    case JOB_TYPE::SINGLE: {
      current_load += job.pickup;
      current_load -= job.delivery;
      break;
    }
    case JOB_TYPE::PICKUP: {
      expected_delivery_ranks.insert(job_rank + 1);

      current_load += job.pickup;
      break;
    }
    case JOB_TYPE::DELIVERY: {
      auto search = expected_delivery_ranks.find(job_rank);
      if (search == expected_delivery_ranks.end()) {
        throw InputException(
          std::format("Invalid shipment in route for vehicle {}.", vehicle.id));
      }
      expected_delivery_ranks.erase(search);

      current_load -= job.delivery;
      break;
    }
    default:
      assert(false);
    }

    // Check validity after this step wrt capacity.
    if (!(current_load <= vehicle.capacity)) {
      throw InputException(
        std::format("Route over capacity for vehicle {}.", vehicle.id));
    }
  }

  if (vehicle.has_end() && !job_ranks.empty()) {
    // Update with last route leg.
    assert(previous_index.has_value());
    eval_sum +=
      vehicle.eval(previous_index.value(), vehicle.end.value().index());
  }
  if (!vehicle.ok_for_travel_time(eval_sum.duration)) {
    throw InputException(
      std::format("Route over max_travel_time for vehicle {}.", vehicle.id));
  }
  if (!vehicle.ok_for_distance(eval_sum.distance)) {
    throw InputException(
      std::format("Route over max_distance for vehicle {}.", vehicle.id));
  }

  if (vehicle.max_tasks < job_ranks.size()) {
    throw InputException(
      std::format("Too many tasks for vehicle {}.", vehicle.id));
  }

  if (!expected_delivery_ranks.empty()) {
    throw InputException(
      std::format("Invalid shipment in route for vehicle {}.", vehicle.id));
  }

  // Now route is OK with regard to capacity, max_travel_time,
  // max_tasks, precedence and skills constraints.
  if (!job_ranks.empty()) {
    if (!route.is_valid_addition_for_tw(input,
                                        single_jobs_deliveries,
                                        job_ranks.begin(),
                                        job_ranks.end(),
                                        0,
                                        0)) {
      throw InputException(
        std::format("Infeasible route for vehicle {}.", vehicle.id));
    }

    route.replace(input,
                  single_jobs_deliveries,
                  job_ranks.begin(),
                  job_ranks.end(),
                  0,
                  0);
  }
}

template <class Route>
void set_initial_routes(const Input& input,
                        std::vector<Route>& routes,
                        std::unordered_set<Index>& assigned) {
  std::ranges::for_each(routes,
                        [&](auto& r) { set_route(input, r, assigned); });
}

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

template Eval basic(const Input& input,
                    RawSolution& routes,
                    std::set<Index> unassigned,
                    std::vector<Index> vehicles_ranks,
                    INIT init,
                    double lambda,
                    SORT sort);

template Eval dynamic_vehicle_choice(const Input& input,
                                     RawSolution& routes,
                                     std::set<Index> unassigned,
                                     std::vector<Index> vehicles_ranks,
                                     INIT init,
                                     double lambda,
                                     SORT sort);

template void set_initial_routes(const Input& input,
                                 RawSolution& routes,
                                 std::unordered_set<Index>& assigned);

template Eval basic(const Input& input,
                    TWSolution& routes,
                    std::set<Index> unassigned,
                    std::vector<Index> vehicles_ranks,
                    INIT init,
                    double lambda,
                    SORT sort);

template Eval dynamic_vehicle_choice(const Input& input,
                                     TWSolution& routes,
                                     std::set<Index> unassigned,
                                     std::vector<Index> vehicles_ranks,
                                     INIT init,
                                     double lambda,
                                     SORT sort);

template void set_initial_routes(const Input& input,
                                 TWSolution& routes,
                                 std::unordered_set<Index>& assigned);

} // namespace vroom::heuristics
