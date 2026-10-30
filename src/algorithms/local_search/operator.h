#ifndef OPERATOR_H
#define OPERATOR_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/preference_baselines.h"
#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"

namespace vroom::ls {

// Move acceptance key: net soft-skills preference delta first, then
// the regular Eval gain. Lexicographic comparison of keys implements
// the (priority >) preference > Eval hierarchy (Q4/Q5): a
// preference-gaining move wins over any non-preference move, a
// preference-losing move never wins on Eval alone. When all deltas
// are zero the comparison reduces to the upstream Eval-only
// behavior (zero change invariant).
struct GainKey {
  int preference_delta{0};
  Eval gain;

  friend bool operator<(const GainKey& lhs, const GainKey& rhs) {
    return std::tie(lhs.preference_delta, lhs.gain)
           < std::tie(rhs.preference_delta, rhs.gain);
  }
};

// Type-erased view on the current solution routes for on-demand
// preference baseline scans. The LocalSearch instantiation owns a
// concrete SolutionPreferenceScan<Route> (RawRoute for CVRP, TWRoute
// for VRPTW) and hands it to the preference-aware operators, so
// compute_gain() implementations never need to know the concrete
// route type themselves.
class PreferenceScanContext {
public:
  virtual ~PreferenceScanContext() = default;

  virtual PreferenceBaselines baselines(const Input& input,
                                        const utils::SolutionState& sol_state,
                                        Index job_rank) const = 0;

  // Best feasible insertion eval of the task at job_rank (JOB_TYPE::
  // SINGLE or JOB_TYPE::PICKUP) into the frozen route of vehicle
  // v_rank; NO_EVAL when no feasible insertion exists.
  virtual Eval best_insertion(const Input& input,
                              const utils::SolutionState& sol_state,
                              Index job_rank,
                              Index v_rank) const = 0;
};

template <class Route>
class SolutionPreferenceScan : public PreferenceScanContext {
private:
  const std::vector<Route>& _sol;

public:
  explicit SolutionPreferenceScan(const std::vector<Route>& sol) : _sol(sol) {
  }

  PreferenceBaselines baselines(const Input& input,
                                const utils::SolutionState& sol_state,
                                Index job_rank) const override {
    return compute_preference_baselines(input, sol_state, _sol, job_rank);
  }

  Eval best_insertion(const Input& input,
                      const utils::SolutionState& sol_state,
                      Index job_rank,
                      Index v_rank) const override {
    assert(input.jobs[job_rank].type == JOB_TYPE::SINGLE
           || input.jobs[job_rank].type == JOB_TYPE::PICKUP);
    return (input.jobs[job_rank].type == JOB_TYPE::SINGLE)
             ? compute_best_insertion_single(input,
                                             sol_state,
                                             job_rank,
                                             v_rank,
                                             _sol[v_rank])
                 .eval
             : compute_best_insertion_pd(input,
                                         sol_state,
                                         job_rank,
                                         v_rank,
                                         _sol[v_rank],
                                         NO_EVAL)
                 .eval;
  }
};

class Operator {
protected:
  const OperatorName _name;
  const Input& _input;
  const utils::SolutionState& _sol_state;

  // Source of move for this operator.
  RawRoute& source;
  std::vector<Index>& s_route;
  const Index s_vehicle;
  const Index s_rank;
  // Target of move for this operator.
  RawRoute& target;
  std::vector<Index>& t_route;
  const Index t_vehicle;
  const Index t_rank;

  bool gain_computed{false};
  Eval s_gain;
  Eval t_gain;
  Eval stored_gain;

  // Net preference delta of this move over the tasks it relocates
  // (number of tasks gaining a threshold-qualified match minus the
  // number losing their current match). Stays 0 unless soft-skills
  // preference logic is active (Tasks 17-20).
  int preference_delta{0};

  // Owned by the LocalSearch instantiation; null when no preference
  // context was wired for this operator.
  const PreferenceScanContext* _preference_scan{nullptr};

  virtual void compute_gain() = 0;

  // True iff the input actually contains soft-skills. Every preference
  // check added to derived operators (Tasks 17-24) must be gated on
  // this so that soft-skills-free inputs keep the pure Eval-based
  // gain behavior (zero change fast path).
  bool has_soft_skills() const {
    return _input.has_soft_skills();
  }

  // True iff compute_gain() may run preference logic: requires both
  // soft-skills in input and a solution route context for baseline
  // scans.
  bool has_preference_context() const {
    return _preference_scan != nullptr;
  }

  PreferenceBaselines preference_baselines(Index job_rank) const {
    return _preference_scan->baselines(_input, _sol_state, job_rank);
  }

  // Best feasible insertion eval of a task (SINGLE or PICKUP) into
  // the frozen route of the given vehicle.
  Eval best_preference_insertion(Index job_rank, Index v_rank) const {
    return _preference_scan->best_insertion(_input, _sol_state, job_rank,
                                            v_rank);
  }

  // Raw (not threshold-qualified) soft-skill match of the task at
  // job_rank with the given vehicle.
  bool task_pre_match(Index job_rank, Index v_rank) const {
    return task_matches_vehicle(_input, job_rank, v_rank);
  }

  // Post-move preference status of the task at job_rank placed on
  // vehicle to_vehicle: the vehicle must match the task's soft
  // skills and the given placement eval must stay within threshold
  // of the frozen non-match baseline (Q3 auto-pass when no
  // feasible non-matching alternative exists). A task currently on a
  // matching vehicle counts as matched for the "pre" side regardless
  // of threshold: status-quo assignments are not decisions (see
  // fixture 03).
  template <class EvalFn>
  bool task_post_match(Index job_rank,
                       Index to_vehicle,
                       EvalFn&& placement_eval) const {
    if (!task_matches_vehicle(_input, job_rank, to_vehicle)) {
      return false;
    }
    const auto baselines = preference_baselines(job_rank);
    if (!baselines.nonmatch_feasible) {
      return true;
    }
    return ls::task_post_match(_input,
                               job_rank,
                               to_vehicle,
                               placement_eval(),
                               baselines);
  }

  // Preference delta of moving the task at job_rank from
  // from_vehicle to to_vehicle, landing in the frozen content of
  // route destination at destination_rank. Placement eval: the
  // actual single-job addition eval at the destination rank, or the
  // best feasible pickup-and-delivery insertion for a shipment.
  // DELIVERY endpoints contribute nothing: their preference is the
  // PICKUP's (shared soft skills), counted once per shipment.
  template <class Route>
  int task_move_delta(Index job_rank,
                      Index from_vehicle,
                      Index to_vehicle,
                      const Route& destination,
                      Index destination_rank) const {
    const auto& job = _input.jobs[job_rank];
    if (job.type == JOB_TYPE::DELIVERY) {
      return 0;
    }
    const int post = task_post_match(job_rank, to_vehicle, [&]() {
      return (job.type == JOB_TYPE::SINGLE)
               ? utils::addition_eval(_input,
                                      job_rank,
                                      _input.vehicles[to_vehicle],
                                      destination.route,
                                      destination_rank)
               : compute_best_insertion_pd(_input,
                                           _sol_state,
                                           job_rank,
                                           to_vehicle,
                                           destination,
                                           NO_EVAL)
                     .eval;
    })
                ? 1
                : 0;
    return post - (task_pre_match(job_rank, from_vehicle) ? 1 : 0);
  }

  bool is_valid_for_source_range_bounds() const;

  bool is_valid_for_target_range_bounds() const;

  // Used for internal operators only.
  bool is_valid_for_range_bounds() const;

public:
  Operator(OperatorName name,
           const Input& input,
           const utils::SolutionState& sol_state,
           RawRoute& s_raw_route,
           Index s_vehicle,
           Index s_rank,
           RawRoute& t_raw_route,
           Index t_vehicle,
           Index t_rank)
    : _name(name),
      _input(input),
      _sol_state(sol_state),
      source(s_raw_route),
      s_route(s_raw_route.route),
      s_vehicle(s_vehicle),
      s_rank(s_rank),
      target(t_raw_route),
      t_route(t_raw_route.route),
      t_vehicle(t_vehicle),
      t_rank(t_rank) {
    if (!input.has_soft_skills()) {
      // Fast path: no soft-skills in input, so no preference state to
      // prepare and the operator keeps its exact upstream Eval-only
      // behavior. Tasks 17-24 add any preference-related construction
      // strictly after this guard.
      return;
    }

    // Placeholder for Tasks 17-24: operator preference state setup.
  }

  OperatorName get_name() const;

  // Wire the solution route context used for on-demand preference
  // baseline scans. Called by LocalSearch on the operators it
  // evaluates; operators never wired keep preference_delta at 0.
  void set_preference_context(const PreferenceScanContext& scan) {
    _preference_scan = &scan;
  }

  // Acceptance key for this move: preference delta first, then Eval
  // gain. Gain computation must run first since it is what computes
  // the preference delta on most operators.
  GainKey key() {
    this->gain();
    return GainKey{preference_delta, stored_gain};
  }

  virtual Eval gain();

  virtual bool is_valid() = 0;

  virtual void apply() = 0;

  virtual std::vector<Index> addition_candidates() const = 0;

  virtual std::vector<Index> update_candidates() const = 0;

  // Used to check if a move should be made invalid due to a change in
  // unassigned jobs.
  virtual std::vector<Index> required_unassigned() const;

  // Used to check if a move should be made invalid due to an indirect
  // change in another route.
  virtual bool invalidated_by(Index rank) const;

  virtual ~Operator() = default;
};

} // namespace vroom::ls

#endif
