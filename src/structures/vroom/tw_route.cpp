/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <optional>

#include "structures/vroom/tw_route.h"
#include "utils/helpers.h"

namespace vroom {

namespace {

Duration saturating_add(const Duration lhs, const Duration rhs) {
  return (std::numeric_limits<Duration>::max() - lhs < rhs)
           ? std::numeric_limits<Duration>::max()
           : lhs + rhs;
}

Duration saturating_sub(const Duration lhs, const Duration rhs) {
  return (lhs <= rhs) ? 0 : lhs - rhs;
}

// Setup only applies when the vehicle arrives from a different location;
// action time is service plus that conditional setup.
Duration action_time_for(const Job& j,
                         const Index v_type,
                         const Index previous_location) {
  return (j.index() == previous_location)
           ? j.services[v_type]
           : j.setups[v_type] + j.services[v_type];
}

// First time window whose end can accommodate time t, or tws.end().
std::vector<TimeWindow>::const_iterator
first_reachable_tw(const std::vector<TimeWindow>& tws, const Duration t) {
  return std::ranges::find_if(tws, [&](const auto& tw) { return t <= tw.end; });
}

// Apply one forward break step: from departure time t, place the break in
// its first reachable time window, absorbing any wait into the remaining
// travel of the current leg; record its start and advance t past its
// service. Forward twin of apply_break_latest below. Returns false when no
// window can take the break.
bool apply_break_earliest(const Break& b,
                          Duration& t,
                          Duration& remaining_travel,
                          Duration& break_start) {
  const auto b_tw = first_reachable_tw(b.tws, t);
  if (b_tw == b.tws.end()) {
    return false;
  }
  if (t < b_tw->start) {
    const auto margin = b_tw->start - t;
    remaining_travel =
      (margin < remaining_travel) ? remaining_travel - margin : 0;
    t = b_tw->start;
  }
  break_start = t;
  t = saturating_add(t, b.service);
  return true;
}

// Apply one backward break step: subtract its service, select the last
// compatible time window, clamp to its end and absorb that clamp in travel.
// The result only changes the supplied local values, never route state.
bool apply_break_latest(const Break& b,
                        Duration& current_latest,
                        Duration& remaining_travel) {
  if (b.service > current_latest) {
    return false;
  }
  current_latest -= b.service;

  const auto b_tw =
    std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
      return tw.start <= current_latest;
    });
  if (b_tw == b.tws.rend()) {
    return false;
  }
  if (b_tw->end < current_latest) {
    const auto margin = current_latest - b_tw->end;
    remaining_travel =
      (margin < remaining_travel) ? remaining_travel - margin : 0;
    current_latest = b_tw->end;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Cap-aware scheduler engine: shared forward-simulation helper.
//
// Runs an ASAP forward pass over job_seq[from_pos..N) where N = job_seq.size().
// For each job position pos, first processes brk_cnt[pos] breaks (vehicle-break
// ranks brk_fst[pos]..brk_fst[pos]+brk_cnt[pos]-1) then the job itself.
//
// Parameters entering from_pos's break slot:
//   cur_dep      : departure time of the preceding stop (job or vehicle start)
//   cur_loc      : predecessor job location (or
//   std::numeric_limits<Index>::max()
//                  when there is no predecessor with a known location)
//   rem_travel   : remaining leg travel on the current leg (from cur_loc to
//                  job_seq[from_pos]'s location), for break absorption
//
// Fills S[], A[], dep[], tw_idx[], loc[], break_S[].
// Returns false if any job or break cannot fit in its TWs (infeasible).
// ---------------------------------------------------------------------------
bool cap_asap_forward(const Input& input,
                      const Vehicle& v,
                      const Index v_type,
                      const Duration v_end,
                      const std::vector<Index>& job_seq,
                      const std::vector<unsigned>& brk_cnt,
                      const std::vector<unsigned>& brk_fst,
                      const std::size_t from_pos,
                      Duration cur_dep,
                      Index cur_loc,
                      Duration rem_travel,
                      std::vector<Duration>& S,
                      std::vector<Duration>& A,
                      std::vector<Duration>& dep,
                      std::vector<std::size_t>& tw_idx,
                      std::vector<Index>& loc,
                      std::vector<Duration>& break_S) {
  const std::size_t N = job_seq.size();
  for (std::size_t pos = from_pos; pos < N; ++pos) {
    // Process breaks before job at pos.
    for (unsigned b = 0; b < brk_cnt[pos]; ++b) {
      const unsigned brk_rank = brk_fst[pos] + b;
      if (!apply_break_earliest(v.breaks[brk_rank],
                                cur_dep,
                                rem_travel,
                                break_S[brk_rank])) {
        return false;
      }
    }

    // Process job at pos.
    const Index job_rank = job_seq[pos];
    const auto& j = input.jobs[job_rank];
    const Index job_loc = j.index();
    const Duration arrival = saturating_add(cur_dep, rem_travel);

    const auto j_tw_it = first_reachable_tw(j.tws, arrival);
    if (j_tw_it == j.tws.end()) {
      return false;
    }
    const std::size_t sel_tw =
      static_cast<std::size_t>(j_tw_it - j.tws.begin());
    S[pos] = std::max(arrival, j_tw_it->start);
    tw_idx[pos] = sel_tw;
    const Duration action = action_time_for(j, v_type, cur_loc);
    A[pos] = action;
    dep[pos] = saturating_add(S[pos], action);
    loc[pos] = job_loc;
    cur_dep = dep[pos];
    cur_loc = job_loc;

    if (pos + 1 < N) {
      rem_travel = v.duration(job_loc, input.jobs[job_seq[pos + 1]].index());
    }
  }

  // Process terminal breaks (after the last job, before the vehicle end),
  // mirroring fwd_update_earliest_from's handling of breaks_at_rank[N].
  // brk_cnt has size N+1; slot N holds the terminal break count.
  const Duration end_travel =
    v.has_end() ? v.duration(cur_loc, v.end.value().index()) : 0;
  Duration term_rem = end_travel;
  for (unsigned b = 0; b < brk_cnt[N]; ++b) {
    const unsigned brk_rank = brk_fst[N] + b;
    if (!apply_break_earliest(v.breaks[brk_rank],
                              cur_dep,
                              term_rem,
                              break_S[brk_rank])) {
      return false;
    }
  }
  // Check vehicle end feasibility (mirrors fwd_update_earliest_from's
  // assert(earliest_end <= v_end) and the current.earliest + next.travel >
  // next.latest guard in is_valid_addition_for_tw).
  if (v.has_end() && saturating_add(cur_dep, term_rem) > v_end) {
    return false;
  }
  return true;
}

// Scratch buffers for compute_cap_compliant_schedule (candidate overload),
// avoiding per-call allocations on a path hit millions of times per solve.
// Reuse contract: every buffer is cleared or assign()-initialized at the
// start of each call, no reference outlives a call (the function returns by
// value), and thread_local rules out cross-thread aliasing.
struct CapPairBuf {
  std::size_t p_pos;
  std::size_t d_pos;
  Duration cap;
};

thread_local std::vector<Index> tls_job_seq;
thread_local std::vector<unsigned> tls_brk_cnt;
thread_local std::vector<unsigned> tls_brk_fst;
thread_local std::vector<Duration> tls_S;
thread_local std::vector<Duration> tls_A;
thread_local std::vector<Duration> tls_dep;
thread_local std::vector<std::size_t> tls_tw_idx;
thread_local std::vector<Index> tls_loc;
thread_local std::vector<Duration> tls_break_S;
thread_local std::vector<Index> tls_pred_loc;
thread_local std::vector<CapPairBuf> tls_pairs;

} // namespace

TWRoute::TWRoute(const Input& input, Index v, unsigned amount_size)
  : RawRoute(input, v, amount_size),
    v_start(input.vehicles[v].tw.start),
    v_end(input.vehicles[v].tw.end),
    breaks_at_rank({static_cast<unsigned>(input.vehicles[v].breaks.size())}),
    breaks_counts({static_cast<unsigned>(input.vehicles[v].breaks.size())}),
    break_earliest(input.vehicles[v].breaks.size()),
    break_latest(input.vehicles[v].breaks.size()),
    fwd_smallest_breaks_load_margin(input.vehicles[v].breaks.size()),
    bwd_smallest_breaks_load_margin(input.vehicles[v].breaks.size()) {
  const std::string break_error =
    std::format("Inconsistent breaks for vehicle {}.", input.vehicles[v].id);

  const auto& breaks = input.vehicles[v].breaks;

  Duration previous_earliest = v_start;

  // Store smallest margin component-wise.
  Amount fwd_smallest_margin = utils::max_amount(amount_size);
  Amount bwd_smallest_margin = utils::max_amount(amount_size);

  for (Index i = 0; i < breaks.size(); ++i) {
    const auto& b = breaks[i];
    const auto b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
      return previous_earliest <= tw.end;
    });
    if (b_tw == b.tws.end()) {
      throw InputException(break_error);
    }

    break_earliest[i] = std::max(previous_earliest, b_tw->start);

    previous_earliest = break_earliest[i] + b.service;

    if (b.max_load.has_value()) {
      const auto& max_load = b.max_load.value();
      for (std::size_t a = 0; a < amount_size; ++a) {
        if (max_load[a] < fwd_smallest_margin[a]) {
          fwd_smallest_margin[a] = max_load[a];
        }
      }
    }
    fwd_smallest_breaks_load_margin[i] = fwd_smallest_margin;
  }

  Duration next_latest = v_end;
  for (Index r_i = 0; r_i < breaks.size(); ++r_i) {
    const Index i = breaks.size() - 1 - r_i;
    const auto& b = breaks[i];

    Duration unused_travel = 0;
    if (!apply_break_latest(b, next_latest, unused_travel)) {
      throw InputException(break_error);
    }

    break_latest[i] = next_latest;

    next_latest = break_latest[i];

    if (break_latest[i] < break_earliest[i]) {
      throw InputException(break_error);
    }

    if (b.max_load.has_value()) {
      const auto& max_load = b.max_load.value();
      for (std::size_t a = 0; a < amount_size; ++a) {
        if (max_load[a] < bwd_smallest_margin[a]) {
          bwd_smallest_margin[a] = max_load[a];
        }
      }
    }
    bwd_smallest_breaks_load_margin[i] = bwd_smallest_margin;
  }
}

PreviousInfo TWRoute::previous_info(const Input& input,
                                    const Index job_rank,
                                    const Index rank) const {
  const auto& v = input.vehicles[v_rank];
  const auto& j = input.jobs[job_rank];

  PreviousInfo previous(v_start, 0);
  if (rank > 0) {
    const auto& previous_job = input.jobs[route[rank - 1]];
    previous.earliest = earliest[rank - 1] + action_time[rank - 1];
    previous.travel = v.duration(previous_job.index(), j.index());
    previous.location_index = previous_job.location.index();
  } else {
    if (has_start) {
      previous.location_index = v.start.value().index();
      previous.travel = v.duration(previous.location_index, j.index());
    }
  }

  return previous;
}

NextInfo TWRoute::next_info(const Input& input,
                            const Index job_rank,
                            const Index rank) const {
  const auto& v = input.vehicles[v_rank];
  const auto& j = input.jobs[job_rank];

  NextInfo next(v_end, 0);
  if (rank == route.size()) {
    if (has_end) {
      next.travel = v.duration(j.index(), v.end.value().index());
    }
  } else {
    next.latest = latest[rank];
    next.travel = v.duration(j.index(), input.jobs[route[rank]].index());
  }

  return next;
}

void TWRoute::fwd_update_earliest_from(const Input& input, Index rank) {
  const auto& v = input.vehicles[v_rank];

  Duration current_earliest = earliest[rank];
  bool handle_last_breaks = true;

  for (Index i = rank + 1; i < route.size(); ++i) {
    const auto& next_j = input.jobs[route[i]];
    Duration remaining_travel_time =
      v.duration(input.jobs[route[i - 1]].index(), next_j.index());
    // Departure from the previous job; breaks advance it in place.
    Duration current_departure = current_earliest + action_time[i - 1];

    // Update earliest dates and margins for breaks.
    assert(breaks_at_rank[i] <= breaks_counts[i]);
    Index break_rank = breaks_counts[i] - breaks_at_rank[i];

    for (Index r = 0; r < breaks_at_rank[i]; ++r, ++break_rank) {
      const bool break_is_valid =
        apply_break_earliest(v.breaks[break_rank],
                             current_departure,
                             remaining_travel_time,
                             break_earliest[break_rank]);
      assert(break_is_valid);
    }

    // Back to the job after breaks.
    current_earliest = current_departure + remaining_travel_time;

    const auto j_tw = first_reachable_tw(next_j.tws, current_earliest);
    assert(j_tw != next_j.tws.end());

    current_earliest = std::max(current_earliest, j_tw->start);

    // Check consistency except for situation where latest date has
    // been reset to 0 to force backward propagation after this call
    // to fwd_update_earliest_from.
    assert(current_earliest <= latest[i] || (i == rank + 1 && latest[i] == 0));
    if (current_earliest == earliest[i]) {
      // There won't be any further update so stop earliest date
      // propagation.
      handle_last_breaks = false;
      break;
    }

    earliest[i] = current_earliest;
  }

  if (handle_last_breaks) {
    // Update earliest dates and margins for potential breaks right
    // before route end.
    const Index i = route.size();
    Duration remaining_travel_time =
      (v.has_end())
        ? v.duration(input.jobs[route[i - 1]].index(), v.end.value().index())
        : 0;

    // Departure from the last job; breaks advance it in place.
    Duration current_departure = current_earliest + action_time[i - 1];

    assert(breaks_at_rank[i] <= breaks_counts[i]);
    Index break_rank = breaks_counts[i] - breaks_at_rank[i];

    for (Index r = 0; r < breaks_at_rank[i]; ++r, ++break_rank) {
      const bool break_is_valid =
        apply_break_earliest(v.breaks[break_rank],
                             current_departure,
                             remaining_travel_time,
                             break_earliest[break_rank]);
      assert(break_is_valid);
    }

    earliest_end = current_departure + remaining_travel_time;
    assert(earliest_end <= v_end);
  }
}

void TWRoute::bwd_update_latest_from(const Input& input, Index rank) {
  const auto& v = input.vehicles[v_rank];

  Duration current_latest = latest[rank];
  bool handle_first_breaks = true;

  for (Index next_i = rank; next_i > 0; --next_i) {
    const auto& previous_j = input.jobs[route[next_i - 1]];
    Duration remaining_travel_time =
      v.duration(previous_j.index(), input.jobs[route[next_i]].index());

    // Update latest dates and margins for breaks.
    assert(breaks_at_rank[next_i] <= breaks_counts[next_i]);
    Index break_rank = breaks_counts[next_i];

    for (Index r = 0; r < breaks_at_rank[next_i]; ++r) {
      --break_rank;

      const auto& b = v.breaks[break_rank];
      const bool break_is_valid =
        apply_break_latest(b, current_latest, remaining_travel_time);
      assert(break_is_valid);

      break_latest[break_rank] = current_latest;
    }

    // Back to the job after breaks.
    auto gap = action_time[next_i - 1] + remaining_travel_time;
    assert(gap <= current_latest);
    current_latest -= gap;

    const auto j_tw =
      std::find_if(previous_j.tws.rbegin(),
                   previous_j.tws.rend(),
                   [&](const auto& tw) { return tw.start <= current_latest; });
    assert(j_tw != previous_j.tws.rend());

    current_latest = std::min(current_latest, j_tw->end);

    assert(earliest[next_i - 1] <= current_latest);
    if (current_latest == latest[next_i - 1]) {
      // There won't be any further update so stop latest date
      // propagation.
      handle_first_breaks = false;
      break;
    }

    latest[next_i - 1] = current_latest;
  }

  if (handle_first_breaks) {
    // Update latest dates and margins for breaks right before the
    // first job.
    const Index next_i = 0;

    assert(breaks_at_rank[next_i] <= breaks_counts[next_i]);
    Index break_rank = breaks_counts[next_i];

    for (Index r = 0; r < breaks_at_rank[next_i]; ++r) {
      --break_rank;
      const auto& b = v.breaks[break_rank];
      Duration unused_travel = 0;
      const bool break_is_valid =
        apply_break_latest(b, current_latest, unused_travel);
      assert(break_is_valid);

      break_latest[break_rank] = current_latest;
    }
  }
}

void TWRoute::update_last_latest_date(const Input& input) {
  assert(!route.empty());

  const auto& v = input.vehicles[v_rank];
  auto next = next_info(input, route.back(), route.size());

  // Latest date for breaks before end.
  Index break_rank = breaks_counts[route.size()];
  for (Index r = 0; r < breaks_at_rank[route.size()]; ++r) {
    --break_rank;
    const auto& b = v.breaks[break_rank];
    const bool break_is_valid = apply_break_latest(b, next.latest, next.travel);
    assert(break_is_valid);

    break_latest[break_rank] = next.latest;
  }

  // Latest date for last job.
  const auto& j = input.jobs[route.back()];
  const auto gap = action_time.back() + next.travel;
  assert(gap <= next.latest);
  next.latest -= gap;

  const auto j_tw =
    std::find_if(j.tws.rbegin(), j.tws.rend(), [&](const auto& tw) {
      return tw.start <= next.latest;
    });
  assert(j_tw != j.tws.rend());

  latest.back() = std::min(next.latest, j_tw->end);
}

void TWRoute::fwd_update_action_time_from(const Input& input, Index rank) {
  Index current_index = input.jobs[route[rank]].index();

  for (Index i = rank + 1; i < route.size(); ++i) {
    const auto& next_j = input.jobs[route[i]];
    const auto next_index = next_j.index();

    const auto next_action_time =
      action_time_for(next_j, v_type, current_index);

    action_time[i] = next_action_time;
    current_index = next_index;
  }
}

void TWRoute::fwd_update_breaks_load_margin_from(const Input& input,
                                                 Index rank) {
  const auto& v = input.vehicles[v_rank];

  // Last valid fwd_smallest value, if any.
  auto fwd_smallest =
    (breaks_counts[rank] == 0)
      ? utils::max_amount(input.get_amount_size())
      : fwd_smallest_breaks_load_margin[breaks_counts[rank] - 1];

  for (Index i = rank; i <= route.size(); ++i) {
    if (breaks_at_rank[i] != 0) {
      // Update for breaks right before job at rank i.
      const auto& current_load = load_at_step(i);

      for (auto break_rank = breaks_counts[i] - breaks_at_rank[i];
           break_rank < breaks_counts[i];
           ++break_rank) {
        const auto& b = v.breaks[break_rank];

        assert(b.is_valid_for_load(current_load));
        auto current_margin = (b.max_load.has_value())
                                ? b.max_load.value() - current_load
                                : utils::max_amount(input.get_amount_size());

        for (std::size_t a = 0; a < fwd_smallest.size(); ++a) {
          fwd_smallest[a] = std::min(fwd_smallest[a], current_margin[a]);
        }

        assert(input.zero_amount() <= fwd_smallest);
        fwd_smallest_breaks_load_margin[break_rank] = fwd_smallest;
      }
    }
  }
}

void TWRoute::bwd_update_breaks_load_margin_from(const Input& input,
                                                 Index rank) {
  const auto& v = input.vehicles[v_rank];

  // Last valid bwd_smallest value, if any.
  auto bwd_smallest = (breaks_counts[rank] == breaks_counts.back())
                        ? utils::max_amount(input.get_amount_size())
                        : bwd_smallest_breaks_load_margin[breaks_counts[rank]];

  for (Index bwd_i = 0; bwd_i <= rank; ++bwd_i) {
    const auto i = rank - bwd_i;
    if (breaks_at_rank[i] != 0) {
      // Update for breaks right before job at rank i.
      const auto& current_load = load_at_step(i);

      for (unsigned bwd_break_count = 0; bwd_break_count < breaks_at_rank[i];
           ++bwd_break_count) {
        const auto break_rank = breaks_counts[i] - 1 - bwd_break_count;
        const auto& b = v.breaks[break_rank];

        assert(b.is_valid_for_load(current_load));
        auto current_margin = (b.max_load.has_value())
                                ? b.max_load.value() - current_load
                                : utils::max_amount(input.get_amount_size());

        for (std::size_t a = 0; a < bwd_smallest.size(); ++a) {
          bwd_smallest[a] = std::min(bwd_smallest[a], current_margin[a]);
        }

        assert(input.zero_amount() <= bwd_smallest);
        bwd_smallest_breaks_load_margin[break_rank] = bwd_smallest;
      }
    }
  }
}

OrderChoice::OrderChoice(const Input& input,
                         const Index job_rank,
                         const Break& b,
                         const PreviousInfo& previous)
  : input(input),
    j_tw(std::ranges::find_if(input.jobs[job_rank].tws,
                              [&](const auto& tw) {
                                return previous.earliest + previous.travel <=
                                       tw.end;
                              })),
    b_tw(std::ranges::find_if(b.tws, [&](const auto& tw) {
      return previous.earliest <= tw.end;
    })) {
}

OrderChoice TWRoute::order_choice(const Input& input,
                                  const Index job_rank,
                                  const Duration job_action_time,
                                  const Break& b,
                                  const PreviousInfo& previous,
                                  const NextInfo& next,
                                  const Amount& current_load,
                                  bool check_max_load) const {
  OrderChoice oc(input, job_rank, b, previous);
  const auto& v = input.vehicles[v_rank];
  const auto& j = input.jobs[job_rank];

  if (oc.j_tw == j.tws.end() || oc.b_tw == b.tws.end()) {
    // If either job or break can't fit first, then none of the
    // orderings are valid.
    return oc;
  }

  Duration job_then_break_end;
  Duration break_then_job_end;

  // Try putting job first then break.
  const Duration earliest_job_end =
    std::max(previous.earliest + previous.travel, oc.j_tw->start) +
    job_action_time;
  Duration job_then_break_margin = 0;

  const auto new_b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
    return earliest_job_end <= tw.end;
  });
  if (new_b_tw == b.tws.end()) {
    // Break does not fit after job due to its time windows. Only
    // option is to choose break first, if valid for max_load.
    oc.add_break_first = !check_max_load || b.is_valid_for_load(current_load);
    return oc;
  }

  Duration travel_after_break = next.travel;
  if (earliest_job_end < new_b_tw->start) {
    job_then_break_margin = new_b_tw->start - earliest_job_end;
    if (job_then_break_margin < travel_after_break) {
      travel_after_break -= job_then_break_margin;
    } else {
      travel_after_break = 0;
    }

    job_then_break_end = oc.b_tw->start + b.service;
  } else {
    job_then_break_end = earliest_job_end + b.service;
  }

  if (job_then_break_end + travel_after_break > next.latest) {
    // Starting the break is possible but then next step is not.
    oc.add_break_first = true;
    return oc;
  }

  if (check_max_load && j.type == JOB_TYPE::SINGLE &&
      !(j.pickup <= bwd_smallest_breaks_load_margin[v.break_rank(b.id)])) {
    // Break won't fit after job for load reason.
    oc.add_break_first = b.is_valid_for_load(current_load);
    return oc;
  }

  // Try putting break first then job.
  if (check_max_load && !b.is_valid_for_load(current_load)) {
    // Not doable based on max_load, only option is to choose job
    // first.
    oc.add_job_first = true;
    return oc;
  }

  travel_after_break = previous.travel;
  Duration earliest_job_start = previous.earliest;

  if (previous.earliest < oc.b_tw->start) {
    if (const auto margin = oc.b_tw->start - previous.earliest;
        margin < travel_after_break) {
      travel_after_break -= margin;
    } else {
      travel_after_break = 0;
    }

    earliest_job_start = oc.b_tw->start;
  }

  earliest_job_start += b.service + travel_after_break;

  const auto new_j_tw = std::ranges::find_if(j.tws, [&](const auto& tw) {
    return earliest_job_start <= tw.end;
  });

  if (new_j_tw == j.tws.end()) {
    // Job does not fit after break due to its time windows. Only
    // option is to choose job first.
    oc.add_job_first = true;
    return oc;
  }
  break_then_job_end =
    std::max(earliest_job_start, new_j_tw->start) + job_action_time;

  if (break_then_job_end + next.travel > next.latest) {
    // Arrival at the job is valid but next step is not.
    oc.add_job_first = true;
    return oc;
  }

  // Now both ordering options are doable based on timing constraints.

  // For a pickup, we favor putting the pickup first, except if adding
  // the delivery afterwards is not possible. This is mandatory to
  // avoid heuristically forcing a pickup -> break choice resulting in
  // invalid options, while break -> pickup -> delivery might be
  // valid.
  if (j.type == JOB_TYPE::PICKUP) {
    const auto& matching_d = input.jobs[job_rank + 1];
    assert(matching_d.type == JOB_TYPE::DELIVERY);

    // Try pickup -> break -> delivery.
    auto delivery_travel = v.duration(j.index(), matching_d.index());
    if (job_then_break_margin < delivery_travel) {
      delivery_travel -= job_then_break_margin;
    } else {
      delivery_travel = 0;
    }
    const Duration pb_d_candidate = job_then_break_end + delivery_travel;
    if (const auto pb_d_tw = std::ranges::find_if(matching_d.tws,
                                                  [&](const auto& tw) {
                                                    return pb_d_candidate <=
                                                           tw.end;
                                                  });
        pb_d_tw != matching_d.tws.end() &&
        (!check_max_load || b.is_valid_for_load(current_load + j.pickup))) {
      // pickup -> break -> delivery is doable, choose pickup first.
      oc.add_job_first = true;
      return oc;
    }

    // Previous order not doable, so try pickup -> delivery -> break.
    const Duration delivery_candidate =
      earliest_job_end + v.duration(j.index(), matching_d.index());
    if (const auto d_tw = std::ranges::find_if(matching_d.tws,
                                               [&](const auto& tw) {
                                                 return delivery_candidate <=
                                                        tw.end;
                                               });
        d_tw != matching_d.tws.end()) {
      const auto matching_d_action_time =
        action_time_for(matching_d, v_type, j.index());

      const Duration break_candidate =
        std::max(delivery_candidate, d_tw->start) + matching_d_action_time;

      const auto after_d_b_tw =
        std::ranges::find_if(b.tws, [&](const auto& tw) {
          return break_candidate <= tw.end;
        });
      if (after_d_b_tw != b.tws.end()) {
        // pickup -> delivery -> break is doable, choose pickup first.
        assert(!check_max_load || b.is_valid_for_load(current_load));
        oc.add_job_first = true;
        return oc;
      }
    }

    // Doing pickup first actually leads to infeasible options, so put
    // break first.
    oc.add_break_first = true;
    return oc;
  }

  // For a single job, we pick the ordering minimizing earliest end
  // date for sequence.
  if (break_then_job_end < job_then_break_end) {
    oc.add_break_first = true;
  } else if (break_then_job_end == job_then_break_end) {
    // If end date is the same for both ordering options, decide based
    // on earliest deadline, except for deliveries. If a delivery
    // without TW constraint is postponed, it can introduce arbitrary
    // waiting time between zero max_load breaks.
    if (j.type == JOB_TYPE::DELIVERY || oc.j_tw->end <= oc.b_tw->end) {
      oc.add_job_first = true;
    } else {
      oc.add_break_first = true;
    }
  } else {
    oc.add_job_first = true;
  }

  return oc;
}

bool TWRoute::check_max_transit_time(const Input& input,
                                     const std::vector<TraceEvent>& trace,
                                     const Index first_rank,
                                     const Index last_rank,
                                     const Index inserted_job_count) const {
  // Candidate-path LB filter: reject only when the path lower bound alone
  // exceeds the cap. Exact scheduling is handled by the engine downstream.
  const auto& v = input.vehicles[v_rank];

  const auto event_for = [&](const Index rank) -> const TraceEvent* {
    const auto found = std::ranges::find_if(trace, [&](const auto& event) {
      return event.kind == TraceEvent::Kind::JOB && event.rank == rank;
    });
    return (found == trace.end()) ? nullptr : &*found;
  };
  const auto trace_candidate_rank =
    [&](const Index rank) -> std::optional<Index> {
    Index candidate_rank = first_rank;
    for (const auto& event : trace) {
      if (event.kind == TraceEvent::Kind::JOB) {
        if (event.rank == rank) {
          return candidate_rank;
        }
        ++candidate_rank;
      }
    }
    return std::nullopt;
  };
  // Returns the candidate-route rank of an external (non-inserted) job, or
  // nullopt when it is not part of the candidate route.
  const auto external_rank = [&](const Index rank) -> std::optional<Index> {
    const auto removed_count = last_rank - first_rank;
    for (Index i = 0; i < route.size(); ++i) {
      if ((i < first_rank || last_rank <= i) && route[i] == rank) {
        return (i < first_rank)
                 ? i
                 : static_cast<Index>(i + inserted_job_count - removed_count);
      }
    }
    return std::nullopt;
  };

  std::vector<Index> checked_pickups;
  checked_pickups.reserve(inserted_job_count);
  const auto candidate_route_size = static_cast<Index>(
    route.size() - (last_rank - first_rank) + inserted_job_count);

  // Returns the job rank at candidate-route position k. The candidate route
  // is: route[0..first_rank), trace JOB events, route[last_rank..end).
  const auto candidate_job_at = [&](const Index k) -> Index {
    const auto removed_count = last_rank - first_rank;
    if (k < first_rank) {
      return route[k];
    }
    if (k < first_rank + inserted_job_count) {
      Index job_pos = k - first_rank;
      for (const auto& evt : trace) {
        if (evt.kind == TraceEvent::Kind::JOB) {
          if (job_pos == 0) {
            return evt.rank;
          }
          --job_pos;
        }
      }
      return std::numeric_limits<Index>::max();
    }
    const auto orig_k = k - inserted_job_count + removed_count;
    return route[orig_k];
  };

  // Candidate-path lower bound: walk the candidate sequence from the pickup
  // at position pcr to the delivery at position dcr, summing travel between
  // consecutive stops and intermediate action times (breaks skipped
  // conservatively: they only add time). Valid for arbitrary non-metric
  // matrices: the cargo traverses exactly this path, so the sum lower-bounds
  // transit time for every feasible schedule. Returns nullopt when a
  // candidate position cannot be resolved.
  const auto path_lower_bound =
    [&](const Index pickup_rank,
        const Index pcr,
        const Index dcr) -> std::optional<Duration> {
    Index prev_loc = input.jobs[pickup_rank].index();
    Duration path_lb = 0;
    for (Index k = pcr + 1; k <= dcr; ++k) {
      const auto job_r = candidate_job_at(k);
      if (job_r == std::numeric_limits<Index>::max()) {
        return std::nullopt;
      }
      const auto& jk = input.jobs[job_r];
      const Index curr_loc = jk.index();
      path_lb = saturating_add(path_lb, v.duration(prev_loc, curr_loc));
      if (k < dcr) {
        // Intermediate stop: suppress setup when the candidate predecessor
        // location matches, mirroring the forward-trace action-time rule.
        const Duration at = action_time_for(jk, v_type, prev_loc);
        path_lb = saturating_add(path_lb, at);
      }
      prev_loc = curr_loc;
    }
    return path_lb;
  };

  for (const auto& event : trace) {
    if (event.kind != TraceEvent::Kind::JOB ||
        !input.jobs[event.rank].max_transit_time.has_value()) {
      continue;
    }
    // A delivery's input rank is always its pickup's rank plus one, so the
    // subtraction below cannot underflow for valid input.
    assert(input.jobs[event.rank].type == JOB_TYPE::PICKUP || event.rank >= 1);
    const Index pickup_rank = (input.jobs[event.rank].type == JOB_TYPE::PICKUP)
                                ? event.rank
                                : event.rank - 1;
    // A trace contains only a few events, so linear deduplication avoids an
    // input-sized allocation in this local-search hot path.
    if (std::ranges::find(checked_pickups, pickup_rank) !=
        checked_pickups.end()) {
      continue;
    }
    checked_pickups.push_back(pickup_rank);

    const auto delivery_rank = pickup_rank + 1;
    const auto* pickup_event = event_for(pickup_rank);
    const auto* delivery_event = event_for(delivery_rank);
    const auto pickup_external =
      pickup_event ? std::optional<Index>() : external_rank(pickup_rank);
    const auto delivery_external =
      delivery_event ? std::optional<Index>() : external_rank(delivery_rank);
    const auto pickup_candidate_rank =
      pickup_event ? trace_candidate_rank(pickup_rank) : pickup_external;
    const auto delivery_candidate_rank =
      delivery_event ? trace_candidate_rank(delivery_rank) : delivery_external;
    if ((!pickup_event && (!pickup_external.has_value() ||
                           pickup_external.value() >= candidate_route_size)) ||
        (!delivery_event &&
         (!delivery_external.has_value() ||
          delivery_external.value() >= candidate_route_size))) {
      continue;
    }

    Duration lower_bound = 0;
    if (pickup_candidate_rank.has_value() &&
        delivery_candidate_rank.has_value() &&
        pickup_candidate_rank.value() < delivery_candidate_rank.value()) {
      const auto path_lb = path_lower_bound(pickup_rank,
                                            pickup_candidate_rank.value(),
                                            delivery_candidate_rank.value());
      if (path_lb.has_value()) {
        lower_bound = path_lb.value();
      }
    }
    // Reject only when the path lower bound exceeds the cap.
    if (lower_bound > input.jobs[pickup_rank].max_transit_time.value()) {
      return false;
    }
  }

  // Also check constrained shipment pairs where both halves are external to
  // the trace (pickup in route[0..first_rank), delivery in
  // route[last_rank..end)). The trace loop above misses these because neither
  // half appears as a trace event: this happens when only unrelated jobs are
  // inserted between an existing pickup/delivery pair. Path lower bound only.
  for (Index i = 0; i < first_rank; ++i) {
    const Index job_r = route[i];
    if (input.jobs[job_r].type != JOB_TYPE::PICKUP ||
        !input.jobs[job_r].max_transit_time.has_value()) {
      continue;
    }
    const Index pickup_rank = job_r;
    if (std::ranges::find(checked_pickups, pickup_rank) !=
        checked_pickups.end()) {
      continue;
    }
    const Index delivery_rank = pickup_rank + 1;
    std::optional<Index> delivery_ext_rank;
    for (Index j = last_rank; j < route.size(); ++j) {
      if (route[j] == delivery_rank) {
        delivery_ext_rank = j;
        break;
      }
    }
    if (!delivery_ext_rank.has_value()) {
      continue;
    }
    checked_pickups.push_back(pickup_rank);
    const Index removed_count = last_rank - first_rank;
    const Index pcr = i;
    const Index dcr =
      delivery_ext_rank.value() + inserted_job_count - removed_count;
    if (pcr >= dcr) {
      continue;
    }
    const auto path_lb = path_lower_bound(pickup_rank, pcr, dcr);
    if (path_lb.has_value() &&
        path_lb.value() > input.jobs[pickup_rank].max_transit_time.value()) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Cap-aware scheduler engine: whole-route overload. The committed route is
// the degenerate candidate: an empty insertion range at rank 0 leaves the
// entire route as suffix, so one implementation serves both.
// ---------------------------------------------------------------------------
std::optional<std::vector<Duration>> TWRoute::compute_cap_compliant_schedule(
  const Input& input,
  const std::size_t iter_budget_factor) const {
  assert(!route.empty());
  return compute_cap_compliant_schedule(input,
                                        std::vector<TraceEvent>(),
                                        0,
                                        0,
                                        0,
                                        iter_budget_factor);
}

// ---------------------------------------------------------------------------
// Cap-aware scheduler engine: virtual-candidate overload.
// ---------------------------------------------------------------------------
std::optional<std::vector<Duration>> TWRoute::compute_cap_compliant_schedule(
  const Input& input,
  const std::vector<TraceEvent>& trace,
  const Index first_rank,
  const Index last_rank,
  const Index inserted_job_count,
  const std::size_t iter_budget_factor) const {

  const auto& v = input.vehicles[v_rank];
  const Index NO_LOC = std::numeric_limits<Index>::max();

  // -------------------------------------------------------------------------
  // Build the virtual job sequence and break layout from:
  //   route[0..first_rank)  : prefix (committed, unchanged)
  //   trace JOB events      : insertion range (with trace BREAK events)
  //   route[last_rank..)    : suffix (committed, unchanged)
  //
  // Trace BREAK events carry vehicle-break ranks in
  //   [breaks_counts[first_rank]-breaks_at_rank[first_rank],
  //   breaks_counts[last_rank]-1].
  // All committed breaks at positions first_rank..last_rank ARE in the trace;
  // suffix positions > last_rank use their committed layout.
  // -------------------------------------------------------------------------
  const std::size_t removed_count = last_rank - first_rank;
  const std::size_t virt_N = route.size() + inserted_job_count - removed_count;

  if (virt_N == 0) {
    return std::vector<Duration>{};
  }

  // Reuse thread_local scratch buffers; Invariant 5: no cross-call aliasing
  // (function returns by value; callers hold no references to these vectors).
  auto& job_seq = tls_job_seq;
  auto& brk_cnt = tls_brk_cnt;
  auto& brk_fst = tls_brk_fst;
  job_seq.clear();
  brk_cnt.clear();
  brk_fst.clear();
  job_seq.reserve(virt_N);
  brk_cnt.reserve(virt_N + 1);
  brk_fst.reserve(virt_N + 1);

  // Prefix: route[0..first_rank).
  for (Index r = 0; r < first_rank; ++r) {
    job_seq.push_back(route[r]);
    brk_cnt.push_back(breaks_at_rank[r]);
    brk_fst.push_back(breaks_counts[r] - breaks_at_rank[r]);
  }

  // Insertion range: parse trace for JOB events with preceding BREAKs.
  // Contiguous BREAK events before each JOB → (brk_fst, brk_cnt) pair.
  unsigned trace_cur_brk_fst = 0;
  unsigned trace_cur_brk_cnt = 0;
  unsigned trailing_brk_fst = 0;
  unsigned trailing_brk_cnt = 0;
  bool have_pending_break = false;

  for (const auto& ev : trace) {
    if (ev.kind == TraceEvent::Kind::BREAK) {
      if (!have_pending_break) {
        trace_cur_brk_fst = static_cast<unsigned>(ev.rank);
        have_pending_break = true;
      }
      ++trace_cur_brk_cnt;
    } else { // JOB
      job_seq.push_back(ev.rank);
      brk_cnt.push_back(trace_cur_brk_cnt);
      brk_fst.push_back(trace_cur_brk_cnt > 0 ? trace_cur_brk_fst : 0);
      trace_cur_brk_fst = 0;
      trace_cur_brk_cnt = 0;
      have_pending_break = false;
    }
  }
  // Trailing breaks after last trace JOB → go before first suffix job.
  trailing_brk_cnt = trace_cur_brk_cnt;
  trailing_brk_fst = (trace_cur_brk_cnt > 0) ? trace_cur_brk_fst : 0;

  // With an empty edit (no trace, empty insertion range) no committed
  // position was replaced, so no break ever entered a trace and every slot
  // reads the committed layout; the whole-route overload reduces to this.
  const bool empty_edit = trace.empty() && first_rank == last_rank;

  // Suffix: route[last_rank..route.size()).
  for (Index r = last_rank; r < static_cast<Index>(route.size()); ++r) {
    job_seq.push_back(route[r]);
    if (r == last_rank && !empty_edit) {
      // Breaks at committed position last_rank were processed by order_choice
      // inside the insertion range and appear in the trace (possibly as
      // trailing trace BREAKs). Use trailing trace breaks here.
      brk_cnt.push_back(trailing_brk_cnt);
      brk_fst.push_back(trailing_brk_fst);
    } else {
      // r > last_rank: committed breaks NOT in the trace.
      brk_cnt.push_back(breaks_at_rank[r]);
      brk_fst.push_back(breaks_counts[r] - breaks_at_rank[r]);
    }
  }

  assert(job_seq.size() == virt_N);

  // Terminal slot (index virt_N): breaks after the last virtual job.
  // When a suffix exists, terminal breaks are the committed terminal breaks
  // (not covered by the trace range). When no suffix, all breaks including
  // terminal were routed through order_choice into the trace, so trailing
  // trace breaks ARE the committed terminal breaks.
  if (empty_edit || last_rank < static_cast<Index>(route.size())) {
    brk_cnt.push_back(breaks_at_rank[route.size()]);
    brk_fst.push_back(breaks_counts[route.size()] -
                      breaks_at_rank[route.size()]);
  } else {
    brk_cnt.push_back(trailing_brk_cnt);
    brk_fst.push_back(trailing_brk_fst);
  }
  // Every vehicle break is assigned to exactly one virtual-route slot.
  // Rank agreement with the layout replace() later commits is not asserted
  // here; output realization re-certifies the schedule against the
  // committed layout, so any divergence surfaces there rather than
  // silently.
  {
    unsigned total = 0;
    for (const auto c : brk_cnt) {
      total += c;
    }
    assert(total == static_cast<unsigned>(v.breaks.size()));
  }

  // Mutable schedule state: thread_local scratch buffers (Invariants 2 to 4:
  // assign/resize here; zero-init via assign where the algorithm requires it).
  auto& S = tls_S;
  auto& A = tls_A;
  auto& dep = tls_dep;
  auto& tw_idx = tls_tw_idx;
  auto& loc = tls_loc;
  auto& break_S = tls_break_S;
  S.resize(virt_N);
  A.resize(virt_N);
  dep.resize(virt_N);
  tw_idx.assign(virt_N, 0); // Invariant 2: must zero-init each call.
  loc.resize(virt_N);
  break_S.assign(v.breaks.size(), 0); // Invariant 3: must zero-init each call.

  // Initial ASAP pass. With a committed prefix (first_rank > 0), positions
  // 0..first_rank-1 are not re-simulated: cap_asap_forward mirrors
  // fwd_update_earliest_from, so their ASAP values equal the committed
  // earliest[]/action_time[] arrays and the forward pass can start at
  // first_rank. Any returned schedule is still re-verified from scratch by
  // the self-verification block below, so seeding can only cause rejection,
  // never a false acceptance.
  const Index init_loc = has_start ? v.start.value().index() : NO_LOC;

  if (first_rank > 0) {
    // First break index at position first_rank in the virtual route, by the
    // cumulative-count invariant maintained in replace(). Breaks before it
    // are committed prefix breaks; later ones are filled by cap_asap_forward.
    const Index B_fst = breaks_counts[first_rank] - breaks_at_rank[first_rank];
    for (Index b = 0; b < B_fst; ++b) {
      break_S[b] = break_earliest[b];
    }

    for (Index r = 0; r < first_rank; ++r) {
      const auto& j = input.jobs[job_seq[r]];
      S[r] = earliest[r];
      A[r] = action_time[r];
      dep[r] = saturating_add(earliest[r], action_time[r]);
      loc[r] = j.index();
      // Committed route validity guarantees earliest[r] lies inside the TW
      // selected when the route was built, so the first window whose end
      // covers it is exactly the one cap_asap_forward would pick.
      const auto j_tw_it = std::ranges::find_if(j.tws, [&](const auto& tw) {
        return earliest[r] <= tw.end;
      });
      assert(j_tw_it != j.tws.end());
      assert(j_tw_it->start <= earliest[r]);
      tw_idx[r] = static_cast<std::size_t>(j_tw_it - j.tws.begin());
    }

    if (first_rank < virt_N) {
      // Depart from the last committed prefix job. When first_rank == virt_N
      // (pure suffix removal, no insertion) there is nothing to simulate
      // after the prefix; self-verification below still checks vehicle end.
      const Duration seed_dep = dep[first_rank - 1];
      const Index seed_loc = loc[first_rank - 1];
      const Duration seed_rem =
        v.duration(seed_loc, input.jobs[job_seq[first_rank]].index());

      if (!cap_asap_forward(input,
                            v,
                            v_type,
                            v_end,
                            job_seq,
                            brk_cnt,
                            brk_fst,
                            first_rank,
                            seed_dep,
                            seed_loc,
                            seed_rem,
                            S,
                            A,
                            dep,
                            tw_idx,
                            loc,
                            break_S)) {
        return std::nullopt;
      }
    }
  } else {
    // No prefix: start from vehicle origin.
    const Duration init_rem =
      (init_loc != NO_LOC)
        ? v.duration(init_loc, input.jobs[job_seq[0]].index())
        : 0;
    if (!cap_asap_forward(input,
                          v,
                          v_type,
                          v_end,
                          job_seq,
                          brk_cnt,
                          brk_fst,
                          0,
                          v_start,
                          init_loc,
                          init_rem,
                          S,
                          A,
                          dep,
                          tw_idx,
                          loc,
                          break_S)) {
      return std::nullopt;
    }
  }

  // Build constrained-pair list for the virtual route.
  // Invariant 1: pairs.clear() before the loop; must not be moved after.
  auto& pairs = tls_pairs;
  pairs.clear();
  for (std::size_t pos = 0; pos < virt_N; ++pos) {
    const Index job_rank = job_seq[pos];
    const auto& j = input.jobs[job_rank];
    if (j.type != JOB_TYPE::PICKUP || !j.max_transit_time.has_value()) {
      continue;
    }
    const Index del_input_rank = job_rank + 1;
    for (std::size_t d = pos + 1; d < virt_N; ++d) {
      if (job_seq[d] == del_input_rank) {
        pairs.push_back({pos, d, j.max_transit_time.value()});
        break;
      }
    }
  }

  const std::size_t K = pairs.size();
  // A factor of 0 removes the budget: service starts are monotone
  // non-decreasing and bounded by time-window ends, so the fixpoint
  // terminates without it. Realization retries use that.
  const std::size_t max_iters = (iter_budget_factor == 0)
                                  ? std::numeric_limits<std::size_t>::max()
                                  : iter_budget_factor * (2 * K + 2);

  // Fixpoint loop.
  bool found_compliant = false;
  for (std::size_t outer = 0;; ++outer) {
    Duration worst_excess = 0;
    const CapPairBuf* worst = nullptr;
    for (const auto& pr : pairs) {
      const Duration transit_time = saturating_sub(S[pr.d_pos], dep[pr.p_pos]);
      const Duration excess = saturating_sub(transit_time, pr.cap);
      if (excess > worst_excess) {
        worst_excess = excess;
        worst = &pr;
      }
    }
    if (worst == nullptr) {
      found_compliant = true;
      break;
    }
    if (outer >= max_iters) {
      break;
    }

    const std::size_t P = worst->p_pos;
    const auto& pickup_j = input.jobs[job_seq[P]];

    Duration candidate_S = saturating_add(S[P], worst_excess);
    std::size_t attempt_tw = tw_idx[P];

    while (candidate_S > pickup_j.tws[attempt_tw].end) {
      ++attempt_tw;
      if (attempt_tw >= pickup_j.tws.size()) {
        return std::nullopt;
      }
      // The required delay is a lower bound on the pickup start; entering a
      // later window must not undershoot it.
      candidate_S = std::max(candidate_S, pickup_j.tws[attempt_tw].start);
    }

    S[P] = candidate_S;
    tw_idx[P] = attempt_tw;
    dep[P] = saturating_add(S[P], A[P]);

    const Duration next_rem =
      v.duration(loc[P], input.jobs[job_seq[P + 1]].index());
    if (!cap_asap_forward(input,
                          v,
                          v_type,
                          v_end,
                          job_seq,
                          brk_cnt,
                          brk_fst,
                          P + 1,
                          dep[P],
                          loc[P],
                          next_rem,
                          S,
                          A,
                          dep,
                          tw_idx,
                          loc,
                          break_S)) {
      return std::nullopt;
    }
  }

  if (!found_compliant) {
    return std::nullopt;
  }

  // Self-verification: rebuild the schedule from raw input and check it.
  {
    Duration sv_dep = v_start;
    Index sv_loc = init_loc;
    // Invariant 4: pred_loc.assign(virt_N, NO_LOC) zero-inits each call.
    auto& pred_loc = tls_pred_loc;
    pred_loc.assign(virt_N, NO_LOC);

    for (std::size_t pos = 0; pos < virt_N; ++pos) {
      const Index job_rank = job_seq[pos];
      const auto& j = input.jobs[job_rank];
      const Index job_loc = j.index();

      Duration rem = (sv_loc != NO_LOC) ? v.duration(sv_loc, job_loc) : 0;

      for (unsigned b = 0; b < brk_cnt[pos]; ++b) {
        const unsigned brk_rank = brk_fst[pos] + b;
        const auto& bk = v.breaks[brk_rank];
        const Duration bS = break_S[brk_rank];
        if (sv_dep > bS) {
          return std::nullopt;
        }
        const auto b_tw = std::ranges::find_if(bk.tws, [&](const auto& tw) {
          return bS <= tw.end;
        });
        if (b_tw == bk.tws.end() || bS < b_tw->start) {
          return std::nullopt;
        }
        if (sv_dep < bS) {
          const Duration margin = bS - sv_dep;
          rem = (margin < rem) ? rem - margin : 0;
        }
        sv_dep = saturating_add(bS, bk.service);
      }

      const Duration arrival = saturating_add(sv_dep, rem);
      if (arrival > S[pos]) {
        return std::nullopt;
      }
      const auto& tw = j.tws[tw_idx[pos]];
      if (S[pos] < tw.start || S[pos] > tw.end) {
        return std::nullopt;
      }
      pred_loc[pos] = sv_loc;
      const Duration A_raw = action_time_for(j, v_type, sv_loc);
      sv_dep = saturating_add(S[pos], A_raw);
      sv_loc = job_loc;
    }

    for (const auto& pr : pairs) {
      const auto& pj = input.jobs[job_seq[pr.p_pos]];
      const Duration A_P_raw = action_time_for(pj, v_type, pred_loc[pr.p_pos]);
      const Duration dep_P = saturating_add(S[pr.p_pos], A_P_raw);
      if (S[pr.d_pos] < dep_P) {
        return std::nullopt;
      }
      const Duration transit_time = S[pr.d_pos] - dep_P;
      if (transit_time > pr.cap) {
        return std::nullopt;
      }
    }

    // Self-verify terminal breaks (after last virtual job, before vehicle end).
    const Duration term_end_travel =
      v.has_end() ? v.duration(sv_loc, v.end.value().index()) : 0;
    Duration term_rem = term_end_travel;
    for (unsigned b = 0; b < brk_cnt[virt_N]; ++b) {
      const unsigned brk_rank = brk_fst[virt_N] + b;
      const auto& bk = v.breaks[brk_rank];
      const Duration bS = break_S[brk_rank];
      if (sv_dep > bS) {
        return std::nullopt; // Arrival past terminal break service start.
      }
      const auto b_tw = std::ranges::find_if(bk.tws, [&](const auto& tw) {
        return bS <= tw.end;
      });
      if (b_tw == bk.tws.end() || bS < b_tw->start) {
        return std::nullopt; // Terminal break outside any TW.
      }
      if (sv_dep < bS) {
        const Duration margin = bS - sv_dep;
        term_rem = (margin < term_rem) ? term_rem - margin : 0;
      }
      sv_dep = saturating_add(bS, bk.service);
    }
    // Self-verify vehicle end feasibility.
    if (v.has_end() && saturating_add(sv_dep, term_rem) > v_end) {
      return std::nullopt;
    }
  }

  return std::vector<Duration>(S.begin(), S.end());
}

std::optional<std::vector<Duration>> TWRoute::realize_cap_compliant_schedule(
  const Input& input,
  std::vector<Duration>* break_starts) const {
  // The counter gates enforcement, so a bookkeeping slip in replace()
  // would disable it silently; reconcile against route contents here, off
  // the hot path (once per route at output), since default builds keep
  // asserts enabled.
  assert(
    !input.has_max_transit_time() ||
    constrained_job_count_ ==
      static_cast<Index>(std::ranges::count_if(route, [&input](const Index r) {
        const auto& j = input.jobs[r];
        return j.type == JOB_TYPE::PICKUP && j.max_transit_time.has_value();
      })));
  if (!input.has_max_transit_time() || constrained_job_count_ == 0) {
    return std::nullopt;
  }
  auto sched = compute_cap_compliant_schedule(input);
  if (!sched.has_value()) {
    // Realization runs once per route at output time, off the hot path, and
    // a failure here would otherwise surface as an internal error. Retry
    // without an iteration budget: extra iterations only continue the same
    // monotone trajectory and recover deep pair interactions the hot-path
    // budget truncates.
    sched = compute_cap_compliant_schedule(input, 0);
  }
  if (sched.has_value() && break_starts != nullptr) {
    // tls_break_S was filled by the successful engine run on this thread.
    *break_starts = tls_break_S;
  }
  return sched;
}

template <std::forward_iterator Iter>
bool TWRoute::is_valid_addition_for_tw(const Input& input,
                                       const Amount& delivery,
                                       const Iter first_job,
                                       const Iter last_job,
                                       const Index first_rank,
                                       const Index last_rank,
                                       bool check_max_load) const {
  assert(first_job <= last_job);
  assert(first_rank <= last_rank);

  const auto& v = input.vehicles[v_rank];

  // Override this value if vehicle does not need this check anyway to
  // spare some work.
  check_max_load = v.has_break_max_load && check_max_load;

  // Gate trace recording and the cap checks: when no constrained pickups
  // are currently in the route (constrained_job_count_ == 0) and no inserted
  // job is a constrained pickup, no cap pair can span this route.
  // Mirror of the has_break_max_load idiom for the max_load check above.
  bool any_inserted_constrained = false;
  if (input.has_max_transit_time() && constrained_job_count_ == 0) {
    for (auto it = first_job; it != last_job; ++it) {
      const auto& ij = input.jobs[*it];
      if (ij.type == JOB_TYPE::PICKUP && ij.max_transit_time.has_value()) {
        any_inserted_constrained = true;
        break;
      }
    }
  }
  const bool filter_max_transit_time =
    input.has_max_transit_time() &&
    (constrained_job_count_ > 0 || any_inserted_constrained);

  const auto inserted_job_count =
    static_cast<size_t>(std::distance(first_job, last_job));
  // Reused per-thread scratch, cleared each call; nothing holds a reference
  // across calls (same contract as the engine's tls buffers).
  static thread_local std::vector<TraceEvent> trace;
  trace.clear();
  if (filter_max_transit_time) {
    trace.reserve(inserted_job_count + breaks_counts[last_rank] -
                  (breaks_counts[first_rank] - breaks_at_rank[first_rank]));
  }

  PreviousInfo current(0, 0);
  NextInfo next(0, 0);

  // Value initialization differ whether there are actually jobs added
  // or not.
  if (first_job < last_job) {
    current = previous_info(input, *first_job, first_rank);
    next = next_info(input, *(last_job - 1), last_rank);
  } else {
    // This is actually a removal as no jobs are inserted.
    current.earliest = v_start;
    next.latest = v_end;

    if (first_rank > 0) {
      const auto& previous_job = input.jobs[route[first_rank - 1]];
      current.earliest = earliest[first_rank - 1] + action_time[first_rank - 1];
      current.location_index = previous_job.index();

      if (last_rank < route.size()) {
        next.latest = latest[last_rank];
        next.travel = v.duration(previous_job.index(),
                                 input.jobs[route[last_rank]].index());
      } else {
        if (has_end) {
          next.travel = v.duration(previous_job.index(), v.end.value().index());
        }
      }
    } else {
      if (last_rank < route.size()) {
        next.latest = latest[last_rank];
        if (has_start) {
          current.location_index = v.start.value().index();
          next.travel = v.duration(v.start.value().index(),
                                   input.jobs[route[last_rank]].index());
        }
      } else {
        // Emptying the whole route is valid.
        return true;
      }
    }
  }

  // Determine break range between first_rank and last_rank.
  Index current_break = breaks_counts[first_rank] - breaks_at_rank[first_rank];
  const Index last_break = breaks_counts[last_rank];

  // Maintain current load while adding insertion range. Initial load
  // is lowered based on removed range.
  Amount current_load;

  if (check_max_load) {
    const auto previous_init_load =
      (route.empty()) ? input.zero_amount() : load_at_step(first_rank);
    assert(delivery_in_range(first_rank, last_rank) <= previous_init_load);
    const Amount delta_delivery =
      delivery - delivery_in_range(first_rank, last_rank);

    if (current_break != 0 &&
        !(delta_delivery <=
          fwd_smallest_breaks_load_margin[current_break - 1])) {
      return false;
    }

    current_load = previous_init_load + delta_delivery;
  }

  // Propagate earliest dates for all jobs and breaks in their
  // respective addition ranges.
  auto current_job = first_job;
  while (current_job != last_job || current_break != last_break) {
    if (current_job == last_job) {
      // Compute earliest end date for break after last inserted jobs.
      const auto& b = v.breaks[current_break];

      const auto b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
        return current.earliest <= tw.end;
      });

      if (b_tw == b.tws.end()) {
        // Break does not fit due to its time windows.
        return false;
      }

      if (check_max_load && !b.is_valid_for_load(current_load)) {
        // Break does not fit due to current load.
        return false;
      }

      if (current.earliest < b_tw->start) {
        if (const auto margin = b_tw->start - current.earliest;
            margin < next.travel) {
          next.travel -= margin;
        } else {
          next.travel = 0;
        }

        current.earliest = b_tw->start;
      }

      if (filter_max_transit_time) {
        trace.push_back({TraceEvent::Kind::BREAK,
                         current_break,
                         current.earliest,
                         b.service,
                         std::numeric_limits<Index>::max()});
      }
      current.earliest += b.service;

      ++current_break;
      continue;
    }

    // We still have jobs to go through.
    const auto& j = input.jobs[*current_job];

    if (current_break == last_break) {
      // Compute earliest end date for job after last inserted breaks.
      current.earliest += current.travel;

      const auto j_tw = std::ranges::find_if(j.tws, [&](const auto& tw) {
        return current.earliest <= tw.end;
      });
      if (j_tw == j.tws.end()) {
        return false;
      }
      const auto job_action_time =
        action_time_for(j, v_type, current.location_index);
      current.earliest = std::max(current.earliest, j_tw->start);
      if (filter_max_transit_time) {
        trace.push_back({TraceEvent::Kind::JOB,
                         *current_job,
                         current.earliest,
                         job_action_time,
                         j.index()});
      }
      current.location_index = j.index();
      current.earliest += job_action_time;

      if (check_max_load) {
        assert(j.delivery <= current_load);
        current_load += (j.pickup - j.delivery);
      }

      ++current_job;
      if (current_job != last_job) {
        // Account for travel time to next current job.
        current.travel =
          v.duration(j.index(), input.jobs[*current_job].index());
      }
      continue;
    }

    // We still have both jobs and breaks to go through, so decide on
    // ordering.
    const auto& b = v.breaks[current_break];
    const auto job_action_time =
      action_time_for(j, v_type, current.location_index);

    // Use next info after insertion range for ordering decision,
    // except if there are still jobs to insert after j, in which case
    // we might have tighter constraints.
    auto tighter_next = next;
    if (current_job + 1 < last_job) {
      const auto& next_j = input.jobs[*(current_job + 1)];

      assert(next.travel <= next.latest);
      tighter_next.latest =
        std::min(next.latest - next.travel, next_j.tws.back().end);
      tighter_next.travel = v.duration(j.index(), next_j.index());
    }

    const auto oc = order_choice(input,
                                 *current_job,
                                 job_action_time,
                                 b,
                                 current,
                                 tighter_next,
                                 current_load,
                                 check_max_load);

    if (!oc.add_job_first && !oc.add_break_first) {
      // Infeasible insertion.
      return false;
    }

    // Feasible insertion based on time windows, now update next end
    // time with given insertion choice.
    assert(oc.add_job_first xor oc.add_break_first);
    if (oc.add_break_first) {
      if (check_max_load && !b.is_valid_for_load(current_load)) {
        return false;
      }

      if (current.earliest < oc.b_tw->start) {
        if (const auto margin = oc.b_tw->start - current.earliest;
            margin < current.travel) {
          current.travel -= margin;
        } else {
          current.travel = 0;
        }

        current.earliest = oc.b_tw->start;
      }

      if (filter_max_transit_time) {
        trace.push_back({TraceEvent::Kind::BREAK,
                         current_break,
                         current.earliest,
                         b.service,
                         std::numeric_limits<Index>::max()});
      }
      current.earliest += b.service;

      ++current_break;
    }
    if (oc.add_job_first) {
      current.earliest =
        std::max(current.earliest + current.travel, oc.j_tw->start);
      if (filter_max_transit_time) {
        trace.push_back({TraceEvent::Kind::JOB,
                         *current_job,
                         current.earliest,
                         job_action_time,
                         j.index()});
      }
      current.location_index = j.index();
      current.earliest += job_action_time;

      if (check_max_load) {
        assert(j.delivery <= current_load);
        current_load += (j.pickup - j.delivery);
      }

      ++current_job;
      if (current_job != last_job) {
        // Account for travel time to next current job.
        current.travel =
          v.duration(j.index(), input.jobs[*current_job].index());
      }
    }
  }

  if (check_max_load && last_break < v.breaks.size()) {
    const auto previous_final_load =
      (route.empty()) ? input.zero_amount() : load_at_step(last_rank);

    const Amount delta_pickup = current_load - previous_final_load;

    if (!(delta_pickup <= bwd_smallest_breaks_load_margin[last_break])) {
      return false;
    }
  }

  if (last_rank < route.size() &&
      input.jobs[route[last_rank]].index() != current.location_index) {
    // There is a task right after replace range and setup time does
    // apply to it.
    const auto& j_after = input.jobs[route[last_rank]];
    auto new_action_time = j_after.setups[v_type] + j_after.services[v_type];
    if (action_time[last_rank] < new_action_time) {
      // Setup time did not previously apply to that task as action
      // time has increased. In that case the margin check for job at
      // last_rank may be OK in the return clause below, BUT shifting
      // earliest date for next task with new setup time may make it
      // not doable anymore.
      auto earliest_after = current.earliest + next.travel;
      const auto j_after_tw =
        std::ranges::find_if(j_after.tws, [&](const auto& tw) {
          return earliest_after <= tw.end;
        });
      if (j_after_tw == j_after.tws.end()) {
        return false;
      }
      earliest_after = std::max(earliest_after, j_after_tw->start);

      auto next_after = next_info(input, route[last_rank], last_rank + 1);

      // Go through breaks right after.
      Index break_rank =
        breaks_counts[last_rank + 1] - breaks_at_rank[last_rank + 1];

      for (Index r = 0; r < breaks_at_rank[last_rank + 1]; ++r, ++break_rank) {
        const auto& b = v.breaks[break_rank];

        earliest_after += new_action_time;

        const auto b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
          return earliest_after <= tw.end;
        });
        if (b_tw == b.tws.end()) {
          // Break does not fit due to its time windows.
          return false;
        }

        if (earliest_after < b_tw->start) {
          if (const auto margin = b_tw->start - earliest_after;
              margin < next_after.travel) {
            next_after.travel -= margin;
          } else {
            next_after.travel = 0;
          }

          earliest_after = b_tw->start;
        }

        new_action_time = v.breaks[break_rank].service;
      }

      if (earliest_after + new_action_time + next_after.travel >
          next_after.latest) {
        return false;
      }
    }
  }

  if (current.earliest + next.travel > next.latest) {
    return false;
  }
  if (!filter_max_transit_time) {
    return true;
  }

  // Candidate-path lower bound: proven-sound rejections only. Optimistic
  // acceptances pass through to the exact engine check below.
  if (!check_max_transit_time(input,
                              trace,
                              first_rank,
                              last_rank,
                              static_cast<Index>(inserted_job_count))) {
    return false;
  }

  // ASAP-witness short-circuit.
  //
  // When the ASAP schedule satisfies every constrained pair's cap, it is a
  // sound feasibility witness and the full engine call is unnecessary.
  //
  // ASAP departure from a stop = earliest_start + action_time.
  // Exact ASAP is available for:
  //   prefix stops (rank < first_rank): committed earliest[] / action_time[].
  //   trace JOB events:                ev.earliest / ev.action_time.
  // Suffix stops have no exact ASAP without an engine forward pass.
  //
  // Three soundness invariants that must hold for every future code change:
  //   I1. Witness may only ACCEPT (return true), never reject: any case that
  //       cannot be proven safe must fall through to the engine below.
  //   I2. Any constrained delivery at route rank >= last_rank (suffix) must
  //       set witness_eligible = false; there is no exact ASAP for suffix
  //       stops.
  //   I3. Any constrained pickup at route rank >= last_rank also disqualifies;
  //       its delivery is in the suffix too (pickup precedes delivery).
  {
    bool witness_eligible = true;
    bool asap_all_ok = true;

    // Returns the trace JOB event for the given input-job rank, or nullptr.
    const auto trace_ev = [&](const Index job_rank) -> const TraceEvent* {
      for (const auto& ev : trace) {
        if (ev.kind == TraceEvent::Kind::JOB && ev.rank == job_rank)
          return &ev;
      }
      return nullptr;
    };
    // Whether the given input-job rank appears in the candidate suffix.
    const auto in_suffix = [&](const Index job_rank) {
      for (Index r = last_rank; r < route.size(); ++r) {
        if (route[r] == job_rank)
          return true;
      }
      return false;
    };

    // Prefix pickups (rank < first_rank): delivery is in prefix or trace.
    for (Index r = 0; r < first_rank && witness_eligible; ++r) {
      const auto& j = input.jobs[route[r]];
      if (j.type != JOB_TYPE::PICKUP || !j.max_transit_time.has_value())
        continue;
      const Index del_rank = route[r] + 1; // input rank of paired delivery
      std::optional<Index> prefix_del_pos;
      for (Index dr = r + 1; dr < first_rank; ++dr) {
        if (route[dr] == del_rank) {
          prefix_del_pos = dr;
          break;
        }
      }
      if (prefix_del_pos.has_value()) {
        // Both stops in prefix. The committed route may be compliant only
        // through a delayed schedule; the witness certifies the ASAP
        // schedule, so an ASAP violation here means ASAP is not a valid
        // whole-route witness and the engine must decide (Invariant I1:
        // fall through, never reject).
        const Duration depart_P = earliest[r] + action_time[r];
        const Duration arrive_D = earliest[prefix_del_pos.value()];
        if (arrive_D < depart_P ||
            arrive_D - depart_P > j.max_transit_time.value())
          asap_all_ok = false;
        continue;
      }
      const TraceEvent* de = trace_ev(del_rank);
      if (!de) {
        if (in_suffix(del_rank)) {
          // Delivery in suffix (Invariant I2): must not accept via witness.
          witness_eligible = false;
          break;
        }
        // Delivery absent from the candidate (being removed): the pair is
        // incomplete and constrains nothing yet.
        continue;
      }
      // Pickup in prefix (exact committed), delivery in trace (exact ASAP).
      const Duration depart_P = earliest[r] + action_time[r];
      if (de->earliest < depart_P ||
          de->earliest - depart_P > j.max_transit_time.value())
        asap_all_ok = false;
    }

    // Trace pickups: delivery must also appear in the trace for exact ASAP.
    for (const auto& ev : trace) {
      if (!witness_eligible)
        break;
      if (ev.kind != TraceEvent::Kind::JOB)
        continue;
      const auto& j = input.jobs[ev.rank];
      if (j.type != JOB_TYPE::PICKUP || !j.max_transit_time.has_value())
        continue;
      const TraceEvent* de = trace_ev(ev.rank + 1);
      if (!de) {
        if (in_suffix(ev.rank + 1)) {
          // Delivery in suffix (Invariant I2): must not accept via witness.
          witness_eligible = false;
          break;
        }
        // Delivery absent from the candidate: the pair is incomplete and
        // constrains nothing yet. Half-pair insertion probes
        // (insertion_search evaluates the pickup alone before the full
        // pair) land here and keep the witness fast path.
        continue;
      }
      if (!asap_all_ok)
        continue; // Already failing; finish eligibility scan before returning.
      const Duration depart_P = ev.earliest + ev.action_time;
      if (de->earliest < depart_P ||
          de->earliest - depart_P > j.max_transit_time.value())
        asap_all_ok = false;
    }

    // Suffix pickups (Invariant I3): disqualify witness.
    if (witness_eligible) {
      for (Index r = last_rank;
           r < static_cast<Index>(route.size()) && witness_eligible;
           ++r) {
        const auto& j = input.jobs[route[r]];
        if (j.type == JOB_TYPE::PICKUP && j.max_transit_time.has_value() &&
            in_suffix(route[r] + 1))
          witness_eligible = false; // Invariant I3 (complete suffix pair).
      }
    }

    if (witness_eligible && asap_all_ok)
      return true; // ASAP schedule is a cap-compliant witness; engine skipped.
    // Invariant I1: no return false here; fall through to the engine.
  }

  // Exact cap-aware engine: eliminates the optimistic acceptances.
  return compute_cap_compliant_schedule(input,
                                        trace,
                                        first_rank,
                                        last_rank,
                                        static_cast<Index>(inserted_job_count))
    .has_value();
}

template <std::random_access_iterator Iter>
void TWRoute::replace(const Input& input,
                      const Amount& delivery,
                      const Iter first_job,
                      const Iter last_job,
                      const Index first_rank,
                      const Index last_rank) {
  assert(first_job <= last_job);
  assert(first_rank <= last_rank);

  const auto& v = input.vehicles[v_rank];

  PreviousInfo current(0, 0);
  NextInfo next(0, 0);

  // Value initialization differ whether there are actually jobs added
  // or not.
  if (first_job < last_job) {
    current = previous_info(input, *first_job, first_rank);
    next = next_info(input, *(last_job - 1), last_rank);
  } else {
    // This is actually a removal as no jobs are inserted.
    current.earliest = v_start;
    next.latest = v_end;

    if (first_rank > 0) {
      const auto& previous_job = input.jobs[route[first_rank - 1]];
      const auto previous_index = previous_job.index();
      current.earliest = earliest[first_rank - 1] + action_time[first_rank - 1];
      current.location_index = previous_index;

      if (last_rank < route.size()) {
        next.latest = latest[last_rank];
        next.travel =
          v.duration(previous_index, input.jobs[route[last_rank]].index());
      } else {
        if (has_end) {
          next.travel = v.duration(previous_index, v.end.value().index());
        }
      }
    } else {
      if (last_rank < route.size()) {
        next.latest = latest[last_rank];
        if (has_start) {
          current.location_index = v.start.value().index();
          next.travel = v.duration(v.start.value().index(),
                                   input.jobs[route[last_rank]].index());
        }
      }
    }
  }

  // Determine break range between first_rank and last_rank.
  Index current_break = breaks_counts[first_rank] - breaks_at_rank[first_rank];
  const Index last_break = breaks_counts[last_rank];

  // Maintain current load while adding insertion range. Initial load
  // is lowered based on removed range.
  const auto previous_init_load =
    (route.empty()) ? input.zero_amount() : load_at_step(first_rank);
  const auto previous_final_load =
    (route.empty()) ? input.zero_amount() : load_at_step(last_rank);
  assert(delivery_in_range(first_rank, last_rank) <= previous_init_load);
  const Amount delta_delivery =
    delivery - delivery_in_range(first_rank, last_rank);
  Amount current_load = previous_init_load + delta_delivery;

  // Update all break load margins prior to modified range.
  assert(current_break == 0 ||
         delta_delivery <= fwd_smallest_breaks_load_margin[current_break - 1]);
  for (std::size_t i = 0; i < current_break; ++i) {
    assert(delta_delivery <= fwd_smallest_breaks_load_margin[i]);

    // Manually decrement margin to avoid overflows that would end up
    // in a negative margin with a plain
    // fwd_smallest_breaks_load_margin[i] -= delta_delivery;
    for (std::size_t a = 0; a < delta_delivery.size(); ++a) {
      if ((-delta_delivery[a]) <= (std::numeric_limits<Capacity>::max() -
                                   fwd_smallest_breaks_load_margin[i][a])) {
        fwd_smallest_breaks_load_margin[i][a] -= delta_delivery[a];
      } else {
        fwd_smallest_breaks_load_margin[i][a] =
          std::numeric_limits<Capacity>::max();
      }
    }
  }

  // Maintain constrained_job_count_: decrement for constrained pickups
  // removed from [first_rank, last_rank), increment for those inserted.
  for (Index r = first_rank; r < last_rank; ++r) {
    const auto& rj = input.jobs[route[r]];
    if (rj.type == JOB_TYPE::PICKUP && rj.max_transit_time.has_value()) {
      --constrained_job_count_;
    }
  }
  for (auto it = first_job; it != last_job; ++it) {
    const auto& ij = input.jobs[*it];
    if (ij.type == JOB_TYPE::PICKUP && ij.max_transit_time.has_value()) {
      ++constrained_job_count_;
    }
  }

  unsigned previous_breaks_counts =
    (first_rank != 0) ? breaks_counts[first_rank - 1] : 0;

  // Adjust various vector sizes. Dummy inserted values and unmodified
  // old values in the insertion range will be overwritten below.
  const unsigned erase_count = last_rank - first_rank;
  const unsigned add_count = std::distance(first_job, last_job);

  // Update data structures. For earliest and latest dates, we need to
  // overwrite old values. Otherwise they may happen to be identical
  // to new computed values and stop propagation inside
  // fwd_update_earliest_from and bwd_update_latest_from below.
  if (add_count < erase_count) {
    auto to_erase = erase_count - add_count;
    route.erase(route.begin() + first_rank,
                route.begin() + first_rank + to_erase);
    earliest.erase(earliest.begin() + first_rank,
                   earliest.begin() + first_rank + to_erase);
    latest.erase(latest.begin() + first_rank,
                 latest.begin() + first_rank + to_erase);
    action_time.erase(action_time.begin() + first_rank,
                      action_time.begin() + first_rank + to_erase);
    breaks_at_rank.erase(breaks_at_rank.begin() + first_rank,
                         breaks_at_rank.begin() + first_rank + to_erase);
    breaks_counts.erase(breaks_counts.begin() + first_rank,
                        breaks_counts.begin() + first_rank + to_erase);

    std::fill(earliest.begin() + first_rank,
              earliest.begin() + first_rank + add_count,
              std::numeric_limits<Duration>::max());
    std::fill(latest.begin() + first_rank,
              latest.begin() + first_rank + add_count,
              0);
  } else {
    std::fill(earliest.begin() + first_rank,
              earliest.begin() + first_rank + erase_count,
              std::numeric_limits<Duration>::max());
    std::fill(latest.begin() + first_rank,
              latest.begin() + first_rank + erase_count,
              0);

    auto to_insert = add_count - erase_count;
    route.insert(route.begin() + first_rank, to_insert, 0);
    earliest.insert(earliest.begin() + first_rank, to_insert, 0);
    latest.insert(latest.begin() + first_rank, to_insert, 0);
    action_time.insert(action_time.begin() + first_rank, to_insert, 0);
    breaks_at_rank.insert(breaks_at_rank.begin() + first_rank, to_insert, 0);
    breaks_counts.insert(breaks_counts.begin() + first_rank, to_insert, 0);
  }

  // Current rank in route/earliest/latest/action_time vectors.
  Index current_job_rank = first_rank;
  unsigned breaks_before = 0;

  // Propagate earliest dates (and action times) for all jobs and
  // breaks in their respective addition ranges.
  auto current_job = first_job;
  while (current_job != last_job || current_break != last_break) {
    if (current_job == last_job) {
      // Compute earliest end date for break after last inserted jobs.
      const auto& b = v.breaks[current_break];
      assert(b.is_valid_for_load(current_load));

      const auto b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
        return current.earliest <= tw.end;
      });
      assert(b_tw != b.tws.end());

      if (current.earliest < b_tw->start) {
        if (const auto margin = b_tw->start - current.earliest;
            margin < next.travel) {
          next.travel -= margin;
        } else {
          next.travel = 0;
        }

        current.earliest = b_tw->start;
      }
      break_earliest[current_break] = current.earliest;

      current.earliest += b.service;

      // Update break max load margin.
      auto current_margin = (b.max_load.has_value())
                              ? b.max_load.value() - current_load
                              : utils::max_amount(input.get_amount_size());
      if (current_break == 0) {
        // New fwd_smallest_breaks_load_margin is solely based on this
        // break max_load.
        fwd_smallest_breaks_load_margin[current_break] = current_margin;
      } else {
        const auto& previous_margin =
          fwd_smallest_breaks_load_margin[current_break - 1];
        for (std::size_t i = 0; i < previous_margin.size(); ++i) {
          fwd_smallest_breaks_load_margin[current_break][i] =
            std::min(previous_margin[i], current_margin[i]);
        }
      }

      ++breaks_before;
      ++current_break;
      continue;
    }

    // We still have jobs to go through.
    const auto& j = input.jobs[*current_job];

    if (current_break == last_break) {
      // Compute earliest end date for job after last inserted breaks.
      current.earliest += current.travel;

      const auto j_tw = std::ranges::find_if(j.tws, [&](const auto& tw) {
        return current.earliest <= tw.end;
      });
      assert(j_tw != j.tws.end());

      current.earliest = std::max(current.earliest, j_tw->start);

      route[current_job_rank] = *current_job;
      earliest[current_job_rank] = current.earliest;
      breaks_at_rank[current_job_rank] = breaks_before;
      breaks_counts[current_job_rank] = previous_breaks_counts + breaks_before;

      action_time[current_job_rank] =
        action_time_for(j, v_type, current.location_index);
      current.location_index = j.index();
      current.earliest += action_time[current_job_rank];

      ++current_job_rank;
      previous_breaks_counts += breaks_before;
      breaks_before = 0;

      assert(j.delivery <= current_load);
      current_load += (j.pickup - j.delivery);

      ++current_job;
      if (current_job != last_job) {
        // Account for travel time to next current job.
        current.travel =
          v.duration(j.index(), input.jobs[*current_job].index());
      }
      continue;
    }

    // We still have both jobs and breaks to go through, so decide on
    // ordering.
    const auto& b = v.breaks[current_break];

    const auto job_action_time =
      action_time_for(j, v_type, current.location_index);

    // Use next info after insertion range for ordering decision,
    // except if there are still jobs to insert after j, in which case
    // we might have tighter constraints.
    auto tighter_next = next;
    if (current_job + 1 < last_job) {
      const auto& next_j = input.jobs[*(current_job + 1)];

      assert(next.travel <= next.latest);
      tighter_next.latest =
        std::min(next.latest - next.travel, next_j.tws.back().end);
      tighter_next.travel = v.duration(j.index(), next_j.index());
    }

    const auto oc = order_choice(input,
                                 *current_job,
                                 job_action_time,
                                 b,
                                 current,
                                 tighter_next,
                                 current_load);

    assert(oc.add_job_first xor oc.add_break_first);
    if (oc.add_break_first) {
      assert(b.is_valid_for_load(current_load));

      if (current.earliest < oc.b_tw->start) {
        if (const auto margin = oc.b_tw->start - current.earliest;
            margin < current.travel) {
          current.travel -= margin;
        } else {
          current.travel = 0;
        }

        current.earliest = oc.b_tw->start;
      }
      break_earliest[current_break] = current.earliest;

      current.earliest += b.service;

      // Update break max load margin.
      auto current_margin = (b.max_load.has_value())
                              ? b.max_load.value() - current_load
                              : utils::max_amount(input.get_amount_size());
      if (current_break == 0) {
        // New fwd_smallest_breaks_load_margin is solely based on this
        // break max_load.
        fwd_smallest_breaks_load_margin[current_break] = current_margin;
      } else {
        const auto& previous_margin =
          fwd_smallest_breaks_load_margin[current_break - 1];
        for (std::size_t i = 0; i < previous_margin.size(); ++i) {
          fwd_smallest_breaks_load_margin[current_break][i] =
            std::min(previous_margin[i], current_margin[i]);
        }
      }

      ++breaks_before;
      ++current_break;
    }
    if (oc.add_job_first) {
      current.earliest =
        std::max(current.earliest + current.travel, oc.j_tw->start);

      route[current_job_rank] = *current_job;
      earliest[current_job_rank] = current.earliest;
      breaks_at_rank[current_job_rank] = breaks_before;
      breaks_counts[current_job_rank] = previous_breaks_counts + breaks_before;

      action_time[current_job_rank] = job_action_time;
      current.earliest += job_action_time;
      current.location_index = j.index();

      ++current_job_rank;
      previous_breaks_counts += breaks_before;
      breaks_before = 0;

      assert(j.delivery <= current_load);
      current_load += (j.pickup - j.delivery);

      ++current_job;
      if (current_job != last_job) {
        // Account for travel time to next current job.
        current.travel =
          v.duration(j.index(), input.jobs[*current_job].index());
      }
    }
  }

  assert(current_job_rank == first_rank + add_count);

  // Update all break load margins after modified range.
  const Amount delta_pickup = current_load - previous_final_load;
  for (std::size_t i = last_break; i < v.breaks.size(); ++i) {
    assert(delta_pickup <= bwd_smallest_breaks_load_margin[i]);

    // Manually decrement margin to avoid overflows that would end up
    // in a negative margin with a plain
    // bwd_smallest_breaks_load_margin[i] -= delta_pickup;
    for (std::size_t a = 0; a < delta_pickup.size(); ++a) {
      if ((-delta_pickup[a]) <= (std::numeric_limits<Capacity>::max() -
                                 bwd_smallest_breaks_load_margin[i][a])) {
        bwd_smallest_breaks_load_margin[i][a] -= delta_pickup[a];
      } else {
        bwd_smallest_breaks_load_margin[i][a] =
          std::numeric_limits<Capacity>::max();
      }
    }
  }

  // Update remaining number of breaks due before next step.
  breaks_at_rank[current_job_rank] = breaks_before;
  assert(previous_breaks_counts + breaks_at_rank[current_job_rank] ==
         breaks_counts[current_job_rank]);

  if (!route.empty()) {
    auto valid_latest_date_rank = current_job_rank;
    auto valid_earliest_date_rank = 0;
    const bool replace_last_jobs = (current_job_rank == route.size());
    bool do_update_last_latest_date = false;

    if (replace_last_jobs) {
      earliest_end = current.earliest + next.travel;

      do_update_last_latest_date = true;
      valid_latest_date_rank = route.size() - 1;
    } else {
      // current_job_rank is the rank of the first non-replaced job.
      const auto& j = input.jobs[route[current_job_rank]];

      const auto new_action_time =
        action_time_for(j, v_type, current.location_index);
      assert(action_time[current_job_rank] == j.services[v_type] ||
             action_time[current_job_rank] ==
               j.services[v_type] + j.setups[v_type]);

      const bool current_action_time_changed =
        (new_action_time != action_time[current_job_rank]);
      if (current_action_time_changed) {
        // Due to removal, total time spent at first non-replaced
        // task changed, so we need its latest date updated, either
        // directly if at the end of the route, either by going
        // backward from next task (if any).
        if (current_job_rank == route.size() - 1) {
          do_update_last_latest_date = true;
        } else {
          valid_latest_date_rank = current_job_rank + 1;
          // We need to update latest dates for the previous jobs
          // **before** current_job_rank, but bwd_update_latest_from
          // has a stop criterion for propagation that will trigger if
          // latest date happens to not change at current_job_rank.
          latest[current_job_rank] = 0;
        }
      }

      if (current_job_rank == 0) {
        // First jobs in route have been erased and not replaced, so
        // update new first job earliest date and action time.
        current.earliest += next.travel;
        const auto j_tw = std::ranges::find_if(j.tws, [&](const auto& tw) {
          return current.earliest <= tw.end;
        });
        assert(j_tw != j.tws.end());

        earliest[0] = std::max(current.earliest, j_tw->start);
        assert(earliest[0] <= latest[0] ||
               (current_action_time_changed && latest[current_job_rank] == 0));

        action_time[0] = new_action_time;
      } else {
        valid_earliest_date_rank = current_job_rank - 1;
        if (current_action_time_changed) {
          // We need to update earliest dates for the following jobs
          // **after** current_job_rank, but fwd_update_earliest_from
          // has a stop criterion for propagation that will trigger if
          // earliest date happens to not change at current_job_rank.
          earliest[current_job_rank] = std::numeric_limits<Duration>::max();
        }
      }
    }

    if (!replace_last_jobs) {
      // Update earliest dates forward.
      fwd_update_action_time_from(input, valid_earliest_date_rank);
      fwd_update_earliest_from(input, valid_earliest_date_rank);
    }

    if (do_update_last_latest_date) {
      update_last_latest_date(input);
    }
    // Update latest dates backward.
    bwd_update_latest_from(input, valid_latest_date_rank);
  }

  update_amounts(input);

  // Propagate fwd/bwd_smallest_breaks_load_margin if required.
  if (last_break < v.breaks.size()) {
    fwd_update_breaks_load_margin_from(input, current_job_rank);
  }
  if (last_break > 0) {
    bwd_update_breaks_load_margin_from(input, current_job_rank);
  }
}

template bool
TWRoute::is_valid_addition_for_tw(const Input& input,
                                  const Amount& delivery,
                                  const std::vector<Index>::iterator first_job,
                                  const std::vector<Index>::iterator last_job,
                                  const Index first_rank,
                                  const Index last_rank,
                                  bool check_max_load) const;

template bool TWRoute::is_valid_addition_for_tw(
  const Input& input,
  const Amount& delivery,
  const std::vector<Index>::reverse_iterator first_job,
  const std::vector<Index>::reverse_iterator last_job,
  const Index first_rank,
  const Index last_rank,
  bool check_max_load) const;

template bool TWRoute::is_valid_addition_for_tw(
  const Input& input,
  const Amount& delivery,
  const std::array<Index, 1>::const_iterator first_job,
  const std::array<Index, 1>::const_iterator last_job,
  const Index first_rank,
  const Index last_rank,
  bool check_max_load) const;

template bool TWRoute::is_valid_addition_for_tw(
  const Input& input,
  const Amount& delivery,
  const std::vector<Index>::const_iterator first_job,
  const std::vector<Index>::const_iterator last_job,
  const Index first_rank,
  const Index last_rank,
  bool check_max_load) const;

template void TWRoute::replace(const Input& input,
                               const Amount& delivery,
                               const std::vector<Index>::iterator first_job,
                               const std::vector<Index>::iterator last_job,
                               const Index first_rank,
                               const Index last_rank);
template void
TWRoute::replace(const Input& input,
                 const Amount& delivery,
                 const std::vector<Index>::const_iterator first_job,
                 const std::vector<Index>::const_iterator last_job,
                 const Index first_rank,
                 const Index last_rank);
template void
TWRoute::replace(const Input& input,
                 const Amount& delivery,
                 const std::vector<Index>::reverse_iterator first_job,
                 const std::vector<Index>::reverse_iterator last_job,
                 const Index first_rank,
                 const Index last_rank);

template void
TWRoute::replace(const Input& input,
                 const Amount& delivery,
                 const std::array<Index, 1>::const_iterator first_job,
                 const std::array<Index, 1>::const_iterator last_job,
                 const Index first_rank,
                 const Index last_rank);

} // namespace vroom
