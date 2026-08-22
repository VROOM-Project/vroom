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

  // Margin filter: proven-sound rejections only. A move that survives this
  // may still break a cap; nothing here accepts a move as compliant.
  return check_max_transit_time(input,
                                trace,
                                first_rank,
                                last_rank,
                                static_cast<Index>(inserted_job_count));
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
