/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/tw_route.h"
#include "utils/exception.h"

namespace vroom {

TWRoute::TWRoute(const Input& input, Index v)
  : RawRoute(input, v),
    v_start(input.vehicles[v].tw.start),
    v_end(input.vehicles[v].tw.end),
    breaks_at_rank({static_cast<unsigned>(input.vehicles[v].breaks.size())}),
    breaks_counts({static_cast<unsigned>(input.vehicles[v].breaks.size())}),
    break_earliest(input.vehicles[v].breaks.size()),
    break_latest(input.vehicles[v].breaks.size()),
    break_tw_ranks(input.vehicles[v].breaks.size()) {
  std::string break_error = "Inconsistent breaks for vehicle " +
                            std::to_string(input.vehicles[v].id) + ".";

  const auto& breaks = input.vehicles[v].breaks;

  Duration previous_earliest = v_start;

  for (Index i = 0; i < breaks.size(); ++i) {
    const auto& b = breaks[i];
    auto tw_candidate =
      std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
        return previous_earliest <= tw.end;
      });

    if (tw_candidate == b.tws.end()) {
      throw Exception(ERROR::INPUT, break_error);
    }

    break_earliest[i] = std::max(previous_earliest, tw_candidate->start);
    break_tw_ranks[i] = std::distance(b.tws.begin(), tw_candidate);

    previous_earliest = break_earliest[i] + b.service;
  }

  Duration next_latest = v_end;
  for (Index r_i = 0; r_i < breaks.size(); ++r_i) {
    Index i = breaks.size() - 1 - r_i;
    const auto& b = breaks[i];

    if (next_latest < b.service) {
      throw Exception(ERROR::INPUT, break_error);
    }

    next_latest -= b.service;
    next_latest = std::min(next_latest, b.tws[break_tw_ranks[i]].end);

    break_latest[i] = next_latest;

    if (break_latest[i] < break_earliest[i]) {
      throw Exception(ERROR::INPUT, break_error);
    }
  }
}

Duration TWRoute::new_earliest_candidate(const Input& input,
                                         Index job_rank,
                                         Index rank) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration previous_earliest = v_start;
  Duration previous_service = 0;
  Duration previous_travel = 0;
  if (rank > 0) {
    const auto& previous_job = input.jobs[route[rank - 1]];
    previous_earliest = earliest[rank - 1];
    previous_service = previous_job.service;
    previous_travel = m[previous_job.index()][j.index()];
  } else {
    if (has_start) {
      previous_travel = m[v.start.get().index()][j.index()];
    }
  }

  return previous_earliest + previous_service + previous_travel;
}

Duration TWRoute::new_latest_candidate(const Input& input,
                                       Index job_rank,
                                       Index rank) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration next_latest = v_end;
  Duration next_travel = 0;
  if (rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[rank];
    next_travel = m[j.index()][input.jobs[route[rank]].index()];
  }

  assert(j.service + next_travel <= next_latest);
  return next_latest - j.service - next_travel;
}

Duration TWRoute::new_earliest_candidate(const Input& input,
                                         const Index job_rank,
                                         const Index rank,
                                         const Index break_position) const {
  assert(break_position <= breaks_at_rank[rank]);

  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration previous_earliest = v_start;
  Duration previous_service = 0;
  Duration previous_travel = 0;
  if (rank > 0) {
    const auto& previous_job = input.jobs[route[rank - 1]];
    previous_earliest = earliest[rank - 1];
    previous_service = previous_job.service;
    previous_travel = m[previous_job.index()][j.index()];
  } else {
    if (has_start) {
      previous_travel = m[v.start.get().index()][j.index()];
    }
  }

  if (break_position > 0) {
    // Some breaks are before insertion, use rank of previous break in
    // vehicle breaks.
    assert(breaks_at_rank[rank] + 1 <= breaks_counts[rank] + break_position);
    Index break_rank =
      (breaks_counts[rank] + break_position) - (breaks_at_rank[rank] + 1);

    previous_earliest = break_earliest[break_rank];
    previous_service = v.breaks[break_rank].service;

    // Compute part of the travel time from last job that remains to
    // be done.
    auto breaks_travel_margin =
      std::accumulate(breaks_travel_margin_before.begin() + break_rank,
                      breaks_travel_margin_before.begin() + break_rank +
                        break_position,
                      static_cast<Duration>(0),
                      [&](auto sum, const auto& m) { return sum + m; });

    if (previous_travel <= breaks_travel_margin) {
      // Margins before breaks can eat up all travel time.
      previous_travel = 0;
    } else {
      // Remove travel time share that can be done before previous
      // breaks.
      previous_travel -= breaks_travel_margin;
    }
  }

  return previous_earliest + previous_service + previous_travel;
}

Margin TWRoute::new_latest_candidate(const Input& input,
                                     const Index job_rank,
                                     const Index rank,
                                     const Index break_position) const {
  assert(break_position <= breaks_at_rank[rank]);

  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Margin next_latest = v_end;
  Duration next_travel = 0;
  if (rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[rank];
    next_travel = m[j.index()][input.jobs[route[rank]].index()];
  }

  if (break_position < breaks_at_rank[rank]) {
    // Some breaks are after insertion, use rank of next break in
    // vehicle breaks.
    assert(breaks_at_rank[rank] <= breaks_counts[rank] + break_position);
    Index break_rank =
      (breaks_counts[rank] + break_position) - breaks_at_rank[rank];

    next_latest = break_latest[break_rank];

    // Compute part of the travel time from last job that remains to
    // be done.
    auto breaks_travel_margin =
      std::accumulate(breaks_travel_margin_after.begin() + break_rank,
                      breaks_travel_margin_after.begin() + breaks_counts[rank],
                      static_cast<Duration>(0),
                      [&](auto sum, const auto& m) { return sum + m; });

    if (next_travel <= breaks_travel_margin) {
      // Margins after breaks can eat up all travel time.
      next_travel = 0;
    } else {
      // Remove travel time share that can be done after next breaks.
      next_travel -= breaks_travel_margin;
    }
  }

  return next_latest - static_cast<Margin>(j.service + next_travel);
}

void TWRoute::fwd_update_earliest_from(const Input& input, Index rank) {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  Duration current_earliest = earliest[rank];
  Duration previous_service = input.jobs[route[rank]].service;

  for (Index i = rank + 1; i < route.size(); ++i) {
    const auto& next_j = input.jobs[route[i]];
    Duration remaining_travel_time =
      m[input.jobs[route[i - 1]].index()][next_j.index()];

    // Update earliest dates and margins for breaks.
    assert(breaks_at_rank[i] <= breaks_counts[i]);
    Index break_rank = breaks_counts[i] - breaks_at_rank[i];

    for (Index b = 0; b < breaks_at_rank[i]; ++b, ++break_rank) {
      current_earliest += previous_service;

      const auto& break_TW =
        v.breaks[break_rank].tws[break_tw_ranks[break_rank]];
      if (current_earliest < break_TW.start) {
        auto margin = break_TW.start - current_earliest;
        breaks_travel_margin_before[break_rank] = margin;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          remaining_travel_time = 0;
        }

        current_earliest = break_TW.start;
      } else {
        breaks_travel_margin_before[break_rank] = 0;
      }

      previous_service = v.breaks[break_rank].service;
    }

    // Back to the job after breaks.
    current_earliest += previous_service + remaining_travel_time;

    const auto& next_TW = next_j.tws[tw_ranks[i]];
    current_earliest = std::max(current_earliest, next_TW.start);

    assert(current_earliest <= latest[i]);
    if (current_earliest == earliest[i]) {
      break;
    }

    earliest[i] = current_earliest;
  }
}

void TWRoute::bwd_update_latest_from(const Input& input, Index rank) {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  Duration current_latest = latest[rank];

  for (Index next_i = rank; next_i > 0; --next_i) {
    const auto& previous_j = input.jobs[route[next_i - 1]];
    Duration remaining_travel_time =
      m[previous_j.index()][input.jobs[route[next_i]].index()];

    // Update latest dates and margins for breaks.
    assert(breaks_at_rank[next_i] <= breaks_counts[next_i]);
    Index break_rank = breaks_counts[next_i];

    for (Index b = 0; b < breaks_at_rank[next_i]; ++b) {
      --break_rank;
      assert(v.breaks[break_rank].service <= current_latest);
      current_latest -= v.breaks[break_rank].service;

      const auto& break_TW =
        v.breaks[break_rank].tws[break_tw_ranks[break_rank]];
      if (break_TW.end < current_latest) {
        auto margin = current_latest - break_TW.end;
        breaks_travel_margin_after[break_rank] = margin;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          remaining_travel_time = 0;
        }

        current_latest = break_TW.end;
      } else {
        breaks_travel_margin_after[break_rank] = 0;
      }
    }

    // Back to the job after breaks.
    auto gap = previous_j.service + remaining_travel_time;
    assert(gap <= current_latest);
    current_latest -= gap;

    const auto& previous_TW = previous_j.tws[tw_ranks[next_i - 1]];
    current_latest = std::min(current_latest, previous_TW.end);

    assert(earliest[next_i - 1] <= current_latest);
    if (current_latest == latest[next_i - 1]) {
      break;
    }

    latest[next_i - 1] = current_latest;
  }
}

Margin TWRoute::addition_margin(const Input& input,
                                const Index job_rank,
                                const Index rank,
                                const Index break_position) const {
  const auto& j = input.jobs[job_rank];

  Duration job_earliest =
    new_earliest_candidate(input, job_rank, rank, break_position);

  if (j.tws.back().end < job_earliest) {
    // Early abort if we're after the latest deadline for current job.
    return std::numeric_limits<Margin>::min();
  }

  auto tw_candidate =
    std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
      return job_earliest <= tw.end;
    });

  // The situation where there is no TW candidate has been previously
  // filtered by early abort above.
  assert(tw_candidate != j.tws.end());
  job_earliest = std::max(job_earliest, tw_candidate->start);

  // // TODO test this early abort option.
  // if (job_earliest > ((rank == route.size()) ? v_end: latest[rank])) {
  //   return std::numeric_limits<Margin>::min();
  // }

  auto job_latest = new_latest_candidate(input, job_rank, rank, break_position);

  job_latest = std::min(job_latest, static_cast<Margin>(tw_candidate->end));

  return job_latest - static_cast<Margin>(job_earliest);
}

bool TWRoute::is_valid_addition_for_tw(const Input& input,
                                       const Index job_rank,
                                       const Index rank) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration job_earliest = new_earliest_candidate(input, job_rank, rank);

  if (j.tws.back().end < job_earliest) {
    // Early abort if we're after the latest deadline for current job.
    return false;
  }

  Duration next_latest = v_end;
  Duration next_travel = 0;
  if (rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[rank];
    next_travel = m[j.index()][input.jobs[route[rank]].index()];
  }

  bool valid = job_earliest + j.service + next_travel <= next_latest;

  if (valid) {
    Duration new_latest = next_latest - j.service - next_travel;

    auto overlap_candidate =
      std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
        return job_earliest <= tw.end;
      });

    // The situation where there is no TW candidate should have been
    // previously filtered by early abort above.
    assert(overlap_candidate != j.tws.end());
    valid = overlap_candidate->start <= new_latest;
  }

  return valid;
}

bool TWRoute::is_valid_addition_for_tw(const Input& input,
                                       const Index job_rank,
                                       const Index rank,
                                       Index& break_position) const {
  Margin best_margin = std::numeric_limits<Margin>::min();

  for (unsigned break_pos = 0; break_pos <= breaks_at_rank[rank]; ++break_pos) {
    auto current_margin = addition_margin(input, job_rank, rank, break_pos);
    if (current_margin > best_margin) {
      break_position = break_pos;
      best_margin = current_margin;
    }
  }

  return best_margin >= 0;
}

template <class InputIterator>
bool TWRoute::is_valid_addition_for_tw(const Input& input,
                                       InputIterator first_job,
                                       InputIterator last_job,
                                       const Index first_rank,
                                       const Index last_rank) const {
  if (first_job == last_job) {
    return true;
  }
  assert(first_rank <= last_rank);

  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  // Handle first job earliest date.
  const auto& first_j = input.jobs[*first_job];
  Duration job_earliest = new_earliest_candidate(input, *first_job, first_rank);

  auto tw_candidate =
    std::find_if(first_j.tws.begin(), first_j.tws.end(), [&](const auto& tw) {
      return job_earliest <= tw.end;
    });

  if (tw_candidate == first_j.tws.end()) {
    // Early abort if we're after the latest deadline for current job.
    return false;
  }
  job_earliest = std::max(job_earliest, tw_candidate->start);

  // Propagate earliest dates for all jobs in the addition range.
  auto next_job = first_job + 1;
  while (next_job != last_job) {
    const auto& first_j = input.jobs[*first_job];
    const auto& next_j = input.jobs[*next_job];
    job_earliest += first_j.service + m[first_j.index()][next_j.index()];

    tw_candidate =
      std::find_if(next_j.tws.begin(), next_j.tws.end(), [&](const auto& tw) {
        return job_earliest <= tw.end;
      });
    if (tw_candidate == next_j.tws.end()) {
      // Early abort if we're after the latest deadline for current job.
      return false;
    }
    job_earliest = std::max(job_earliest, tw_candidate->start);

    ++first_job;
    ++next_job;
  }

  // Check latest date for last inserted job.
  const auto& j = input.jobs[*first_job];
  Duration next_latest = v_end;
  Duration next_travel = 0;
  if (last_rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[last_rank];
    next_travel = m[j.index()][input.jobs[route[last_rank]].index()];
  }

  return job_earliest + j.service + next_travel <= next_latest;
}

void TWRoute::add(const Input& input, const Index job_rank, const Index rank) {
  assert(rank <= route.size());

  Duration job_earliest = new_earliest_candidate(input, job_rank, rank);
  Duration job_latest = new_latest_candidate(input, job_rank, rank);

  // Pick first compatible TW.
  const auto& tws = input.jobs[job_rank].tws;
  auto candidate = std::find_if(tws.begin(), tws.end(), [&](const auto& tw) {
    return job_earliest <= tw.end;
  });
  assert(candidate != tws.end());

  job_earliest = std::max(job_earliest, candidate->start);
  job_latest = std::min(job_latest, candidate->end);

  tw_ranks.insert(tw_ranks.begin() + rank,
                  std::distance(tws.begin(), candidate));

  // Needs to be done after TW stuff as new_[earliest|latest] rely on
  // route size before addition ; but before earliest/latest date
  // propagation that rely on route structure after addition.
  route.insert(route.begin() + rank, job_rank);

  // Update earliest/latest date for new job, then propagate
  // constraints.
  earliest.insert(earliest.begin() + rank, job_earliest);
  latest.insert(latest.begin() + rank, job_latest);

  fwd_update_earliest_from(input, rank);
  bwd_update_latest_from(input, rank);

  update_amounts(input);
}

bool TWRoute::is_fwd_valid_removal(const Input& input,
                                   const Index rank,
                                   const unsigned count) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  // Check forward validity as of first non-removed job.
  Index current_rank = rank + count;
  if (current_rank == route.size()) {
    if (rank == 0 or !has_end) {
      // Emptying a route or removing the end of a route with no
      // vehicle end is always OK.
      return true;
    } else {
      // Otherwise check for end date validity.
      const auto& new_last_job = input.jobs[route[rank - 1]];
      return earliest[rank - 1] + new_last_job.service +
               m[new_last_job.index()][v.end.get().index()] <=
             v_end;
    }
  }

  const auto current_index = input.jobs[route[current_rank]].index();

  Duration previous_earliest = v_start;
  Duration previous_service = 0;
  Duration previous_travel = 0;

  if (rank > 0) {
    const auto& previous_job = input.jobs[route[rank - 1]];
    previous_earliest = earliest[rank - 1];
    previous_service = previous_job.service;
    previous_travel = m[previous_job.index()][current_index];
  } else {
    if (has_start) {
      previous_travel = m[v.start.get().index()][current_index];
    }
  }

  Duration job_earliest =
    previous_earliest + previous_service + previous_travel;

  while (current_rank < route.size()) {
    if (job_earliest <= earliest[current_rank]) {
      return true;
    }
    if (latest[current_rank] < job_earliest) {
      return false;
    }

    // Pick first compatible TW to keep on checking for next jobs.
    const auto& current_job = input.jobs[route[current_rank]];
    const auto& tws = current_job.tws;
    auto candidate = std::find_if(tws.begin(), tws.end(), [&](const auto& tw) {
      return job_earliest <= tw.end;
    });
    assert(candidate != tws.end());
    job_earliest = std::max(job_earliest, candidate->start);
    job_earliest += current_job.service;
    if (current_rank < route.size() - 1) {
      job_earliest +=
        m[current_job.index()][input.jobs[route[current_rank + 1]].index()];
    } else {
      if (has_end) {
        job_earliest += m[current_job.index()][v.end.get().index()];
      }
    }

    ++current_rank;
  }

  return job_earliest <= v_end;
}

bool TWRoute::is_bwd_valid_removal(const Input& input,
                                   const Index rank,
                                   const unsigned count) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  if (rank == 0) {
    if (count == route.size() or !has_start) {
      // Emptying a route or removing the start of a route with no
      // vehicle start is always OK.
      return true;
    }

    // Check for start date validity.
    const auto new_first_index = input.jobs[route[count]].index();
    return v_start + m[v.start.get().index()][new_first_index] <= latest[count];
  }

  // Check backward validity as of first non-removed job.
  Index current_rank = rank - 1;
  Index current_index = input.jobs[route[current_rank]].index();

  Index next_rank = rank + count;
  Duration next_latest = v_end;
  Duration next_travel = 0;

  if (next_rank == route.size()) {
    if (has_end) {
      next_travel = m[current_index][v.end.get().index()];
    }
  } else {
    const auto& next_job = input.jobs[route[next_rank]];
    next_latest = latest[next_rank];
    next_travel = m[current_index][next_job.index()];
  }

  while (current_rank > 0) {
    Duration current_service = input.jobs[route[current_rank]].service;
    if (latest[current_rank] + current_service + next_travel <= next_latest) {
      return true;
    }
    if (next_latest < earliest[current_rank] + current_service + next_travel) {
      return false;
    }

    // Update new latest date for current job. No underflow due to
    // previous check.
    next_latest = next_latest - current_service - next_travel;
    assert(input.jobs[route[current_rank]].tws[tw_ranks[current_rank]].contains(
      next_latest));

    if (current_rank > 0) {
      auto previous_index = input.jobs[route[current_rank - 1]].index();
      next_travel = m[previous_index][current_index];
      current_index = previous_index;
    }

    --current_rank;
  }

  next_travel = 0;
  if (has_start) {
    next_travel = m[v.start.get().index()][current_index];
  }
  return v_start + next_travel <= next_latest;
}

bool TWRoute::is_valid_removal(const Input& input,
                               const Index rank,
                               const unsigned count) const {
  assert(!route.empty());
  assert(rank + count <= route.size());

  return is_fwd_valid_removal(input, rank, count) and
         is_bwd_valid_removal(input, rank, count);
}

void TWRoute::remove(const Input& input,
                     const Index rank,
                     const unsigned count) {
  assert(rank + count <= route.size());

  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  bool empty_route = rank == 0 and count == route.size();

  // Find out where to start updates for earliest/latest dates in new
  // route. fwd/bwd_ranks are relative to the route *after* the erase
  // operations below.
  auto fwd_rank = rank - 1;
  auto bwd_rank = rank;
  if (!empty_route) {
    if (rank == 0) {
      fwd_rank = 0;
      // Update earliest date for new first job.
      const auto& new_first_j = input.jobs[route[count]];
      Duration start_earliest = v_start;
      if (has_start) {
        start_earliest += m[v.start.get().index()][new_first_j.index()];
      }
      const auto& new_first_TW = new_first_j.tws[tw_ranks[count]];
      earliest[count] = std::max(start_earliest, new_first_TW.start);
    }

    if (rank + count == route.size()) {
      // Implicitly rank > 0 because !empty_route.
      bwd_rank = rank - 1;
      // Update earliest date for new last job.
      const auto& new_last_j = input.jobs[route[bwd_rank]];
      Duration end_latest = v_end;
      if (has_end) {
        auto gap =
          new_last_j.service + m[new_last_j.index()][v.end.get().index()];
        assert(gap <= v_end);
        end_latest -= gap;
      }
      const auto& new_last_TW = new_last_j.tws[tw_ranks[bwd_rank]];
      latest[bwd_rank] = std::min(end_latest, new_last_TW.end);
    }
  }

  route.erase(route.begin() + rank, route.begin() + rank + count);
  earliest.erase(earliest.begin() + rank, earliest.begin() + rank + count);
  latest.erase(latest.begin() + rank, latest.begin() + rank + count);
  tw_ranks.erase(tw_ranks.begin() + rank, tw_ranks.begin() + rank + count);

  // Update earliest/latest dates.
  if (!empty_route) {
    fwd_update_earliest_from(input, fwd_rank);
    bwd_update_latest_from(input, bwd_rank);
  }

  update_amounts(input);
}

template <class InputIterator>
void TWRoute::replace(const Input& input,
                      InputIterator first_job,
                      InputIterator last_job,
                      const Index first_rank,
                      const Index last_rank) {
  assert(first_rank <= last_rank);

  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  // Number of items to erase and add.
  unsigned erase_count = last_rank - first_rank;
  unsigned add_count = std::distance(first_job, last_job);

  // Start updates for earliest/latest dates in new route.
  Duration current_earliest;
  if (first_rank > 0) {
    current_earliest = earliest[first_rank - 1];
  } else {
    // Use earliest date for new first job.
    const auto& new_first_j = input.jobs[*first_job];
    current_earliest = v_start;
    if (has_start) {
      current_earliest += m[v.start.get().index()][new_first_j.index()];
    }
  }

  // Replacing values in route that are to be removed anyway and
  // updating earliest/tw_rank along the way.
  auto insert_rank = first_rank;
  while (first_job != last_job and insert_rank != last_rank) {
    route[insert_rank] = *first_job;
    const auto& current_j = input.jobs[route[insert_rank]];

    if (insert_rank > 0) {
      const auto& previous_j = input.jobs[route[insert_rank - 1]];
      current_earliest +=
        previous_j.service + m[previous_j.index()][current_j.index()];
    }
    auto tw_candidate =
      std::find_if(current_j.tws.begin(),
                   current_j.tws.end(),
                   [&](const auto& tw) { return current_earliest <= tw.end; });
    assert(tw_candidate != current_j.tws.end());

    current_earliest = std::max(current_earliest, tw_candidate->start);
    earliest[insert_rank] = current_earliest;
    tw_ranks[insert_rank] = std::distance(current_j.tws.begin(), tw_candidate);
    // Invalidated latest values.
    latest[insert_rank] = 0;

    ++first_job;
    ++insert_rank;
  }

  // Perform remaining insert/erase in route and resize other vectors
  // accordingly.
  if (add_count < erase_count) {
    assert(insert_rank < last_rank and first_job == last_job);
    route.erase(route.begin() + insert_rank, route.begin() + last_rank);

    auto to_erase = erase_count - add_count;
    earliest.erase(earliest.begin() + insert_rank,
                   earliest.begin() + insert_rank + to_erase);
    latest.erase(latest.begin() + insert_rank,
                 latest.begin() + insert_rank + to_erase);
    tw_ranks.erase(tw_ranks.begin() + insert_rank,
                   tw_ranks.begin() + insert_rank + to_erase);
  }
  if (erase_count < add_count) {
    assert(first_job != last_job and insert_rank == last_rank);
    route.insert(route.begin() + insert_rank, first_job, last_job);

    // Inserted values don't matter, they will be overwritten below or
    // during [fwd|bwd]_update_latest_from below.
    auto to_insert = add_count - erase_count;
    earliest.insert(earliest.begin() + insert_rank, to_insert, 0);
    latest.insert(latest.begin() + insert_rank, to_insert, 0);
    tw_ranks.insert(tw_ranks.begin() + insert_rank, to_insert, 0);
  }

  // Keep updating for remaining added jobs.
  unsigned last_add_rank = insert_rank;
  if (erase_count < add_count) {
    last_add_rank += add_count - erase_count;
  }
  while (insert_rank < last_add_rank) {
    const auto& current_j = input.jobs[route[insert_rank]];

    if (insert_rank > 0) {
      const auto& previous_j = input.jobs[route[insert_rank - 1]];
      current_earliest +=
        previous_j.service + m[previous_j.index()][current_j.index()];
    }
    auto tw_candidate =
      std::find_if(current_j.tws.begin(),
                   current_j.tws.end(),
                   [&](const auto& tw) { return current_earliest <= tw.end; });
    assert(tw_candidate != current_j.tws.end());

    current_earliest = std::max(current_earliest, tw_candidate->start);
    earliest[insert_rank] = current_earliest;
    tw_ranks[insert_rank] = std::distance(current_j.tws.begin(), tw_candidate);
    // Invalidated latest values.
    latest[insert_rank] = 0;

    ++insert_rank;
  }

  if (!route.empty()) {
    if (add_count == 0 and insert_rank == 0) {
      // First jobs in route have been erased and not replaced, so
      // update new first job earliest date.
      const auto& current_j = input.jobs[route[insert_rank]];
      current_earliest = v_start;
      if (has_start) {
        current_earliest += m[v.start.get().index()][current_j.index()];
      }

      auto tw_candidate = std::find_if(current_j.tws.begin(),
                                       current_j.tws.end(),
                                       [&](const auto& tw) {
                                         return current_earliest <= tw.end;
                                       });
      assert(tw_candidate != current_j.tws.end());

      earliest[insert_rank] = std::max(current_earliest, tw_candidate->start);
      tw_ranks[insert_rank] =
        std::distance(current_j.tws.begin(), tw_candidate);
      // Invalidated latest values.
      latest[insert_rank] = 0;
      ++insert_rank;
    }

    // If valid, insert_rank is the rank of the first job with known
    // latest date.
    if (insert_rank == route.size()) {
      // Replacing last job(s) in route, so update earliest and latest
      // date for new last job based on relevant time-window.
      --insert_rank;
      const auto& new_last_j = input.jobs[route[insert_rank]];

      Duration end_latest = v_end;
      if (has_end) {
        auto gap =
          new_last_j.service + m[new_last_j.index()][v.end.get().index()];
        assert(gap <= v_end);
        end_latest -= gap;
      }
      latest[insert_rank] =
        std::min(end_latest, new_last_j.tws[tw_ranks[insert_rank]].end);
    } else {
      // Update earliest dates forward in end of route.
      fwd_update_earliest_from(input, insert_rank - 1);
    }

    // Update latest dates backward.
    bwd_update_latest_from(input, insert_rank);
  }

  update_amounts(input);
}

template bool
TWRoute::is_valid_addition_for_tw(const Input& input,
                                  std::vector<Index>::iterator first_job,
                                  std::vector<Index>::iterator last_job,
                                  const Index first_rank,
                                  const Index last_rank) const;
template bool TWRoute::is_valid_addition_for_tw(
  const Input& input,
  std::vector<Index>::reverse_iterator first_job,
  std::vector<Index>::reverse_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;

template void TWRoute::replace(const Input& input,
                               std::vector<Index>::iterator first_job,
                               std::vector<Index>::iterator last_job,
                               const Index first_rank,
                               const Index last_rank);
template void TWRoute::replace(const Input& input,
                               std::vector<Index>::reverse_iterator first_job,
                               std::vector<Index>::reverse_iterator last_job,
                               const Index first_rank,
                               const Index last_rank);

} // namespace vroom
