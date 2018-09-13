/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/tw_route.h"

tw_route::tw_route(const input& input, index_t i)
  : _input(input),
    m(_input.get_matrix()),
    v(_input._vehicles[i]),
    has_start(_input._vehicles[i].has_start()),
    has_end(_input._vehicles[i].has_end()),
    v_start(_input._vehicles[i].tw.start),
    v_end(_input._vehicles[i].tw.end) {
}

duration_t tw_route::new_earliest_candidate(index_t job_rank,
                                            index_t rank) const {
  const auto& j = _input._jobs[job_rank];

  duration_t previous_earliest = v_start;
  duration_t previous_service = 0;
  duration_t previous_travel = 0;
  if (rank > 0) {
    const auto& previous_job = _input._jobs[route[rank - 1]];
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

duration_t tw_route::new_latest_candidate(index_t job_rank,
                                          index_t rank) const {
  const auto& j = _input._jobs[job_rank];

  duration_t next_latest = v_end;
  duration_t next_travel = 0;
  if (rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[rank];
    next_travel = m[j.index()][_input._jobs[route[rank]].index()];
  }

  assert(j.service + next_travel <= next_latest);
  return next_latest - j.service - next_travel;
}

void tw_route::fwd_update_earliest_from(index_t rank) {
  duration_t previous_earliest = earliest[rank];
  for (index_t i = rank + 1; i < route.size(); ++i) {
    const auto& previous_j = _input._jobs[route[i - 1]];
    const auto& next_j = _input._jobs[route[i]];
    duration_t next_earliest = previous_earliest + previous_j.service +
                               m[previous_j.index()][next_j.index()];

    if (next_earliest <= earliest[i]) {
      break;
    } else {
      assert(next_earliest <= latest[i]);
      earliest[i] = next_earliest;
      previous_earliest = next_earliest;
    }
  }
}

void tw_route::bwd_update_latest_from(index_t rank) {
  duration_t next_latest = latest[rank];
  for (index_t next_i = rank; next_i > 0; --next_i) {
    const auto& previous_j = _input._jobs[route[next_i - 1]];
    const auto& next_j = _input._jobs[route[next_i]];

    duration_t gap = previous_j.service + m[previous_j.index()][next_j.index()];
    assert(gap <= next_latest);
    duration_t previous_latest = next_latest - gap;

    if (latest[next_i - 1] <= previous_latest) {
      break;
    } else {
      assert(earliest[next_i - 1] <= previous_latest);
      latest[next_i - 1] = previous_latest;
      next_latest = previous_latest;
    }
  }
}

void tw_route::fwd_update_earliest_with_TW_from(index_t rank) {
  duration_t current_earliest = earliest[rank];
  for (index_t i = rank + 1; i < route.size(); ++i) {
    const auto& previous_j = _input._jobs[route[i - 1]];
    const auto& next_j = _input._jobs[route[i]];
    current_earliest +=
      previous_j.service + m[previous_j.index()][next_j.index()];
    const auto& next_TW = next_j.tws[tw_ranks[i]];
    current_earliest = std::max(current_earliest, next_TW.start);

    assert(current_earliest <= latest[i]);
    earliest[i] = current_earliest;
  }
}

void tw_route::bwd_update_latest_with_TW_from(index_t rank) {
  duration_t current_latest = latest[rank];
  for (index_t next_i = rank; next_i > 0; --next_i) {
    const auto& previous_j = _input._jobs[route[next_i - 1]];
    const auto& next_j = _input._jobs[route[next_i]];

    auto gap = previous_j.service + m[previous_j.index()][next_j.index()];
    assert(gap <= current_latest);
    current_latest -= gap;

    const auto& previous_TW = previous_j.tws[tw_ranks[next_i - 1]];
    current_latest = std::min(current_latest, previous_TW.end);

    assert(earliest[next_i - 1] <= current_latest);
    latest[next_i - 1] = current_latest;
  }
}

void tw_route::log() const {
  std::cout << "Route:\t\t";
  for (const auto j : route) {
    std::cout << _input._jobs[j].id << "\t";
  }
  std::cout << std::endl;

  std::cout << "Earliest:\t";
  for (const auto t : earliest) {
    std::cout << t << "\t";
  }
  std::cout << std::endl;

  std::cout << "Latest:\t\t";
  for (const auto t : latest) {
    std::cout << t << "\t";
  }
  std::cout << std::endl;
}

bool tw_route::is_valid_addition_for_tw(const index_t job_rank,
                                        const index_t rank) const {
  const auto& j = _input._jobs[job_rank];

  duration_t job_earliest = new_earliest_candidate(job_rank, rank);

  if (j.tws.back().end < job_earliest) {
    // Early abort if we're after the latest deadline for current job.
    return false;
  }

  duration_t next_latest = v_end;
  duration_t next_travel = 0;
  if (rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[rank];
    next_travel = m[j.index()][_input._jobs[route[rank]].index()];
  }

  bool valid = job_earliest + j.service + next_travel <= next_latest;

  if (valid) {
    duration_t new_latest = next_latest - j.service - next_travel;

    auto overlap_candidate =
      std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
        return job_earliest <= tw.end;
      });

    // The situation where there is no TW candidate should have been
    // previously filtered by early abort above.
    assert(overlap_candidate != j.tws.end());
    valid &= overlap_candidate->start <= new_latest;
  }

  return valid;
}

bool tw_route::is_valid_addition_for_tw(
  std::vector<index_t>::iterator first_job,
  std::vector<index_t>::iterator last_job,
  const index_t first_rank,
  const index_t last_rank) const {
  assert(first_job != last_job);
  // Handle first job earliest date.
  const auto& first_j = _input._jobs[*first_job];
  duration_t job_earliest = new_earliest_candidate(*first_job, first_rank);

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
    const auto& first_j = _input._jobs[*first_job];
    const auto& next_j = _input._jobs[*next_job];
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
  const auto& j = _input._jobs[*first_job];
  duration_t next_latest = v_end;
  duration_t next_travel = 0;
  if (last_rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[last_rank];
    next_travel = m[j.index()][_input._jobs[route[last_rank]].index()];
  }

  return job_earliest + j.service + next_travel <= next_latest;
}

void tw_route::add(const index_t job_rank, const index_t rank) {
  assert(rank <= route.size());

  duration_t job_earliest = new_earliest_candidate(job_rank, rank);
  duration_t job_latest = new_latest_candidate(job_rank, rank);

  // Pick first compatible TW.
  const auto& tws = _input._jobs[job_rank].tws;
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

  fwd_update_earliest_from(rank);
  bwd_update_latest_from(rank);
}

void tw_route::remove(const index_t rank, const unsigned count) {
  assert(rank + count <= route.size());

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
      const auto& new_first_j = _input._jobs[route[count]];
      duration_t start_earliest = v_start;
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
      const auto& new_last_j = _input._jobs[route[bwd_rank]];
      duration_t end_latest = v_end;
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
    fwd_update_earliest_with_TW_from(fwd_rank);
    bwd_update_latest_with_TW_from(bwd_rank);
  }
}
