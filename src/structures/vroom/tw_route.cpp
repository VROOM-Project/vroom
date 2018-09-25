/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/tw_route.h"

tw_route::tw_route(const input& input, index_t i)
  : vehicle_rank(i),
    has_start(input._vehicles[i].has_start()),
    has_end(input._vehicles[i].has_end()),
    v_start(input._vehicles[i].tw.start),
    v_end(input._vehicles[i].tw.end) {
}

duration_t tw_route::new_earliest_candidate(const input& input,
                                            index_t job_rank,
                                            index_t rank) const {
  const auto& m = input.get_matrix();
  const auto& v = input._vehicles[vehicle_rank];
  const auto& j = input._jobs[job_rank];

  duration_t previous_earliest = v_start;
  duration_t previous_service = 0;
  duration_t previous_travel = 0;
  if (rank > 0) {
    const auto& previous_job = input._jobs[route[rank - 1]];
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

duration_t tw_route::new_latest_candidate(const input& input,
                                          index_t job_rank,
                                          index_t rank) const {
  const auto& m = input.get_matrix();
  const auto& v = input._vehicles[vehicle_rank];
  const auto& j = input._jobs[job_rank];

  duration_t next_latest = v_end;
  duration_t next_travel = 0;
  if (rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[rank];
    next_travel = m[j.index()][input._jobs[route[rank]].index()];
  }

  assert(j.service + next_travel <= next_latest);
  return next_latest - j.service - next_travel;
}

void tw_route::fwd_update_earliest_from(const input& input, index_t rank) {
  const auto& m = input.get_matrix();

  duration_t previous_earliest = earliest[rank];
  for (index_t i = rank + 1; i < route.size(); ++i) {
    const auto& previous_j = input._jobs[route[i - 1]];
    const auto& next_j = input._jobs[route[i]];
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

void tw_route::bwd_update_latest_from(const input& input, index_t rank) {
  const auto& m = input.get_matrix();

  duration_t next_latest = latest[rank];
  for (index_t next_i = rank; next_i > 0; --next_i) {
    const auto& previous_j = input._jobs[route[next_i - 1]];
    const auto& next_j = input._jobs[route[next_i]];

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

void tw_route::fwd_update_earliest_with_TW_from(const input& input,
                                                index_t rank) {
  const auto& m = input.get_matrix();

  duration_t current_earliest = earliest[rank];
  for (index_t i = rank + 1; i < route.size(); ++i) {
    const auto& previous_j = input._jobs[route[i - 1]];
    const auto& next_j = input._jobs[route[i]];
    current_earliest +=
      previous_j.service + m[previous_j.index()][next_j.index()];
    const auto& next_TW = next_j.tws[tw_ranks[i]];
    current_earliest = std::max(current_earliest, next_TW.start);

    assert(current_earliest <= latest[i]);
    if (current_earliest == earliest[i]) {
      break;
    }
    earliest[i] = current_earliest;
  }
}

void tw_route::bwd_update_latest_with_TW_from(const input& input,
                                              index_t rank) {
  const auto& m = input.get_matrix();

  duration_t current_latest = latest[rank];
  for (index_t next_i = rank; next_i > 0; --next_i) {
    const auto& previous_j = input._jobs[route[next_i - 1]];
    const auto& next_j = input._jobs[route[next_i]];

    auto gap = previous_j.service + m[previous_j.index()][next_j.index()];
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

void tw_route::log(const input& input) const {
  std::cout << "Route:\t\t";
  for (const auto j : route) {
    std::cout << input._jobs[j].id << "\t";
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

bool tw_route::is_valid_addition_for_tw(const input& input,
                                        const index_t job_rank,
                                        const index_t rank) const {
  const auto& m = input.get_matrix();
  const auto& v = input._vehicles[vehicle_rank];
  const auto& j = input._jobs[job_rank];

  duration_t job_earliest = new_earliest_candidate(input, job_rank, rank);

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
    next_travel = m[j.index()][input._jobs[route[rank]].index()];
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

template <class InputIterator>
bool tw_route::is_valid_addition_for_tw(const input& input,
                                        InputIterator first_job,
                                        InputIterator last_job,
                                        const index_t first_rank,
                                        const index_t last_rank) const {
  if (first_job == last_job) {
    return true;
  }
  assert(first_rank <= last_rank);

  const auto& m = input.get_matrix();
  const auto& v = input._vehicles[vehicle_rank];

  // Handle first job earliest date.
  const auto& first_j = input._jobs[*first_job];
  duration_t job_earliest =
    new_earliest_candidate(input, *first_job, first_rank);

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
    const auto& first_j = input._jobs[*first_job];
    const auto& next_j = input._jobs[*next_job];
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
  const auto& j = input._jobs[*first_job];
  duration_t next_latest = v_end;
  duration_t next_travel = 0;
  if (last_rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.get().index()];
    }
  } else {
    next_latest = latest[last_rank];
    next_travel = m[j.index()][input._jobs[route[last_rank]].index()];
  }

  return job_earliest + j.service + next_travel <= next_latest;
}

void tw_route::add(const input& input,
                   const index_t job_rank,
                   const index_t rank) {
  assert(rank <= route.size());

  duration_t job_earliest = new_earliest_candidate(input, job_rank, rank);
  duration_t job_latest = new_latest_candidate(input, job_rank, rank);

  // Pick first compatible TW.
  const auto& tws = input._jobs[job_rank].tws;
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
}

void tw_route::remove(const input& input,
                      const index_t rank,
                      const unsigned count) {
  assert(rank + count <= route.size());

  const auto& m = input.get_matrix();
  const auto& v = input._vehicles[vehicle_rank];

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
      const auto& new_first_j = input._jobs[route[count]];
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
      const auto& new_last_j = input._jobs[route[bwd_rank]];
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
    fwd_update_earliest_with_TW_from(input, fwd_rank);
    bwd_update_latest_with_TW_from(input, bwd_rank);
  }
}

template <class InputIterator>
void tw_route::replace(const input& input,
                       InputIterator first_job,
                       InputIterator last_job,
                       const index_t first_rank,
                       const index_t last_rank) {
  assert(first_rank <= last_rank);

  const auto& m = input.get_matrix();
  const auto& v = input._vehicles[vehicle_rank];

  // Number of items to erase and add.
  unsigned erase_count = last_rank - first_rank;
  unsigned add_count = std::distance(first_job, last_job);

  // Start updates for earliest/latest dates in new route.
  duration_t current_earliest;
  if (first_rank > 0) {
    current_earliest = earliest[first_rank - 1];
  } else {
    // Use earliest date for new first job.
    const auto& new_first_j = input._jobs[*first_job];
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
    const auto& current_j = input._jobs[route[insert_rank]];

    if (insert_rank > 0) {
      const auto& previous_j = input._jobs[route[insert_rank - 1]];
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
    // during [fwd|bwd]_update_latest_with_TW_from below.
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
    const auto& current_j = input._jobs[route[insert_rank]];

    if (insert_rank > 0) {
      const auto& previous_j = input._jobs[route[insert_rank - 1]];
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
      const auto& current_j = input._jobs[route[insert_rank]];
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
      const auto& new_last_j = input._jobs[route[insert_rank]];

      duration_t end_latest = v_end;
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
      fwd_update_earliest_with_TW_from(input, insert_rank - 1);
    }

    // Update latest dates backward.
    bwd_update_latest_with_TW_from(input, insert_rank);
  }
}

template bool
tw_route::is_valid_addition_for_tw(const input& input,
                                   std::vector<index_t>::iterator first_job,
                                   std::vector<index_t>::iterator last_job,
                                   const index_t first_rank,
                                   const index_t last_rank) const;
template bool tw_route::is_valid_addition_for_tw(
  const input& input,
  std::vector<index_t>::reverse_iterator first_job,
  std::vector<index_t>::reverse_iterator last_job,
  const index_t first_rank,
  const index_t last_rank) const;

template void tw_route::replace(const input& input,
                                std::vector<index_t>::iterator first_job,
                                std::vector<index_t>::iterator last_job,
                                const index_t first_rank,
                                const index_t last_rank);
template void
tw_route::replace(const input& input,
                  std::vector<index_t>::reverse_iterator first_job,
                  std::vector<index_t>::reverse_iterator last_job,
                  const index_t first_rank,
                  const index_t last_rank);
