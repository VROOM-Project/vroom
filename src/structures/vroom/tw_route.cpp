/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <algorithm>

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
    breaks_travel_margin_before(input.vehicles[v].breaks.size()),
    breaks_travel_margin_after(input.vehicles[v].breaks.size()) {
  std::string break_error = "Inconsistent breaks for vehicle " +
                            std::to_string(input.vehicles[v].id) + ".";

  const auto& breaks = input.vehicles[v].breaks;

  Duration previous_earliest = v_start;

  for (Index i = 0; i < breaks.size(); ++i) {
    const auto& b = breaks[i];
    const auto b_tw =
      std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
        return previous_earliest <= tw.end;
      });
    if (b_tw == b.tws.end()) {
      throw Exception(ERROR::INPUT, break_error);
    }

    break_earliest[i] = std::max(previous_earliest, b_tw->start);
    breaks_travel_margin_before[i] = break_earliest[i] - previous_earliest;

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

    const auto b_tw =
      std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
        return tw.start <= next_latest;
      });
    if (b_tw == b.tws.rend()) {
      throw Exception(ERROR::INPUT, break_error);
    }

    break_latest[i] = std::min(next_latest, b_tw->end);
    breaks_travel_margin_after[i] = next_latest - break_latest[i];

    next_latest = break_latest[i];

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
      previous_travel = m[v.start.value().index()][j.index()];
    }
  }

  return previous_earliest + previous_service + previous_travel;
}

Duration TWRoute::previous_earliest_end(const Input& input,
                                        Index job_rank,
                                        Index rank,
                                        Duration& previous_travel) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration previous_earliest = v_start;
  Duration previous_service = 0;
  if (rank > 0) {
    const auto& previous_job = input.jobs[route[rank - 1]];
    previous_earliest = earliest[rank - 1];
    previous_service = previous_job.service;
    previous_travel = m[previous_job.index()][j.index()];
  } else {
    if (has_start) {
      previous_travel = m[v.start.value().index()][j.index()];
    }
  }

  return previous_earliest + previous_service;
}

Duration TWRoute::next_latest_start(const Input& input,
                                    Index job_rank,
                                    Index rank,
                                    Duration& next_travel) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration next_latest = v_end;
  if (rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.value().index()];
    }
  } else {
    next_latest = latest[rank];
    next_travel = m[j.index()][input.jobs[route[rank]].index()];
  }

  return next_latest;
}

void TWRoute::fwd_update_earliest_from(const Input& input, Index rank) {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  Duration current_earliest = earliest[rank];
  bool handle_last_breaks = true;

  for (Index i = rank + 1; i < route.size(); ++i) {
    const auto& next_j = input.jobs[route[i]];
    Duration remaining_travel_time =
      m[input.jobs[route[i - 1]].index()][next_j.index()];
    Duration previous_service = input.jobs[route[i - 1]].service;

    // Update earliest dates and margins for breaks.
    assert(breaks_at_rank[i] <= breaks_counts[i]);
    Index break_rank = breaks_counts[i] - breaks_at_rank[i];

    for (Index r = 0; r < breaks_at_rank[i]; ++r, ++break_rank) {
      const auto& b = v.breaks[break_rank];

      current_earliest += previous_service;

      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      assert(b_tw != b.tws.end());

      if (current_earliest < b_tw->start) {
        auto margin = b_tw->start - current_earliest;
        breaks_travel_margin_before[break_rank] = margin;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          remaining_travel_time = 0;
        }

        current_earliest = b_tw->start;
      } else {
        breaks_travel_margin_before[break_rank] = 0;
      }

      break_earliest[break_rank] = current_earliest;
      previous_service = v.breaks[break_rank].service;
    }

    // Back to the job after breaks.
    current_earliest += previous_service + remaining_travel_time;

    const auto& next_TW = next_j.tws[tw_ranks[i]];
    current_earliest = std::max(current_earliest, next_TW.start);

    assert(current_earliest <= latest[i]);
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
    Index i = route.size();
    Duration remaining_travel_time =
      (v.has_end()) ? m[input.jobs[route[i - 1]].index()][v.end.value().index()]
                    : 0;

    Duration previous_service = input.jobs[route[i - 1]].service;

    assert(breaks_at_rank[i] <= breaks_counts[i]);
    Index break_rank = breaks_counts[i] - breaks_at_rank[i];

    for (Index r = 0; r < breaks_at_rank[i]; ++r, ++break_rank) {
      const auto& b = v.breaks[break_rank];
      current_earliest += previous_service;

      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      assert(b_tw != b.tws.end());

      if (current_earliest < b_tw->start) {
        auto margin = b_tw->start - current_earliest;
        breaks_travel_margin_before[break_rank] = margin;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          remaining_travel_time = 0;
        }

        current_earliest = b_tw->start;
      } else {
        breaks_travel_margin_before[break_rank] = 0;
      }

      break_earliest[break_rank] = current_earliest;
      previous_service = v.breaks[break_rank].service;
    }

    earliest_end = current_earliest + previous_service + remaining_travel_time;
    assert(earliest_end <= v_end);
  }
}

void TWRoute::bwd_update_latest_from(const Input& input, Index rank) {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  Duration current_latest = latest[rank];
  bool handle_first_breaks = true;

  for (Index next_i = rank; next_i > 0; --next_i) {
    const auto& previous_j = input.jobs[route[next_i - 1]];
    Duration remaining_travel_time =
      m[previous_j.index()][input.jobs[route[next_i]].index()];

    // Update latest dates and margins for breaks.
    assert(breaks_at_rank[next_i] <= breaks_counts[next_i]);
    Index break_rank = breaks_counts[next_i];

    for (Index r = 0; r < breaks_at_rank[next_i]; ++r) {
      --break_rank;

      const auto& b = v.breaks[break_rank];
      assert(b.service <= current_latest);
      current_latest -= b.service;

      const auto b_tw =
        std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
          return tw.start <= current_latest;
        });
      assert(b_tw != b.tws.rend());

      if (b_tw->end < current_latest) {
        auto margin = current_latest - b_tw->end;
        breaks_travel_margin_after[break_rank] = margin;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          remaining_travel_time = 0;
        }

        current_latest = b_tw->end;
      } else {
        breaks_travel_margin_after[break_rank] = 0;
      }

      break_latest[break_rank] = current_latest;
    }

    // Back to the job after breaks.
    auto gap = previous_j.service + remaining_travel_time;
    assert(gap <= current_latest);
    current_latest -= gap;

    const auto& previous_TW = previous_j.tws[tw_ranks[next_i - 1]];
    current_latest = std::min(current_latest, previous_TW.end);

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
    Index next_i = 0;

    assert(breaks_at_rank[next_i] <= breaks_counts[next_i]);
    Index break_rank = breaks_counts[next_i];

    for (Index r = 0; r < breaks_at_rank[next_i]; ++r) {
      --break_rank;
      const auto& b = v.breaks[break_rank];

      assert(b.service <= current_latest);
      current_latest -= b.service;

      const auto b_tw =
        std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
          return tw.start <= current_latest;
        });
      assert(b_tw != b.tws.rend());
      if (b_tw->end < current_latest) {
        breaks_travel_margin_after[break_rank] = current_latest - b_tw->end;

        current_latest = b_tw->end;
      } else {
        breaks_travel_margin_after[break_rank] = 0;
      }

      break_latest[break_rank] = current_latest;
    }
  }
}

OrderChoice::OrderChoice(const Job& j,
                         const Break& b,
                         const Duration current_earliest,
                         const Duration previous_travel)
  : add_job_first(false),
    add_break_first(false),
    j_tw(std::find_if(j.tws.begin(),
                      j.tws.end(),
                      [&](const auto& tw) {
                        return current_earliest + previous_travel <= tw.end;
                      })),
    b_tw(std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
      return current_earliest <= tw.end;
    })) {
}

OrderChoice TWRoute::order_choice(const Job& j,
                                  const Break& b,
                                  const Duration current_earliest,
                                  const Duration previous_travel) const {
  OrderChoice oc(j, b, current_earliest, previous_travel);

  if (oc.j_tw == j.tws.end() or oc.b_tw == b.tws.end()) {
    // If either job or break can't fit first, then none of the
    // orderings are valid.
    return oc;
  }

  Duration job_then_break_end;
  Duration break_then_job_end;

  // Try putting job first then break.
  Duration earliest_job_end =
    std::max(current_earliest + previous_travel, oc.j_tw->start) + j.service;

  const auto new_b_tw =
    std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
      return earliest_job_end <= tw.end;
    });
  if (new_b_tw == b.tws.end()) {
    // Break does not fit after job due to its time windows. Only
    // option is to choose break first.
    oc.add_break_first = true;
    return oc;
  } else {
    job_then_break_end =
      std::max(earliest_job_end, new_b_tw->start) + b.service;
  }

  // Try putting break first then job.
  Duration travel_after_break = previous_travel;
  Duration earliest_job_start = current_earliest;

  if (current_earliest < oc.b_tw->start) {
    auto margin = oc.b_tw->start - current_earliest;
    if (margin < travel_after_break) {
      travel_after_break -= margin;
    } else {
      travel_after_break = 0;
    }

    earliest_job_start = oc.b_tw->start;
  }

  earliest_job_start += b.service + travel_after_break;

  const auto new_j_tw =
    std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
      return earliest_job_start <= tw.end;
    });
  if (new_j_tw == j.tws.end()) {
    // Job does not fit after break due to its time windows. Only
    // option is to choose job first.
    oc.add_job_first = true;
    return oc;
  } else {
    break_then_job_end =
      std::max(earliest_job_start, new_j_tw->start) + j.service;
  }

  // In case where both ordering options are doable based on time
  // windows, we pick the ordering that minimizes earliest end date
  // for sequence.
  if (break_then_job_end < job_then_break_end) {
    oc.add_break_first = true;
  } else if (break_then_job_end == job_then_break_end) {
    // If end date is the same for both ordering options, decide
    // based on earliest deadline.
    if (oc.j_tw->end <= oc.b_tw->end) {
      oc.add_job_first = true;
    } else {
      oc.add_break_first = true;
    }
  } else {
    oc.add_job_first = true;
  }

  return oc;
}

bool TWRoute::is_valid_addition_for_tw(const Input& input,
                                       const Index job_rank,
                                       const Index rank) const {
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration previous_travel = 0;
  Duration current_earliest =
    previous_earliest_end(input, job_rank, rank, previous_travel);

  Duration next_travel = 0;
  Duration next_start = next_latest_start(input, job_rank, rank, next_travel);

  bool job_added = false;

  assert(breaks_at_rank[rank] <= breaks_counts[rank]);

  for (Index r = 0; r < breaks_at_rank[rank]; ++r) {
    Index break_rank = (breaks_counts[rank] + r) - breaks_at_rank[rank];
    const auto& b = v.breaks[break_rank];

    if (job_added) {
      // Compute earliest end date for current break.
      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });

      if (b_tw == b.tws.end()) {
        // Break does not fit due to its time windows.
        return false;
      }

      if (current_earliest < b_tw->start) {
        auto margin = b_tw->start - current_earliest;
        if (margin < next_travel) {
          next_travel -= margin;
        } else {
          next_travel = 0;
        }

        current_earliest = b_tw->start;
      }

      current_earliest += b.service;
    } else {
      // Decide on ordering between break and added job.
      auto oc = order_choice(j, b, current_earliest, previous_travel);

      if (!oc.add_job_first and !oc.add_break_first) {
        // Infeasible insertion.
        return false;
      }

      // Feasible insertion based on time windows, now update next end
      // time with given insertion choice.
      assert(oc.add_job_first xor oc.add_break_first);
      if (oc.add_break_first) {
        if (current_earliest < oc.b_tw->start) {
          auto margin = oc.b_tw->start - current_earliest;
          if (margin < previous_travel) {
            previous_travel -= margin;
          } else {
            previous_travel = 0;
          }

          current_earliest = oc.b_tw->start;
        }

        current_earliest += b.service;
      }
      if (oc.add_job_first) {
        current_earliest =
          std::max(current_earliest + previous_travel, oc.j_tw->start) +
          j.service;
        job_added = true;
      }
    }
  }

  if (!job_added) {
    current_earliest += previous_travel;
    const auto j_tw =
      std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
        return current_earliest <= tw.end;
      });
    if (j_tw == j.tws.end()) {
      return false;
    }

    current_earliest = std::max(current_earliest, j_tw->start) + j.service;
  }

  return current_earliest + next_travel <= next_start;
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
      next_travel = m[j.index()][v.end.value().index()];
    }
  } else {
    next_latest = latest[last_rank];
    next_travel = m[j.index()][input.jobs[route[last_rank]].index()];
  }

  return job_earliest + j.service + next_travel <= next_latest;
}

void TWRoute::add(const Input& input, const Index job_rank, const Index rank) {
  assert(rank <= route.size());

  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration previous_travel = 0;
  Duration current_earliest =
    previous_earliest_end(input, job_rank, rank, previous_travel);

  Duration next_travel = 0;
  Duration current_latest =
    next_latest_start(input, job_rank, rank, next_travel);

  bool job_added = false;
  unsigned breaks_before = 0;

  assert(breaks_at_rank[rank] <= breaks_counts[rank]);

  for (Index r = 0; r < breaks_at_rank[rank]; ++r) {
    Index break_rank = (breaks_counts[rank] + r) - breaks_at_rank[rank];
    const auto& b = v.breaks[break_rank];

    // Decide on ordering between break and added job.
    auto oc = order_choice(j, b, current_earliest, previous_travel);
    assert(oc.add_job_first xor oc.add_break_first);

    // Now update next end time based on insertion choice.
    if (oc.add_break_first) {
      // Earliest/rank/margin data for breaks before added job is
      // unchanged.
      ++breaks_before;

      if (breaks_travel_margin_before[break_rank] < previous_travel) {
        previous_travel -= breaks_travel_margin_before[break_rank];
      } else {
        previous_travel = 0;
      }

      current_earliest = break_earliest[break_rank] + b.service;
    }
    if (oc.add_job_first) {
      assert(oc.j_tw != j.tws.end());
      current_earliest =
        std::max(current_earliest + previous_travel, oc.j_tw->start);

      tw_ranks.insert(tw_ranks.begin() + rank,
                      std::distance(j.tws.begin(), oc.j_tw));
      earliest.insert(earliest.begin() + rank, current_earliest);

      current_earliest += j.service;
      job_added = true;

      // Earliest/rank data for breaks after added job will be updated
      // through the call to fwd_update_earliest_from one break counts
      // are straightened up.
      break;
    }
  }

  if (!job_added) {
    current_earliest += previous_travel;
    const auto j_tw =
      std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
        return current_earliest <= tw.end;
      });
    assert(j_tw != j.tws.end());

    current_earliest = std::max(current_earliest, j_tw->start);

    tw_ranks.insert(tw_ranks.begin() + rank,
                    std::distance(j.tws.begin(), j_tw));
    earliest.insert(earliest.begin() + rank, current_earliest);
  }

  // Add breaks_* data for breaks before newly inserted job.
  unsigned breaks_after = breaks_at_rank[rank] - breaks_before;
  breaks_at_rank[rank] -= breaks_before;
  breaks_at_rank.insert(breaks_at_rank.begin() + rank, breaks_before);
  breaks_counts.insert(breaks_counts.begin() + rank,
                       breaks_counts[rank] - breaks_after);

  // Update latest dates for breaks after inserted job using updated
  // break data.
  Index break_rank = breaks_counts[rank + 1];
  for (Index r = 0; r < breaks_after; ++r) {
    --break_rank;
    const auto& b = v.breaks[break_rank];

    assert(b.service <= current_latest);
    current_latest -= b.service;

    const auto b_tw =
      std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
        return tw.start <= current_latest;
      });
    assert(b_tw != b.tws.rend());

    if (b_tw->end < current_latest) {
      auto margin = current_latest - b_tw->end;
      breaks_travel_margin_after[break_rank] = margin;
      if (margin < next_travel) {
        next_travel -= margin;
      } else {
        next_travel = 0;
      }

      current_latest = b_tw->end;
    } else {
      breaks_travel_margin_after[break_rank] = 0;
    }

    break_latest[break_rank] = current_latest;
  }

  // Update inserted job latest date.
  assert(j.service + next_travel <= current_latest);
  current_latest -= (j.service + next_travel);
  current_latest = std::min(current_latest, j.tws[tw_ranks[rank]].end);
  latest.insert(latest.begin() + rank, current_latest);

  // Updating the route needs to be done after TW stuff as
  // previous_earliest_end and next_latest_start rely on route size
  // before addition ; but before earliest/latest date propagation
  // that rely on route structure after addition.
  route.insert(route.begin() + rank, job_rank);

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
               m[new_last_job.index()][v.end.value().index()] <=
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
      previous_travel = m[v.start.value().index()][current_index];
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
    const auto candidate =
      std::find_if(tws.begin(), tws.end(), [&](const auto& tw) {
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
        job_earliest += m[current_job.index()][v.end.value().index()];
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
    return v_start + m[v.start.value().index()][new_first_index] <=
           latest[count];
  }

  // Check backward validity as of first non-removed job.
  Index current_rank = rank - 1;
  Index current_index = input.jobs[route[current_rank]].index();

  Index next_rank = rank + count;
  Duration next_latest = v_end;
  Duration next_travel = 0;

  if (next_rank == route.size()) {
    if (has_end) {
      next_travel = m[current_index][v.end.value().index()];
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
    next_travel = m[v.start.value().index()][current_index];
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
        start_earliest += m[v.start.value().index()][new_first_j.index()];
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
          new_last_j.service + m[new_last_j.index()][v.end.value().index()];
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
      current_earliest += m[v.start.value().index()][new_first_j.index()];
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
    const auto tw_candidate =
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
    const auto tw_candidate =
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
        current_earliest += m[v.start.value().index()][current_j.index()];
      }

      const auto tw_candidate =
        std::find_if(current_j.tws.begin(),
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
          new_last_j.service + m[new_last_j.index()][v.end.value().index()];
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
