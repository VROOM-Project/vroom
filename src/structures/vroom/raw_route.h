#ifndef RAW_ROUTE_H
#define RAW_ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"

namespace vroom {

class RawRoute {
private:
  Amount _zero;

  // _exclusive_tag_counts[t] stores count of exclusive tag t in the route.
  std::vector<unsigned short> _exclusive_tag_counts;
  // _exclusive_tag_limits[t] stores the maximum allowed count for tag t in this
  // route. Typically 1, but can be > 1 when pinned conflicts are allowed and
  // the pinned workload already contains duplicates.
  std::vector<unsigned short> _exclusive_tag_limits;

  // _fwd_pickups[i] (resp. _fwd_deliveries[i]) stores the total
  // pickups (resp. deliveries) for single jobs up to rank i.
  std::vector<Amount> _fwd_pickups;
  std::vector<Amount> _fwd_deliveries;

  // _bwd_deliveries[i] (resp. _bwd_pickups[i]) stores the total
  // deliveries (resp. pickups) for single jobs pending after rank i.
  std::vector<Amount> _bwd_deliveries;
  std::vector<Amount> _bwd_pickups;

  // _pd_loads[i] stores the shipments load at rank i (included).
  std::vector<Amount> _pd_loads;

  // _nb_pickups[i] (resp. _nb_deliveries[i]) stores the number of
  // pickups (resp. deliveries) up to rank i.
  std::vector<unsigned> _nb_pickups;
  std::vector<unsigned> _nb_deliveries;

  // _current_loads[s] stores the vehicle load (taking all job types
  // into account) at *step* s (step 0 is the start, not the first job
  // rank).
  std::vector<Amount> _current_loads;

  // _fwd_peaks[s] stores the peak load (component-wise) up to *step*
  // s. _bwd_peaks[s] stores the peak load (component-wise) after
  // *step* s.
  std::vector<Amount> _fwd_peaks;
  std::vector<Amount> _bwd_peaks;

  // Store the difference between sum of single jobs deliveries
  // (resp. pickups) and vehicle capacity.
  Amount _delivery_margin;
  Amount _pickup_margin;

protected:
  const std::vector<unsigned short>& exclusive_tag_counts() const {
    return _exclusive_tag_counts;
  }

  const std::vector<unsigned short>& exclusive_tag_limits() const {
    return _exclusive_tag_limits;
  }

public:
  Index v_rank;
  Index v_type;
  bool has_start;
  bool has_end;
  Amount capacity;

  std::vector<Index> route;

  RawRoute(const Input& input, Index i, unsigned amount_size);

  void set_route(const Input& input, const std::vector<Index>& r);

  bool empty() const {
    return route.empty();
  }

  std::size_t size() const {
    return route.size();
  }

  void update_amounts(const Input& input);

  bool has_pending_delivery_after_rank(Index rank) const;

  bool has_delivery_after_rank(Index rank) const;

  bool has_pickup_up_to_rank(Index rank) const;

  const Amount& fwd_peak(Index rank) const {
    return _fwd_peaks[rank];
  }

  const Amount& bwd_peak(Index rank) const {
    return _bwd_peaks[rank];
  }

  const Amount& max_load() const {
    return _fwd_peaks.back();
  }

  // Compute max load of sub-route spanning the [0; rank[ range.
  Amount sub_route_max_load_before(Index rank) const {
    assert(0 < rank && rank < size());
    return _fwd_peaks[rank] - _bwd_deliveries[rank - 1];
  }

  // Compute max load of sub-route spanning the [rank; size[ range.
  Amount sub_route_max_load_after(Index rank) const {
    assert(0 < rank && rank < size());
    return _bwd_peaks[rank] - _fwd_pickups[rank - 1];
  }

  // Check validity for addition of a given load in current route at
  // rank.
  bool is_valid_addition_for_capacity(const Input&,
                                      const Amount& pickup,
                                      const Amount& delivery,
                                      Index rank) const;

  // Check if current load allows the addition of a pickup, just
  // considering capacity limitation at rank.
  bool is_valid_addition_for_load(const Input& input,
                                  const Amount& pickup,
                                  Index rank) const;

  // Check validity for inclusion (with regard to not breaking
  // capacity before and after inclusion) of some load in the existing
  // route at rank first_rank and before last_rank *in place of* the
  // current jobs that may be there.
  bool is_valid_addition_for_capacity_margins(const Input& input,
                                              const Amount& pickup,
                                              const Amount& delivery,
                                              Index first_rank,
                                              Index last_rank) const;

  // Check validity for inclusion (with regard to not breaking
  // capacity for included jobs) of the range [first_job; last_job) in
  // the existing route at rank first_rank and before last_rank *in
  // place of* the current jobs that may be there.
  template <std::forward_iterator Iter>
  bool is_valid_addition_for_capacity_inclusion(const Input& input,
                                                Amount delivery,
                                                Iter first_job,
                                                Iter last_job,
                                                Index first_rank,
                                                Index last_rank) const;

  const Amount& job_deliveries_sum() const;

  const Amount& job_pickups_sum() const;

  const Amount& delivery_margin() const;

  const Amount& pickup_margin() const;

  // Get sum of pickups (resp. deliveries) for all jobs in the range
  // [i, j).
  Amount pickup_in_range(Index i, Index j) const;
  Amount delivery_in_range(Index i, Index j) const;

  const Amount& bwd_deliveries(Index i) const {
    return _bwd_deliveries[i];
  }

  const Amount& fwd_deliveries(Index i) const {
    return _fwd_deliveries[i];
  }

  const Amount& bwd_pickups(Index i) const {
    return _bwd_pickups[i];
  }

  const Amount& fwd_pickups(Index i) const {
    return _fwd_pickups[i];
  }

  const Amount& load_at_step(Index s) const {
    return _current_loads[s];
  }

  bool is_valid_addition_for_tw(const Input& input,
                                const Index job_rank,
                                const Index rank) const {
    // Enforce pinned first/last boundaries (no TW logic here)
    const auto v = v_rank;
    // Exclusive tags: route membership constraint.
    if (!_exclusive_tag_counts.empty()) {
      for (const auto tid : input.exclusive_tag_ids(job_rank)) {
        if (_exclusive_tag_counts[tid] >= _exclusive_tag_limits[tid]) {
          return false;
        }
      }
    }
    // Enforce first-leg distance bound on head insertion for vehicles without pre-defined steps.
    if (rank == 0) {
      const auto& vehicle = input.vehicles[v];
      if (vehicle.has_start() && vehicle.steps.empty() &&
          vehicle.max_first_leg_distance != DEFAULT_MAX_DISTANCE) {
        const auto start_index = vehicle.start.value().index();
        const auto head_index = input.jobs[job_rank].index();
        const auto first_leg_distance =
          vehicle.eval(start_index, head_index).distance;
        if (first_leg_distance > vehicle.max_first_leg_distance) {
          return false;
        }
      }
    }
    if (const auto pf = input.pinned_first_for_vehicle(v); pf.has_value()) {
      const auto& req = pf.value();
      if (req.job_rank.has_value()) {
        // No insertion before the pinned-first job
        if (rank == 0 && job_rank != req.job_rank.value()) {
          return false;
        }
      } else if (req.pickup_rank.has_value() && req.delivery_rank.has_value()) {
        // No insertion at ranks 0 or 1 for pinned-first shipment (to preserve [pickup,delivery] at head)
        if (rank <= 1) {
          return false;
        }
      }
    }
    if (const auto pl = input.pinned_last_for_vehicle(v); pl.has_value()) {
      const auto& req = pl.value();
      if (req.job_rank.has_value()) {
        // No insertion after the pinned-last job (i.e., at end)
        if (rank == route.size() && job_rank != req.job_rank.value()) {
          return false;
        }
      } else if (req.pickup_rank.has_value() && req.delivery_rank.has_value()) {
        // No insertion at last or pre-last positions for pinned-last shipment
        if (rank >= (route.size() >= 1 ? route.size() - 1 : 0)) {
          return false;
        }
      }
    }
    return true;
  };

  bool is_valid_addition_for_tw_without_max_load(const Input&,
                                                 const Index,
                                                 const Index) const {
    return true;
  };

  template <std::forward_iterator Iter>
  bool is_valid_addition_for_tw(const Input& input,
                                const Amount& /*delivery*/,
                                const Iter first_job,
                                const Iter last_job,
                                const Index first_rank,
                                const Index last_rank) const {
    // Enforce pinned first/last boundaries (no TW logic here)
    const auto v = v_rank;
    const auto insert_len = static_cast<unsigned>(std::distance(first_job, last_job));

    // Exclusive tags: route membership constraint.
    if (!_exclusive_tag_counts.empty()) {
      // Collect tag ids present in the inserted range.
      std::vector<Index> inserted_tag_ids;
      for (auto it = first_job; it != last_job; ++it) {
        const auto& tags = input.exclusive_tag_ids(*it);
        inserted_tag_ids.insert(inserted_tag_ids.end(), tags.begin(), tags.end());
      }

      if (!inserted_tag_ids.empty()) {
        std::ranges::sort(inserted_tag_ids);

        // Unique tids + counts in inserted range.
        std::vector<Index> inserted_unique_tids;
        std::vector<unsigned short> inserted_counts;
        for (std::size_t i = 0; i < inserted_tag_ids.size();) {
          const auto tid = inserted_tag_ids[i];
          std::size_t j = i + 1;
          while (j < inserted_tag_ids.size() && inserted_tag_ids[j] == tid) {
            ++j;
          }
          const auto c = static_cast<unsigned short>(j - i);
          if (c > _exclusive_tag_limits[tid]) {
            return false;
          }
          inserted_unique_tids.push_back(tid);
          inserted_counts.push_back(c);
          i = j;
        }

        // Count occurrences of those tids inside replaced segment.
        std::vector<unsigned short> removed_counts(inserted_unique_tids.size(), 0);

        const Index stop = std::min(static_cast<Index>(route.size()), last_rank);
        for (Index r = std::min(first_rank, stop); r < stop; ++r) {
          for (const auto tid : input.exclusive_tag_ids(route[r])) {
            const auto lb = std::ranges::lower_bound(inserted_unique_tids, tid);
            if (lb != inserted_unique_tids.end() && *lb == tid) {
              const auto pos = static_cast<std::size_t>(
                std::distance(inserted_unique_tids.begin(), lb));
              removed_counts[pos] += 1;
            }
          }
        }

        for (std::size_t i = 0; i < inserted_unique_tids.size(); ++i) {
          const auto tid = inserted_unique_tids[i];
          const auto total = _exclusive_tag_counts[tid];
          const auto removed = removed_counts[i];
          const auto added = inserted_counts[i];
          // Check resulting count <= limit.
          const int new_total = static_cast<int>(total) -
                                static_cast<int>(removed) +
                                static_cast<int>(added);
          if (new_total > static_cast<int>(_exclusive_tag_limits[tid])) {
            return false;
          }
        }
      }
    }

    // Enforce first-leg distance bound on head insertion for vehicles without pre-defined steps.
    if (first_rank == 0 && insert_len > 0) {
      const auto& vehicle = input.vehicles[v];
      if (vehicle.has_start() && vehicle.steps.empty() &&
          vehicle.max_first_leg_distance != DEFAULT_MAX_DISTANCE) {
        const Index head_job_rank = *first_job;
        const auto start_index = vehicle.start.value().index();
        const auto head_index = input.jobs[head_job_rank].index();
        const auto first_leg_distance =
          vehicle.eval(start_index, head_index).distance;
        if (first_leg_distance > vehicle.max_first_leg_distance) {
          return false;
        }
      }
    }

    if (const auto pf = input.pinned_first_for_vehicle(v); pf.has_value()) {
      const auto& req = pf.value();
      if (req.job_rank.has_value()) {
        if (first_rank == 0) {
          // After replace, first job must be the pinned-first job
          Index new_first;
          if (insert_len > 0) {
            new_first = *first_job;
          } else {
            if (last_rank < route.size()) {
              new_first = route[last_rank];
            } else {
              // Route empty after operation: violates pinned-first presence
              return false;
            }
          }
          if (new_first != req.job_rank.value()) {
            return false;
          }
        }
      } else if (req.pickup_rank.has_value() && req.delivery_rank.has_value()) {
        // For pinned-first shipment, ensure resulting first two are [pickup,delivery]
        if (first_rank == 0) {
          std::optional<Index> n0;
          std::optional<Index> n1;
          if (insert_len >= 2) {
            auto it = first_job;
            n0 = *it;
            ++it;
            n1 = *it;
          } else if (insert_len == 1) {
            n0 = *first_job;
            if (last_rank < route.size()) {
              n1 = route[last_rank];
            }
          } else {
            if (last_rank < route.size()) {
              n0 = route[last_rank];
            }
            if (last_rank + 1 < route.size()) {
              n1 = route[last_rank + 1];
            }
          }
          if (!n0.has_value() || !n1.has_value() ||
              n0.value() != req.pickup_rank.value() ||
              n1.value() != req.delivery_rank.value()) {
            return false;
          }
        }
        // Also disallow inserting at rank 1 when current head is already the pinned pair
        if (first_rank == 1 && insert_len > 0) {
          if (route.size() >= 2 && route[0] == req.pickup_rank.value() &&
              route[1] == req.delivery_rank.value()) {
            return false;
          }
        }
      }
    }

    if (const auto pl = input.pinned_last_for_vehicle(v); pl.has_value()) {
      const auto& req = pl.value();
      if (req.job_rank.has_value()) {
        if (last_rank == route.size()) {
          // After replace, last job must be the pinned-last job
          std::optional<Index> new_last;
          if (insert_len > 0) {
            auto it = first_job;
            std::advance(it, insert_len - 1);
            new_last = *it;
          } else {
            if (first_rank > 0) {
              new_last = route[first_rank - 1];
            }
          }
          if (!new_last.has_value() || new_last.value() != req.job_rank.value()) {
            return false;
          }
        }
      } else if (req.pickup_rank.has_value() && req.delivery_rank.has_value()) {
        // For pinned-last shipment, ensure resulting last two are [pickup,delivery]
        if (last_rank == route.size()) {
          if (insert_len < 2) {
            return false;
          }
          auto it = first_job;
          std::advance(it, insert_len - 2);
          const Index n0 = *it;
          ++it;
          const Index n1 = *it;
          if (n0 != req.pickup_rank.value() || n1 != req.delivery_rank.value()) {
            return false;
          }
        }
        // Also disallow inserting at pre-last rank when tail is already the pinned pair
        if (first_rank == (route.size() >= 1 ? route.size() - 1 : 0) && insert_len > 0) {
          if (route.size() >= 2 && route[route.size() - 2] == req.pickup_rank.value() &&
              route.back() == req.delivery_rank.value()) {
            return false;
          }
        }
      }
    }

    return true;
  }

  void add(const Input& input, Index job_rank, Index rank);

  bool is_valid_removal(const Input& input,
                        const Index rank,
                        const unsigned count) const {
    assert(rank + count <= route.size());

    const auto removes_job = [&](Index job_rank) {
      for (Index r = rank; r < rank + count; ++r) {
        if (route[r] == job_rank) {
          return true;
        }
      }
      return false;
    };

    if (const auto pf = input.pinned_first_for_vehicle(v_rank); pf.has_value()) {
      const auto& req = pf.value();
      if ((req.job_rank.has_value() && removes_job(req.job_rank.value())) ||
          (req.pickup_rank.has_value() && removes_job(req.pickup_rank.value())) ||
          (req.delivery_rank.has_value() && removes_job(req.delivery_rank.value()))) {
        return false;
      }
    }

    if (const auto pl = input.pinned_last_for_vehicle(v_rank); pl.has_value()) {
      const auto& req = pl.value();
      if ((req.job_rank.has_value() && removes_job(req.job_rank.value())) ||
          (req.pickup_rank.has_value() && removes_job(req.pickup_rank.value())) ||
          (req.delivery_rank.has_value() && removes_job(req.delivery_rank.value()))) {
        return false;
      }
    }

    return true;
  };

  void remove(const Input& input, Index rank, unsigned count);

  // Add the range [first_job; last_job) in the existing route at rank
  // first_rank and before last_rank *in place of* the current jobs
  // that may be there.
  template <std::forward_iterator Iter>
  void replace(const Input& input,
               Iter first_job,
               Iter last_job,
               Index first_rank,
               Index last_rank);

  template <std::forward_iterator Iter>
  void replace(const Input& input,
               const Amount&,
               const Iter first_job,
               const Iter last_job,
               const Index first_rank,
               const Index last_rank) {
    replace(input, first_job, last_job, first_rank, last_rank);
  }
};

} // namespace vroom

#endif
