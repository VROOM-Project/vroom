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

  bool is_valid_addition_for_tw(const Input&, const Index, const Index) const {
    return true;
  };

  bool is_valid_addition_for_tw_without_max_load(const Input&,
                                                 const Index,
                                                 const Index) const {
    return true;
  };

  template <std::forward_iterator Iter>
  bool is_valid_addition_for_tw(const Input&,
                                const Amount&,
                                const Iter,
                                const Iter,
                                const Index,
                                const Index) const {
    return true;
  }

  void add(const Input& input, Index job_rank, Index rank);

  bool is_valid_removal(const Input&, const Index, const unsigned) const {
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
