#ifndef RAW_ROUTE_H
#define RAW_ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"

namespace vroom {

class RawRoute {
private:
  // fwd_pickups[i] stores the total pickups up to rank i.
  std::vector<Amount> fwd_pickups;

  // bwd_deliveries[i] stores the total deliveries pending after rank
  // i.
  std::vector<Amount> bwd_deliveries; // TODO maybe no need to
                                      // store this.

  // current_loads[s] stores the vehicle load at *step* s (step 0 is
  // the start, not the first job rank).
  std::vector<Amount> current_loads;

  // fwd_peaks[s] stores the peak load (component-wise) up to *step*
  // s. bwd_peaks[s] stores the peak load (component-wise) after
  // *step* s.
  std::vector<Amount> fwd_peaks;
  std::vector<Amount> bwd_peaks;

  void update_amounts(const Input& input);

public:
  Index vehicle_rank;
  bool has_start;
  bool has_end;

  std::vector<Index> route;

  RawRoute(const Input& input, Index i);

  void set_route(const Input& input, const std::vector<Index>& r);

  bool empty() const;

  std::size_t size() const;

  bool is_valid_addition_for_tw(const Input&, const Index, const Index) const {
    return true;
  };

  void add(const Input& input, const Index job_rank, const Index rank);

  bool is_valid_removal(const Input&, const Index, const unsigned) const {
    return true;
  };

  void remove(const Input& input, const Index rank, const unsigned count);
};

} // namespace vroom

#endif
