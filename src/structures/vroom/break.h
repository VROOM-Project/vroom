#ifndef BREAK_H
#define BREAK_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct Break {
  Id id;
  std::vector<TimeWindow> tws;
  Duration service;
  std::string description;
  std::optional<Amount> max_load;

  Break(Id id,
        const std::vector<TimeWindow>& tws =
          std::vector<TimeWindow>(1, TimeWindow()),
        UserDuration service = 0,
        std::string description = "",
        std::optional<Amount> max_load = std::optional<Amount>());

  bool is_valid_start(Duration time) const;

  bool is_valid_for_load(const Amount& load) const;
};

} // namespace vroom

#endif
