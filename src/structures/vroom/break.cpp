/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/break.h"
#include "utils/helpers.h"

namespace vroom {

Break::Break(Id id, const std::vector<TimeWindow>& tws, Duration service)
  : id(id), tws(tws), service(service) {
  utils::check_tws(tws);
}

} // namespace vroom
