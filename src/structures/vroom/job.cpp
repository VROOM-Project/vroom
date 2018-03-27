/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "job.h"

job_t::job_t(ID_t id, index_t index)
  : job_t(id, amount_t(0), std::unordered_set<skill_t>(), index) {
}

job_t::job_t(ID_t id, index_t index, const coords_t& coords)
  : job_t(id, amount_t(0), std::unordered_set<skill_t>(), index, coords) {
}

job_t::job_t(ID_t id, const coords_t& coords)
  : job_t(id, amount_t(0), std::unordered_set<skill_t>(), coords) {
}
