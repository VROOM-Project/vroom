/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "job.h"

job_t::job_t(ID_t id, index_t index) : job_t(id, boost::none, index) {
}

job_t::job_t(ID_t id, index_t index, const coords_t& coords)
  : job_t(id, boost::none, index, coords) {
}

job_t::job_t(ID_t id, const coords_t& coords) : job_t(id, boost::none, coords) {
}

bool job_t::has_amount() const {
  return amount != boost::none;
}
