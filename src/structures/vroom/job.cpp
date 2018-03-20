/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "job.h"

job_t::job_t(ID_t id, index_t index)
  : job_t(id, boost::none, std::unordered_set<skill_t>(), index) {
}

job_t::job_t(ID_t id, index_t index, const coords_t& coords)
  : job_t(id, boost::none, std::unordered_set<skill_t>(), index, coords) {
}

job_t::job_t(ID_t id, const coords_t& coords)
  : job_t(id, boost::none, std::unordered_set<skill_t>(), coords) {
}

bool job_t::has_amount() const {
  return amount != boost::none;
}

bool job_t::operator==(const job_t& other) const {
  return (this->id == other.id);
}

size_t std::hash<job_t>::operator()(const job_t& j) const {
  return std::hash<ID_t>()(j.id);
}
