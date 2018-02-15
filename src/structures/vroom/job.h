#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"
#include "./amount.h"
#include "./location.h"

struct job_t : public location_t {
  const ID_t id;
  boost::optional<amount_t> amount;

  job_t(ID_t id, index_t index);

  job_t(ID_t id, index_t index, const coords_t& coords);

  job_t(ID_t id, const coords_t& coords);

  template <typename... Args>
  job_t(ID_t id, const boost::optional<amount_t>& amount, Args&&... args)
    : location_t(std::forward<Args>(args)...), id(id), amount(amount) {
  }

  bool has_amount() const;

  // Hash and operator== to be able to use jobs in std::unordered_set.
  bool operator==(const job_t& other) const;
};

namespace std {
template <> struct hash<job_t> { size_t operator()(const job_t& k) const; };
}
#endif
