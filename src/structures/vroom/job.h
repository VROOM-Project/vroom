#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"
#include "./location.h"

struct job_t : public location_t {
  const index_t id;

  template <typename... Args>
  job_t(index_t id, Args&&... args)
    : location_t(std::forward<Args>(args)...),
      id(id) {}
};

#endif
