#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"
#include "./location.h"

struct job: public location{
  const index_t id;

  template <typename... Args>
  job(index_t id,
      Args&&... args):
    location(std::forward<Args>(args)...),
    id(id){}
};

#endif
