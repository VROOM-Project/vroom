#ifndef SUMMARY_H
#define SUMMARY_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../../include/rapidjson/document.h"
#include "./computing_times.h"

struct summary_t{
  duration_t cost;
  duration_t duration;
  distance_t distance;
  computing_times_t computing_times;

  summary_t(duration_t cost);

  rapidjson::Value to_json(bool geometry,
                           rapidjson::Document::AllocatorType& allocator) const;
};

#endif
