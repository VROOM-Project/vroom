#ifndef STEP_H
#define STEP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../../include/rapidjson/document.h"
#include "../location.h"

enum class TYPE {START, JOB, END};

struct step{
  TYPE type;
  location_t location;
  index_t job;

  step(TYPE type, location_t location);

  step(TYPE type, location_t location, index_t job);

  rapidjson::Value to_json(rapidjson::Document::AllocatorType& allocator) const;
};

#endif
