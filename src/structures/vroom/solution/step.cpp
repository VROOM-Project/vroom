/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "step.h"

step::step(TYPE type, location_t location, index_t id)
  : type(type), 
    location(location),
    id(id),
    location_index_provided(false),
    location_index(0) {}

step::step(TYPE type, location_t location, index_t id, index_t location_index)
  : type(type),
    location(location),
    id(id),
    location_index_provided(true),
    location_index(location_index) {}

