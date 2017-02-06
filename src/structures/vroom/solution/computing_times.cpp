/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "computing_times.h"

computing_times_t::computing_times_t():
  loading(0),
  solving(0),
  routing(0){}

rapidjson::Value computing_times_t::to_json(bool geometry,
                                            rapidjson::Document::AllocatorType& allocator) const{
  rapidjson::Value json_ct(rapidjson::kObjectType);

  json_ct.AddMember("loading", loading, allocator);
  json_ct.AddMember("solving", solving, allocator);

  if(geometry){
    // Log route information timing when using OSRM.
    json_ct.AddMember("routing", routing, allocator);
  }

  return json_ct;
}
