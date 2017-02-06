/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "summary.h"

summary_t::summary_t(duration_t cost):
  cost(cost),
  duration(0),
  distance(0){}

rapidjson::Value summary_t::to_json(bool geometry,
                                    rapidjson::Document::AllocatorType& allocator) const{
  rapidjson::Value json_summary(rapidjson::kObjectType);

  json_summary.AddMember("computing_times",
                         computing_times.to_json(geometry,
                                                 allocator),
                         allocator);

  if(geometry){
    json_summary.AddMember("distance", distance, allocator);
    json_summary.AddMember("duration", duration, allocator);
  }

  json_summary.AddMember("cost", cost, allocator);

  return json_summary;
}
