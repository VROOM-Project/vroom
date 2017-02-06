/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../../include/rapidjson/document.h"
#include "./route.h"

route_t::route_t(index_t vehicle,
                 std::vector<step> steps,
                 duration_t cost):
  vehicle(vehicle),
  steps(std::move(steps)),
  cost(cost),
  duration(0),
  distance(0){}

rapidjson::Value route_t::to_json(rapidjson::Document::AllocatorType& allocator) const{
  rapidjson::Value json_route(rapidjson::kObjectType);

  json_route.AddMember("vehicle", vehicle, allocator);
  json_route.AddMember("cost", cost, allocator);

  if(!geometry.empty()){
    json_route.AddMember("distance", distance, allocator);
    json_route.AddMember("duration", duration, allocator);
  }

  rapidjson::Value json_steps(rapidjson::kArrayType);
  for(const auto& step: steps){
    json_steps.PushBack(step.to_json(allocator), allocator);
  }

  json_route.AddMember("steps", json_steps, allocator);

  if(!geometry.empty()){
    json_route.AddMember("geometry", rapidjson::Value(), allocator);
    json_route["geometry"].SetString(geometry.c_str(), geometry.size());
  }

  return json_route;
}
