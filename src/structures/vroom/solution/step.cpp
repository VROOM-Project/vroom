/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "step.h"

step::step(TYPE type, location_t location):
  type(type),
  location(location){}

step::step(TYPE type, location_t location, index_t job):
  type(type),
  location(location),
  job(job){}

rapidjson::Value step::to_json(rapidjson::Document::AllocatorType& allocator) const{
  rapidjson::Value json_step(rapidjson::kObjectType);

  json_step.AddMember("type", rapidjson::Value(), allocator);
  std::string str_type;
  switch (type){
  case TYPE::START:
    str_type = "start";
    break;
  case TYPE::END:
    str_type = "end";
    break;
  case TYPE::JOB:
    str_type = "job";
    break;
  }
  json_step["type"].SetString(str_type.c_str(), str_type.size(), allocator);

  if(location.has_coordinates()){
    json_step.AddMember("location",
                        location.to_json(allocator),
                        allocator);
  }

  if(type == TYPE::JOB){
    json_step.AddMember("job", job, allocator);
  }

  return json_step;
}
