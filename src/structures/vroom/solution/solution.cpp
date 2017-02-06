/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "solution.h"

solution::solution(index_t code, std::string error):
  code(code),
  error(error),
  summary(0){}

solution::solution(index_t code,
                   std::vector<route_t>&& routes,
                   duration_t cost):
    code(code),
    routes(std::move(routes)),
    summary(cost){}

rapidjson::Document solution::to_json(bool geometry) const{
  rapidjson::Document json_output;
  json_output.SetObject();
  rapidjson::Document::AllocatorType& allocator = json_output.GetAllocator();

  json_output.AddMember("code", code, allocator);
  if(code != 0){
    json_output.AddMember("error", rapidjson::Value(), allocator);
    json_output["error"].SetString(error.c_str(), error.size());
  }
  else{
    json_output.AddMember("summary",
                          summary.to_json(geometry, allocator),
                          allocator);
  }

  return json_output;
}
