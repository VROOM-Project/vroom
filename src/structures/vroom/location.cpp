/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "location.h"

location_t::location_t(index_t index):
  index(index){}

location_t::location_t(index_t index, coordinate_t lon, coordinate_t lat):
  index(index), lon(lon), lat(lat){}

bool location_t::has_coordinates() const{
    return (lon != boost::none) and (lat != boost::none);
}

rapidjson::Value location_t::to_json(rapidjson::Document::AllocatorType& allocator) const{
  rapidjson::Value json_coords(rapidjson::kArrayType);

  json_coords.PushBack(lon.get(), allocator);
  json_coords.PushBack(lat.get(), allocator);

  return json_coords;
}
