/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./input.h"

input::input(): _location_number(0){}

void input::add_job(index_t id, optional_coords_t coords){
  // Using current number of locations as index of this job in the
  // matrix.
  if(coords == boost::none){
    _jobs.emplace_back(id, _location_number++);
  }
  else{
    _jobs.emplace_back(id,
                       _location_number++,
                       coords.get()[0],
                       coords.get()[1]);
  }
  _ordered_locations.push_back(_jobs.back());
}

void input::add_vehicle(index_t id,
                        optional_coords_t start_coords,
                        optional_coords_t end_coords){
  // Using current number of locations as index of start and end in
  // the matrix.
  if((!start_coords) and (!end_coords)){
    throw custom_exception("No start or end specified for vehicle "
                           + std::to_string(id)
                           + '.');
  }

  boost::optional<location> start = (start_coords == boost::none) ?
    boost::none:
    boost::optional<location>(
      {++_location_number, (*start_coords)[0], (*start_coords)[1]}
      );

  boost::optional<location> end = (end_coords == boost::none) ?
    boost::none:
    boost::optional<location>(
      {++_location_number, (*end_coords)[0], (*end_coords)[1]}
      );

  _vehicles.emplace_back(id, start, end);

  if(start_coords){
    _ordered_locations.push_back(_vehicles.back().start.value());
  }
  if(end_coords){
    _ordered_locations.push_back(_vehicles.back().end.value());
  }
}

// Helper to get optional array of coordinates.
inline optional_coords_t parse_coordinates(const rapidjson::Value& object,
                                           const char* key){
  if(object.HasMember(key)){
    return {{object[key][0].GetDouble(), object[key][1].GetDouble()}};
  }
  else{
    return boost::none;
  }
}

void input::parse(const std::string& input){
  std::string error_msg;

  // Parsing input.
  if(_json_input.Parse(input.c_str()).HasParseError()){
    std::string error_msg = std::string(rapidjson::GetParseError_En(_json_input.GetParseError()))
      + " (offset: "
      + std::to_string(_json_input.GetErrorOffset())
      + ")";
    throw custom_exception(error_msg);
  }

  // Vehicles check.
  if(!_json_input.HasMember("vehicles")
     or !_json_input["vehicles"].IsArray()
     or _json_input["vehicles"].Empty()){
    throw custom_exception("Incorrect vehicles input.");
  }
  if(_json_input["vehicles"].Size() > 1){
    throw custom_exception("Multiple vehicles are not supported (yet).");
  }

  // Switch input type: explicit matrix or using OSRM.
  if(_json_input.HasMember("matrix")){
    // TODO
  }
  else{
    // Getting vehicle(s).
    if(!_json_input["vehicles"][0].IsObject()){
      throw custom_exception("Ill-formed vehicle object.");
    }
    if(!_json_input["vehicles"][0].HasMember("id")){
      throw custom_exception("Missing mandatory vehicle id.");
    }
    this->add_vehicle(_json_input["vehicles"][0]["id"].GetUint(),
                      parse_coordinates(_json_input["vehicles"][0], "start"),
                      parse_coordinates(_json_input["vehicles"][0], "end"));

    // Getting jobs.
    if(!_json_input.HasMember("jobs")
       or !_json_input["jobs"].IsArray()){
      throw custom_exception("Incorrect jobs input.");
    }
    for(rapidjson::SizeType i = 0; i < _json_input["jobs"].Size(); ++i){
      if(!_json_input["jobs"][i].IsObject()){
        throw custom_exception("Ill-formed job object.");
      }
      if(!_json_input["jobs"][i].HasMember("location")){
        throw custom_exception("Missing mandatory job location.");
      }
      if(!_json_input["jobs"][i].HasMember("id")){
        throw custom_exception("Missing mandatory job id.");
      }

      this->add_job(_json_input["jobs"][i]["id"].GetUint(),
                    parse_coordinates(_json_input["jobs"][i], "location"));
    }

    if(_location_number <= 1){
      throw custom_exception("At least two locations required!");
    }
  }
}
