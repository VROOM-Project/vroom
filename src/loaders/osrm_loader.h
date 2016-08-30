#ifndef OSRM_LOADER_H
#define OSRM_LOADER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>
#include <limits>
#include <regex>
#include "../../include/rapidjson/error/en.h"
#include "./problem_io.h"
#include "../structures/matrix.h"
#include "../utils/exceptions.h"

class osrm_loader : public problem_io<distance_t>{

protected:
  enum class LOC_TYPE {START, END, JOB};
  struct Location {LOC_TYPE type; double lon; double lat; index_t job_id;};

  const std::string _osrm_profile; // OSRM profile name
  const bool _use_osrm_v5;         // For backward compat
  std::vector<Location> _locations;

  static distance_t round_to_distance(double value){
    return static_cast<distance_t>(value + 0.5);
  }

  void add_location(LOC_TYPE type, const rapidjson::Value& location, index_t job_id = 0){
    if(!location.IsArray()
       or location.Size() != 2
       or !location[0].IsNumber()
       or !location[1].IsNumber()){
      throw custom_exception("Invalid input location");
    }
    _locations.push_back({type, location[0].GetDouble(), location[1].GetDouble(), job_id});
  }

  osrm_loader(const std::string& osrm_profile,
              const std::string& input):
    _osrm_profile(osrm_profile),
    _use_osrm_v5(!_osrm_profile.empty())
  {
    rapidjson::Document json_input;
    std::string error_msg;

    // Parsing input.
    if(json_input.Parse(input.c_str()).HasParseError()){
      std::string error_msg = std::string(rapidjson::GetParseError_En(json_input.GetParseError()))
        + " (offset: "
        + std::to_string(json_input.GetErrorOffset())
        + ")";
      throw custom_exception(error_msg);
    }

    // Getting vehicle(s).
    if(!json_input.HasMember("vehicles")
       or !json_input["vehicles"].IsArray()
       or json_input["vehicles"].Empty()){
      throw custom_exception("Incorrect vehicles input.");
    }
    if(json_input["vehicles"].Size() > 1){
      throw custom_exception("Multiple vehicles are not supported (yet).");
    }
    if(!json_input["vehicles"][0].IsObject()){
      throw custom_exception("Ill-formed vehicle object.");
    }
    if(!json_input["vehicles"][0].HasMember("id")){
      throw custom_exception("Missing mandatory vehicle id.");
    }
    _vehicle_id = json_input["vehicles"][0]["id"].GetUint();

    // Add optional vehicle start.
    _pbl_context.force_start = json_input["vehicles"][0].HasMember("start");
    if(_pbl_context.force_start){
      // Remember the index of the start loc to be added.
      _pbl_context.start = _locations.size();
      this->add_location(LOC_TYPE::START, json_input["vehicles"][0]["start"]);
    }

    // Getting jobs.
    if(!json_input.HasMember("jobs")
       or !json_input["jobs"].IsArray()){
      throw custom_exception("Incorrect jobs input.");
    }
    for(rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i){
      if(!json_input["jobs"][i].IsObject()){
        throw custom_exception("Ill-formed job object.");
      }
      if(!json_input["jobs"][i].HasMember("location")){
        throw custom_exception("Missing mandatory job location.");
      }
      if(!json_input["jobs"][i].HasMember("id")){
        throw custom_exception("Missing mandatory job id.");
      }

      this->add_location(LOC_TYPE::JOB,
                         json_input["jobs"][i]["location"],
                         json_input["jobs"][i]["id"].GetUint());
    }

    // Add optional vehicle end as last value in _locations.
    _pbl_context.force_end = json_input["vehicles"][0].HasMember("end");

    if(_pbl_context.force_end){
      // Remember the index of the end loc to be added.
      _pbl_context.end = _locations.size();
      this->add_location(LOC_TYPE::END, json_input["vehicles"][0]["end"]);
    }

    if(_locations.size() <= 1){
      throw custom_exception("At least two locations required!");
    }

    if(!_pbl_context.force_start && !_pbl_context.force_end){
      throw custom_exception("No start or end specified for vehicle "
                             + std::to_string(_vehicle_id)
                             + '.');
    }
  }

  virtual matrix<distance_t> get_matrix() const = 0;

  inline void add_json_step(index_t step_id,
                            std::string type,
                            rapidjson::Value& steps_array,
                            rapidjson::Document::AllocatorType& allocator) const{
    rapidjson::Value json_step(rapidjson::kObjectType);
    json_step.AddMember("type", rapidjson::Value(), allocator);
    json_step["type"].SetString(type.c_str(), type.size(), allocator);

    // Location coordinates.
    json_step.AddMember("location",
                        rapidjson::Value(rapidjson::kArrayType).Move(),
                        allocator);
    json_step["location"].PushBack(_locations[step_id].lon, allocator);
    json_step["location"].PushBack(_locations[step_id].lat, allocator);

    if(_locations[step_id].type == LOC_TYPE::JOB){
      json_step.AddMember("job", _locations[step_id].job_id, allocator);
    }

    steps_array.PushBack(json_step, allocator);
  }

  inline void check_unfound(const std::vector<unsigned>& nb_unfound_from_loc,
                            const std::vector<unsigned>& nb_unfound_to_loc) const{
    assert(nb_unfound_from_loc.size() == nb_unfound_to_loc.size());
    unsigned max_unfound_routes_for_a_loc = 0;
    index_t error_loc = 0;    // Initial value never actually used.
    std::string error_direction;
    // Finding the "worst" location for unfound routes.
    for(unsigned i = 0; i < nb_unfound_from_loc.size(); ++i){
      if(nb_unfound_from_loc[i] > max_unfound_routes_for_a_loc){
        max_unfound_routes_for_a_loc = nb_unfound_from_loc[i];
        error_loc = i;
        error_direction = "from";
      }
      if(nb_unfound_to_loc[i] > max_unfound_routes_for_a_loc){
        max_unfound_routes_for_a_loc = nb_unfound_to_loc[i];
        error_loc = i;
        error_direction = "to";
      }
    }
    if(max_unfound_routes_for_a_loc > 0){
      std::string error_msg = "OSRM has unfound route(s) ";
      switch (_locations[error_loc].type){
      case LOC_TYPE::START:
        error_msg += "from vehicle start";
        break;
      case LOC_TYPE::END:
        error_msg += "to vehicle end";
        break;
      case LOC_TYPE::JOB:
        error_msg += error_direction;
        error_msg += " job ";
        error_msg += std::to_string(_locations[error_loc].job_id);
        break;
      }
      throw custom_exception(error_msg);
    }
  }

  virtual void get_steps(const std::list<index_t>& steps,
                         rapidjson::Value& value,
                         rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value steps_array(rapidjson::kArrayType);
    for(auto const& step_id: steps){

      // Step type
      std::string type;
      switch (_locations[step_id].type){
      case LOC_TYPE::START:
        type = "start";
        break;
      case LOC_TYPE::END:
        type = "end";
        break;
      case LOC_TYPE::JOB:
        type = "job";
        break;
      }

      add_json_step(step_id, type, steps_array, allocator);
    }

    value.Swap(steps_array);
  }

  virtual void get_route_infos(const std::list<index_t>& steps,
                               rapidjson::Value& value,
                               rapidjson::Document::AllocatorType& allocator) const = 0;
};

#endif
