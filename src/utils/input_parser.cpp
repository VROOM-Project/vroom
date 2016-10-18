/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./input_parser.h"

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

input parse(const cl_args_t& cl_args){
  // Input json object.
  rapidjson::Document json_input;

  // Custom input object embedding jobs, vehicles and matrix.
  input input_data;

  // Set relevant wrapper to retrieve the matrix and geometry.
  std::unique_ptr<routing_io<distance_t>> routing_wrapper;
  if(!cl_args.use_libosrm){
    // Use osrm-routed.
    routing_wrapper
      = std::make_unique<routed_wrapper>(cl_args.osrm_address,
                                         cl_args.osrm_port,
                                         cl_args.osrm_profile);
  }
  else{
#if LIBOSRM
    // Use libosrm.
    if(cl_args.osrm_profile.empty()){
      throw custom_exception("-l flag requires -m.");
    }
    routing_wrapper
      = std::make_unique<libosrm_wrapper>(cl_args.osrm_profile);
#else
    throw custom_exception("libosrm must be installed to use -l.");
#endif
  }
  input_data.set_routing(std::move(routing_wrapper));

  // Parsing input string to populate the input object.
  if(json_input.Parse(cl_args.input.c_str()).HasParseError()){
    std::string error_msg = std::string(rapidjson::GetParseError_En(json_input.GetParseError()))
      + " (offset: "
      + std::to_string(json_input.GetErrorOffset())
      + ")";
    throw custom_exception(error_msg);
  }

  // Vehicles check.
  if(!json_input.HasMember("vehicles")
     or !json_input["vehicles"].IsArray()
     or json_input["vehicles"].Empty()){
    throw custom_exception("Incorrect vehicles input.");
  }
  if(json_input["vehicles"].Size() > 1){
    throw custom_exception("Multiple vehicles are not supported (yet).");
  }

  // Switch input type: explicit matrix or using OSRM.
  if(json_input.HasMember("matrix")){
    // TODO
  }
  else{
    // Getting vehicle(s).
    if(!json_input["vehicles"][0].IsObject()){
      throw custom_exception("Ill-formed vehicle object.");
    }
    if(!json_input["vehicles"][0].HasMember("id")){
      throw custom_exception("Missing mandatory vehicle id.");
    }
    input_data.add_vehicle(json_input["vehicles"][0]["id"].GetUint(),
                           parse_coordinates(json_input["vehicles"][0],
                                             "start"),
                           parse_coordinates(json_input["vehicles"][0],
                                             "end"));

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

      input_data.add_job(json_input["jobs"][i]["id"].GetUint(),
                         parse_coordinates(json_input["jobs"][i],
                                           "location"));
    }

    if(input_data.get_location_number() <= 1){
      throw custom_exception("At least two locations required!");
    }

    input_data.set_matrix();
  }

  return input_data;
}
