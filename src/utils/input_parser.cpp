/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./input_parser.h"

// Helper to get optional array of coordinates.
inline optional_coords_t parse_coordinates(const rapidjson::Value& object,
                                           const char* key) {
  if (object.HasMember(key) and object[key].IsArray()) {
    if (object[key].Size() < 2) {
      throw custom_exception("Invalid coordinates array size.");
    }
    return optional_coords_t({object[key][0].GetDouble(),
          object[key][1].GetDouble()});
  }
  else {
    return boost::none;
  }
}

input parse(const cl_args_t& cl_args) {
  BOOST_LOG_TRIVIAL(info) << "[Loading] Parsing input.";

  // Set relevant wrapper to retrieve the matrix and geometry.
  std::unique_ptr<routing_io<distance_t>> routing_wrapper;
  if (!cl_args.use_libosrm) {
    // Use osrm-routed.
    routing_wrapper =
      std::make_unique<routed_wrapper>(cl_args.osrm_address,
                                       cl_args.osrm_port,
                                       cl_args.osrm_profile);
  }
  else {
#if LIBOSRM
    // Use libosrm.
    if (cl_args.osrm_profile.empty()) {
      throw custom_exception("-l flag requires -m.");
    }
    routing_wrapper = std::make_unique<libosrm_wrapper>(cl_args.osrm_profile);
#else
    throw custom_exception("libosrm must be installed to use -l.");
#endif
  }

  // Custom input object embedding jobs, vehicles and matrix.
  input input_data(std::move(routing_wrapper), cl_args.geometry);

  // Input json object.
  rapidjson::Document json_input;

  // Parsing input string to populate the input object.
  if (json_input.Parse(cl_args.input.c_str()).HasParseError()) {
    std::string error_msg =
      std::string(rapidjson::GetParseError_En(json_input.GetParseError())) +
      " (offset: " + std::to_string(json_input.GetErrorOffset()) + ")";
    throw custom_exception(error_msg);
  }

  // Checks, that needs to be done for both modes.
  // Vehicles check.
  if (!json_input.HasMember("vehicles")
     or !json_input["vehicles"].IsArray()
     or json_input["vehicles"].Empty()) {
    throw custom_exception("Incorrect vehicles input.");
  }
  if (!json_input["vehicles"][0].IsObject()) {
    throw custom_exception("Ill-formed vehicle object.");
  }
  if (!json_input["vehicles"][0].HasMember("id")) {
    throw custom_exception("Missing mandatory vehicle id.");
  }
  if (json_input["vehicles"].Size() > 1) {
    throw custom_exception("Multiple vehicles are not supported (yet).");
  }
  // Switch input type: explicit matrix or using OSRM.
  if (json_input.HasMember("matrix")) {
    //
    // ---Custom-Matrix-mode---
    // Load JSON-matrix into input, while checking, if matrix is quadratic.
    rapidjson::SizeType matrix_size = json_input["matrix"].Size();
    matrix<distance_t> matrix_input(matrix_size);
    for (rapidjson::SizeType i = 0; i < matrix_size; ++i) {
      if (json_input["matrix"][i].Size() != matrix_size) {
        throw custom_exception("JSON-matrix is not quadratic.");
      }
      for (rapidjson::SizeType j = 0; j < matrix_size; ++j) {
        if (!json_input["matrix"][i][j].IsNumber()) {
          throw custom_exception("JSON-matrix-entry is not a number.");
        }
        matrix_input[i][j] = json_input["matrix"][i][j].GetUint();
      }
    }
    input_data._matrix = matrix_input;
    // Check, if vehicle has start_index or end_index 
    boost::optional<index_t> start_index; 
    if (json_input["vehicles"][0].HasMember("start_index")) {
      if (!json_input["vehicles"][0]["start_index"].IsNumber()) {
        throw custom_exception("Vehicle attribute 'start_index' is not a number.");
      }
      start_index = json_input["vehicles"][0]["start_index"].GetUint();
      if (matrix_size <= start_index.get()) {
        throw custom_exception("Vehicle start_index does not match to matrix size.");
      }
    }
    boost::optional<index_t> end_index;
    if (json_input["vehicles"][0].HasMember("end_index")) {
      if (!json_input["vehicles"][0]["end_index"].IsNumber()) {
        throw custom_exception("Vehicle attribute 'end_index' is not a number.");
      }
      end_index = json_input["vehicles"][0]["end_index"].GetUint();
      if (matrix_size <= end_index.get()) {
        throw custom_exception("Vehicle end_index does not match to matrix size.");
      }
    }
    // Add vehicle to input
    input_data.add_vehicle(json_input["vehicles"][0]["id"].GetUint(),
                           start_index,
                           end_index,
                           parse_coordinates(json_input["vehicles"][0],
                                             "start"),
                           parse_coordinates(json_input["vehicles"][0],
                                             "end"));
    // Add the jobs
    if (!json_input.HasMember("jobs")) {
        // Compute generic jobs, if not provided by input
        throw custom_exception("Vehicle end_index does not match to matrix size.");
    }
    for (rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i) {
      if (!json_input["jobs"][i].IsObject()){
        throw custom_exception("Ill-formed job object.");
      }
      if (!json_input["jobs"][i].HasMember("id")) {
        throw custom_exception("Missing mandatory job id.");
      }
      if (!json_input["jobs"][i]["id"].IsNumber()) {
        throw custom_exception("Job id is not a number.");
      }
      if (!json_input["jobs"][i].HasMember("location_index")) {
        throw custom_exception("Missing mandatory job location_index for custom_matrix.");
      }
      if (!json_input["jobs"][i]["location_index"].IsNumber()) {
        throw custom_exception("Job location_index is not a number.");
      }
      if (matrix_size <= json_input["jobs"][i]["location_index"].GetUint()) {
        throw custom_exception("Job location_index does not match to matrix size.");
      }
      input_data.add_job( json_input["jobs"][i]["id"].GetUint(),
                        parse_coordinates( json_input["jobs"][i],"location" ),
                        json_input["jobs"][i]["location_index"].GetUint() );
    }

  }
  else {
    //
    // ---OSRM-mode---
    // add vehicle to input
    input_data.add_vehicle(json_input["vehicles"][0]["id"].GetUint(),
                           parse_coordinates(json_input["vehicles"][0],
                                             "start"),
                           parse_coordinates(json_input["vehicles"][0],
                                             "end"));

    // Getting jobs.
    if (!json_input.HasMember("jobs") or !json_input["jobs"].IsArray()) {
      throw custom_exception("Incorrect jobs input.");
    }
    for (rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i) {
      if (!json_input["jobs"][i].IsObject()) {
        throw custom_exception("Ill-formed job object.");
      }
      if (!json_input["jobs"][i].HasMember("location")) {
        throw custom_exception("Missing mandatory job location.");
      }
      if (!json_input["jobs"][i].HasMember("id")) {
        throw custom_exception("Missing mandatory job id.");
      }

      input_data.add_job(json_input["jobs"][i]["id"].GetUint(),
                         parse_coordinates(json_input["jobs"][i], "location"));
    }
  }
  //final check, if enough jobs aquired
  if(input_data.get_location_number() <= 1){
    throw custom_exception("At least two locations required!");
  }
  return input_data;
}
