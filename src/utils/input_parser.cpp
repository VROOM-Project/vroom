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
    if ((object[key].Size() < 2) or !object[key][0].IsNumber() or
        !object[key][1].IsNumber()) {
      throw custom_exception("Invalid " + std::string(key) + " array.");
    }
    return optional_coords_t(
      {object[key][0].GetDouble(), object[key][1].GetDouble()});
  } else {
    return boost::none;
  }
}

input parse(const cl_args_t& cl_args) {
  BOOST_LOG_TRIVIAL(info) << "[Loading] Parsing input.";

  // Set relevant wrapper to retrieve the matrix and geometry.
  std::unique_ptr<routing_io<distance_t>> routing_wrapper;
  if (!cl_args.use_libosrm) {
    // Use osrm-routed.
    routing_wrapper = std::make_unique<routed_wrapper>(cl_args.osrm_address,
                                                       cl_args.osrm_port,
                                                       cl_args.osrm_profile);
  } else {
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

  // Main Checks for valid json input.
  if (!json_input.HasMember("jobs") or !json_input["jobs"].IsArray()) {
    throw custom_exception("Invalid jobs.");
  }

  if (!json_input.HasMember("vehicles") or !json_input["vehicles"].IsArray() or
      json_input["vehicles"].Empty()) {
    throw custom_exception("Invalid vehicles.");
  }
  if (!json_input["vehicles"][0].IsObject() or
      !json_input["vehicles"][0].HasMember("id") or
      !json_input["vehicles"][0]["id"].IsUint64()) {
    throw custom_exception("Invalid vehicle at 0.");
  }
  if (json_input["vehicles"].Size() > 1) {
    throw custom_exception("Multiple vehicles are not supported (yet).");
  }

  // Switch input type: explicit matrix or using OSRM.
  if (json_input.HasMember("matrix")) {
    if (!json_input["matrix"].IsArray()) {
      throw custom_exception("Invalid matrix.");
    }

    // Load custom matrix while checking if it is square.
    rapidjson::SizeType matrix_size = json_input["matrix"].Size();
    matrix<distance_t> matrix_input(matrix_size);
    for (rapidjson::SizeType i = 0; i < matrix_size; ++i) {
      if (!json_input["matrix"][i].IsArray() or
          (json_input["matrix"][i].Size() != matrix_size)) {
        throw custom_exception("Invalid matrix line " + std::to_string(i) +
                               ".");
      }
      for (rapidjson::SizeType j = 0; j < matrix_size; ++j) {
        if (!json_input["matrix"][i][j].IsUint()) {
          throw custom_exception("Invalid matrix entry (" + std::to_string(i) +
                                 "," + std::to_string(j) + ").");
        }
        matrix_input[i][j] = json_input["matrix"][i][j].GetUint();
      }
    }
    input_data._matrix = matrix_input;

    // Check if vehicle has start_index or end_index.
    boost::optional<index_t> start_index;
    optional_coords_t start;
    if (json_input["vehicles"][0].HasMember("start_index")) {
      if (!json_input["vehicles"][0]["start_index"].IsUint()) {
        throw custom_exception("Invalid start_index for vehicle at 0.");
      }
      start_index = json_input["vehicles"][0]["start_index"].GetUint();
      if (matrix_size <= start_index.get()) {
        throw custom_exception(
          "start_index exceeding matrix size for vehicle at 0.");
      }

      start = parse_coordinates(json_input["vehicles"][0], "start");
    }

    boost::optional<index_t> end_index;
    optional_coords_t end;
    if (json_input["vehicles"][0].HasMember("end_index")) {
      if (!json_input["vehicles"][0]["end_index"].IsUint()) {
        throw custom_exception("Invalid end_index for vehicle at 0.");
      }
      end_index = json_input["vehicles"][0]["end_index"].GetUint();
      if (matrix_size <= end_index.get()) {
        throw custom_exception(
          "end_index exceeding matrix size for vehicle at 0.");
      }

      end = parse_coordinates(json_input["vehicles"][0], "end");
    }
    // Add vehicle to input
    input_data.add_vehicle(json_input["vehicles"][0]["id"].GetUint(),
                           start,
                           end,
                           start_index,
                           end_index);
    // Add the jobs
    for (rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i) {
      if (!json_input["jobs"][i].IsObject()) {
        throw custom_exception("Invalid job.");
      }
      if (!json_input["jobs"][i].HasMember("id") or
          !json_input["jobs"][i]["id"].IsUint64()) {
        throw custom_exception("Invalid id for job at " + std::to_string(i) +
                               ".");
      }
      if (!json_input["jobs"][i].HasMember("location_index") or
          !json_input["jobs"][i]["location_index"].IsUint()) {
        throw custom_exception("Invalid location_index for job at " +
                               std::to_string(i) + ".");
      }
      if (matrix_size <= json_input["jobs"][i]["location_index"].GetUint()) {
        throw custom_exception(
          "location_index exceeding matrix size for job at " +
          std::to_string(i) + ".");
      }
      input_data.add_job(json_input["jobs"][i]["id"].GetUint64(),
                         parse_coordinates(json_input["jobs"][i], "location"),
                         json_input["jobs"][i]["location_index"].GetUint());
    }
  } else {
    input_data.add_vehicle(json_input["vehicles"][0]["id"].GetUint(),
                           parse_coordinates(json_input["vehicles"][0],
                                             "start"),
                           parse_coordinates(json_input["vehicles"][0], "end"));

    // Getting jobs.
    for (rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i) {
      if (!json_input["jobs"][i].IsObject()) {
        throw custom_exception("Invalid job.");
      }
      if (!json_input["jobs"][i].HasMember("location") or
          !json_input["jobs"][i]["location"].IsArray()) {
        throw custom_exception("Invalid location for job at " +
                               std::to_string(i) + ".");
      }
      if (!json_input["jobs"][i].HasMember("id") or
          !json_input["jobs"][i]["id"].IsUint64()) {
        throw custom_exception("Invalid id for job at " + std::to_string(i) +
                               ".");
      }

      input_data.add_job(json_input["jobs"][i]["id"].GetUint64(),
                         parse_coordinates(json_input["jobs"][i], "location"));
    }
  }

  if (input_data._locations.size() <= 1) {
    throw custom_exception("At least two locations required!");
  }

  return input_data;
}
