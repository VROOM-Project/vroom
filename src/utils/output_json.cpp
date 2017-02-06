/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "output_json.h"

void write_to_json(const solution& sol,
                   bool geometry,
                   const std::string& output_file){
  // Set output.
  auto start_output = std::chrono::high_resolution_clock::now();
  std::string out = output_file.empty() ? "standard output": output_file;
  BOOST_LOG_TRIVIAL(info) << "[Output] Write solution to "
                          << out << ".";

  auto json_output = sol.to_json(geometry);

  // Rapidjson writing process.
  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> r_writer(s);
  json_output.Accept(r_writer);

  // Write to relevant output.
  if(output_file.empty()){
    // Log to standard output.
    std::cout << s.GetString() << std::endl;
  }
  else{
    // Log to file.
    std::ofstream out_stream (output_file, std::ofstream::out);
    out_stream << s.GetString();
    out_stream.close();
  }
  auto end_output = std::chrono::high_resolution_clock::now();
  auto output_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>
    (end_output - start_output).count();

  BOOST_LOG_TRIVIAL(info) << "[Output] Done, took "
                          << output_duration  << " ms.";
}

