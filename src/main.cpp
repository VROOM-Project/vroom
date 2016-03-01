/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <sstream>
#include <unistd.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include "./structures/typedefs.h"
#include "./heuristics/tsp_strategy.h"

void display_usage(){
  std::string usage = "VROOM Copyright (C) 2015-2016, Julien Coupey\n";
  usage += "Usage :\n\tvroom [OPTION]... \"loc=lat,lon&loc=lat,lon[&loc=lat,lon...]\"";
  usage += "\n\tvroom [OPTION]... -i FILE\n";
  usage += "Options:\n";
  usage += "\t-a=ADDRESS\t OSRM server address (\"0.0.0.0\")\n";
  usage += "\t-p=PORT,\t OSRM listening port (5000)\n";
  usage += "\t-g,\t\t get detailed route geometry for the solution\n";
  usage += "\t-i=FILE,\t read input from FILE rather than from\n\t\t\t command-line\n";
  usage += "\t-o=OUTPUT,\t output file name\n";
  usage += "\t-s,\t\t compute an \"open\" route (not a tour), starting at\n\t\t\t the first input location\n";
  usage += "\t-e,\t\t compute an \"open\" route (not a tour), ending at\n\t\t\t the last input location\n";
  usage += "\t-t,\t\t number of threads to use\n";
  usage += "\t-v,\t\t turn on verbose output\n";
  usage += "\t-V,\t\t turn on verbose output with all details";
  std::cout << usage << std::endl;
  exit(0);
}

int main(int argc, char **argv){
  // Log formatting.
  boost::log::add_console_log(std::cout,
                              boost::log::keywords::format = "%Message%");

  // Load default command-line options.
  cl_args_t cl_args;

  // Parsing command-line arguments.
  const char* optString = "a:egi:o:p:st:vVh?";
  int opt = getopt(argc, argv, optString);

  while(opt != -1) {
    switch(opt){
    case 'a':
      cl_args.osrm_address = optarg;
      break;
    case 'e':
      cl_args.force_end = true;
      break;
    case 'g':
      cl_args.geometry = true;
      break;
    case 'h':
      display_usage();
      break;
    case 'i':
      cl_args.input_file = optarg;
      break;
    case 'o':
      cl_args.output_file = optarg;
      break;
    case 'p':
      cl_args.osrm_port = optarg;
      break;
    case 's':
      cl_args.force_start = true;
      break;
    case 't':
      try{
        cl_args.nb_threads = std::stoul(optarg);
      }
      catch(const std::exception& e){
        BOOST_LOG_TRIVIAL(error) << "[Error] Wrong value for number of threads.";
        exit(1);
      }
      break;
    case 'v':
      cl_args.log_level = boost::log::trivial::info;
      break;
    case 'V':
      cl_args.log_level = boost::log::trivial::trace;
      break;
    default:
      break;
    }
    opt = getopt(argc, argv, optString);
  }

  if(cl_args.input_file.empty()){
    // Getting input from command-line.
    if(argc == optind){
      // Missing argument!
      display_usage();
    }
    cl_args.input = argv[optind];
  }
  else{
    // Getting input from provided file.
    std::ifstream ifs (cl_args.input_file);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    cl_args.input = buffer.str();
  }
  cl_args.use_osrm = (cl_args.input.find("DIMENSION") == std::string::npos);

  // Log level.
  boost::log::core::get()
    ->set_filter(boost::log::trivial::severity >= cl_args.log_level);
  
  try{
    solve_atsp(cl_args);
  }
  catch(const custom_exception& e){
    BOOST_LOG_TRIVIAL(error) << "[Error] " << e.get_message();
    exit(1);
  }

  return 0;
}
