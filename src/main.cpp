/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <sstream>
#include <unistd.h>
#include "./structures/typedefs.h"
#include "./heuristics/tsp_strategy.h"

void display_usage(){
  std::string usage = "VROOM Copyright (C) 2015, Julien Coupey\n";
  usage += "Usage :\n\tvroom [OPTION]... \"loc=lat,lon&loc=lat,lon[&loc=lat,lon...]\"";
  usage += "\n\tvroom [OPTION]... -i FILE\n";
  usage += "Options:\n";
  usage += "\t-a=ADDRESS\t OSRM server address (\"0.0.0.0\")\n";
  usage += "\t-p=PORT,\t OSRM listening port (5000)\n";
  usage += "\t-g,\t\t get detailed route geometry for the solution\n";
  usage += "\t-o=OUTPUT,\t output file name\n";
  usage += "\t-i=FILE,\t read input from FILE rather than from\n\t\t\t command-line\n";
  usage += "\t-t,\t\t read input file from -i option as TSPLIB format\n";
  usage += "\t-v,\t\t turn on verbose output\n";
  usage += "\nThis program is distributed under the terms of the GNU General Public\n";
  usage += "License, version 3, and comes with ABSOLUTELY NO WARRANTY.\n";
  std::cout << usage << std::endl;
  exit(0);
}

int main(int argc, char **argv){
  // Default options.
  cl_args_t cl_args;
  cl_args.geometry = false;
  cl_args.osrm_address = "0.0.0.0";
  cl_args.osrm_port = "5000";
  cl_args.use_osrm = true;
  cl_args.use_tsplib = false;
  cl_args.verbose = false;

  // Parsing command-line arguments.
  const char* optString = "a:gi:o:p:tvh?";
  int opt = getopt(argc, argv, optString);

  while(opt != -1) {
    switch(opt){
    case 'a':
      cl_args.osrm_address = optarg;
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
    case 't':
      cl_args.use_tsplib = true;
      cl_args.use_osrm = false;
      break;
    case 'v':
      cl_args.verbose = true;
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
  
  try{
    solve_atsp(cl_args);
  }
  catch(const custom_exception& e){
    std::cerr << "Error: " << e.get_message() << std::endl;
    exit(1);
  }

  return 0;
}
