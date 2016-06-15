# This file is part of VROOM.
# 
# Copyright (c) 2015-2016, Julien Coupey.
# All rights reserved (see LICENSE).

# Variables.
CXX = g++
CXXFLAGS = -std=c++14 -Wpedantic -Wall -O3 -DBOOST_LOG_DYN_LINK
LDLIBS = -lboost_system -lboost_regex -lboost_log -lboost_log_setup -lpthread -lboost_thread

# Checking for libosrm
ifeq ($(shell pkg-config --exists libosrm && echo 1),1)
LDLIBS += $(shell pkg-config --libs libosrm) -lboost_filesystem -lboost_iostreams -lrt
CXXFLAGS += $(shell pkg-config --cflags libosrm) -D LIBOSRM=true
endif

# Using all cpp files in current directory.
MAIN = ../bin/vroom
SRC = $(wildcard *.cpp)\
			$(wildcard ./algorithms/*.cpp)\
			$(wildcard ./heuristics/*.cpp)\
			$(wildcard ./structures/*.cpp)\
			$(wildcard ./utils/*.cpp)
OBJ = $(SRC:.cpp=.o)

# Main target.
all : $(MAIN)

$(MAIN) : $(OBJ) main.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

# Building .o files.
%.o : %.cpp %.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Specific target for main (no .h file).
main.o : main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean : 
	rm $(OBJ)
	rm $(MAIN)
