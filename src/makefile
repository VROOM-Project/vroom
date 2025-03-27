# This file is part of VROOM.
#
# Copyright (c) 2015-2025, Julien Coupey.
# All rights reserved (see LICENSE).

# Variables.
CXX ?= g++
USE_ROUTING ?= true
CXXFLAGS = -MMD -MP -I. -std=c++20 -Wextra -Wpedantic -Wall -O3 -DASIO_STANDALONE -DUSE_ROUTING=$(USE_ROUTING)
LDLIBS = -lpthread

# Using all cpp files in current directory.
MAIN = ../bin/vroom
LIB = ../lib/libvroom.a
SRC = $(wildcard *.cpp)\
			$(wildcard ./algorithms/*.cpp)\
			$(wildcard ./algorithms/*/*.cpp)\
			$(wildcard ./routing/*.cpp)\
			$(wildcard ./problems/*.cpp)\
			$(wildcard ./problems/*/*.cpp)\
			$(wildcard ./problems/*/*/*.cpp)\
			$(wildcard ./structures/generic/*.cpp)\
			$(wildcard ./structures/vroom/*.cpp)\
			$(wildcard ./structures/vroom/input/*.cpp)\
			$(wildcard ./structures/vroom/solution/*.cpp)\
			$(wildcard ./structures/*.cpp)\
			$(wildcard ./utils/*.cpp)

# Remove routing if we don't require it
ifeq ($(USE_ROUTING),false)
	SRC := $(filter-out $(wildcard ./routing/*.cpp), $(SRC))
else
	LDLIBS += -lssl -lcrypto

	# Checking for libosrm
	ifeq ($(shell pkg-config --exists libosrm && echo 1),1)
		LDLIBS += $(shell pkg-config --libs libosrm) -lboost_system -lboost_filesystem -lboost_iostreams -lboost_thread -lrt -ltbb
		LIBOSRMFLAGS = $(shell pkg-config --cflags libosrm)
		CXXFLAGS += $(filter-out -std=c++17, $(LIBOSRMFLAGS)) -D USE_LIBOSRM=true
	else
		SRC := $(filter-out ./routing/libosrm_wrapper.cpp, $(SRC))
	endif
endif

# Checking for libglpk based on whether the header file is found as
# glpk does not provide a pkg-config setup.
GLPK_HEADER := $(strip $(wildcard /usr/include/glpk.h))
ifeq ($(GLPK_HEADER),)
	SRC := $(filter-out ./algorithms/validation/check.cpp, $(SRC))
	SRC := $(filter-out ./algorithms/validation/choose_ETA.cpp, $(SRC))
else
	LDLIBS += -lglpk
	CXXFLAGS += -D USE_LIBGLPK=true
endif

OBJ = $(SRC:.cpp=.o)
DEPS = $(SRC:.cpp=.d)

# Main target.
all : $(MAIN) $(LIB)

# Shared target, for creating Position Independent Code (PIC)
shared : CXXFLAGS += -fPIC
shared : all

$(MAIN) : $(OBJ) main.o
	mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

$(LIB) : $(OBJ)
	mkdir -p $(@D)
	$(AR) cr $@ $^

# Building .o files.
%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include ${DEPS}

clean :
	$(RM) $(OBJ) $(DEPS)
	$(RM) $(MAIN)
	$(RM) $(LIB)

tidy : $(SRC)
	clang-tidy $(SRC) -fix -- $(CXXFLAGS) -I/usr/include/c++/11/ -I/usr/include/x86_64-linux-gnu/c++/11/
