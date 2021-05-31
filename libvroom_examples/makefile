CXX ?= g++
CXXFLAGS = -I../src -std=c++17 -Wextra -Wpedantic -Wall -O3
LDLIBS = -L../lib/ -lvroom -lpthread -lssl -lcrypto

# Checking for libglpk based on whether the header file is found as
# glpk does not provide a pkg-config setup.
GLPK_HEADER := $(strip $(wildcard /usr/include/glpk.h))
ifneq ($(GLPK_HEADER),)
	LDLIBS += -lglpk
endif

MAIN = ./libvroom-example
SRC = libvroom.cpp

all : $(MAIN)

$(MAIN) : $(SRC)
	$(CXX) $(CXXFLAGS) $^ $(LDLIBS) -o $@

clean :
	$(RM) $(MAIN)
