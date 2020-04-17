CXX ?= g++
CXXFLAGS = -I../src -std=c++17 -Wextra -Wpedantic -Wall -O3
LDLIBS = -L../lib/ -lvroom -lboost_system -lpthread -lssl -lcrypto

MAIN = ./libvroom-example
SRC = libvroom.cpp

all : $(MAIN)

$(MAIN) : $(SRC)
	$(CXX) $(CXXFLAGS) $^ $(LDLIBS) -o $@

clean :
	$(RM) $(MAIN)
