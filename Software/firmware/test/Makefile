# Host unit tests for the hardware-free TCU code.
# Build and run:  make && ./test_tcu
# CI uses the system g++. Any C++17 compiler works: make CXX=clang++

CXX      ?= g++
# OPT can be overridden, e.g. OPT="-O1 -g -fsanitize=undefined,address"
OPT      ?= -O1 -g
CXXFLAGS  = $(OPT) -std=c++17 -Wall -Wextra -pedantic -I../include -MMD -MP
BINARY    = test_tcu
CORE      = caltypes.o cal_330e.o cal_f11.o cal_v095.o calset.o canif.o drivemode.o gears.o interlock.o max22200.o eds.o sas.o \
            engage.o speedsens.o tcu.o valveout.o anaconv.o linpump.o \
            shifter.o
TESTS     = test_main.o test_gears.o test_interlock.o test_lookup.o test_eds.o \
            test_max22200.o test_sas.o test_engage.o test_speedsens.o \
            test_tcu.o test_valveout.o test_anaconv.o test_linpump.o \
            test_shifter.o test_canif.o test_params.o
VPATH     = ../src

all: $(BINARY)

$(BINARY): $(CORE) $(TESTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

run: $(BINARY)
	./$(BINARY)

clean:
	rm -f *.o *.d $(BINARY)

-include $(wildcard *.d)

.PHONY: all run clean
