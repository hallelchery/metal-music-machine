CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I.

# Source files for the simulator target
SIM_SRCS = sim/main.cpp \
           src/fsm.cpp \
           hal/hal_sim.cpp

SIM_OUT  = sim_mmm

.PHONY: sim clean

sim:
	$(CXX) $(CXXFLAGS) $(SIM_SRCS) -o $(SIM_OUT)
	@echo "Build successful. Run with: ./$(SIM_OUT)"

clean:
	rm -f $(SIM_OUT) telemetry.csv