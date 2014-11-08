OMPL_DIR = /usr
#CXXFLAGS = -O2 # change to -g when debugging code
CXXFLAGS = -g -std=c++11
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app -lboost_program_options -lboost_system -lboost_timer
CXX=c++

SMR: ODESolverAndControlsWithRigidBodyPlanning.o SMR.o
	$(CXX) $(CXXFLAGS) -o SMR ODESolverAndControlsWithRigidBodyPlanning.o SMR.o $(LD_FLAGS)

clean:
	rm -rf *.o
	rm -rf *.db
	rm -rf path.txt
	rm -rf SMR

plots:
	ompl_benchmark_statistics.py benchmark.log -d benchmark.db -p benchmark.pdf

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE_FLAGS) $< -o $@
