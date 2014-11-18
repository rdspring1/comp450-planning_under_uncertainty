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

sd:
	ompl_benchmark_statistics.py benchmark_sd0.000000.log -d benchmark_sd0.000000.db -p benchmark_sd0.000000.pdf
	ompl_benchmark_statistics.py benchmark_sd0.010000.log -d benchmark_sd0.010000.db -p benchmark_sd0.010000.pdf
	ompl_benchmark_statistics.py benchmark_sd0.020000.log -d benchmark_sd0.020000.db -p benchmark_sd0.020000.pdf
	ompl_benchmark_statistics.py benchmark_sd0.030000.log -d benchmark_sd0.030000.db -p benchmark_sd0.030000.pdf
	ompl_benchmark_statistics.py benchmark_sd0.040000.log -d benchmark_sd0.040000.db -p benchmark_sd0.040000.pdf
	ompl_benchmark_statistics.py benchmark_sd0.050000.log -d benchmark_sd0.050000.db -p benchmark_sd0.050000.pdf

nodes:
	ompl_benchmark_statistics.py benchmark_nodes10000.log -d benchmark_nodes10000.db -p benchmark_nodes10000.pdf
	ompl_benchmark_statistics.py benchmark_nodes20000.log -d benchmark_nodes20000.db -p benchmark_nodes20000.pdf
	ompl_benchmark_statistics.py benchmark_nodes30000.log -d benchmark_nodes30000.db -p benchmark_nodes30000.pdf
	ompl_benchmark_statistics.py benchmark_nodes40000.log -d benchmark_nodes40000.db -p benchmark_nodes40000.pdf
	ompl_benchmark_statistics.py benchmark_nodes50000.log -d benchmark_nodes50000.db -p benchmark_nodes50000.pdf

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE_FLAGS) $< -o $@
