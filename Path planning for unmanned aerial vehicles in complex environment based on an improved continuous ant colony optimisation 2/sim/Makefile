CXX      = clang++
CXXFLAGS = -g -O0 -std=c++17 -I /usr/local/Cellar/eigen/3.4.0_1/include/eigen3

all: main.o sim.o algo.o
	$(CXX) -o out main.o sim.o algo.o

main.o: main.cpp sim.hpp
	$(CXX) $(CXXFLAGS) -c main.cpp

sim.o: sim.hpp sim.cpp ../algo.cpp
	$(CXX) $(CXXFLAGS) -c sim.cpp 
	$(CXX) $(CXXFLAGS) -c ../algo.cpp

clear:
	rm -f *.o out *.txt

run:
	./out "heightmap.png" 0

show:
	python3 graph.py

reset:
	make clear
	make
	make run
	make show

