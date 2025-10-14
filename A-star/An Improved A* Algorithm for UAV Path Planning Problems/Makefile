CXX = clang++
CXXFLAGS = -g -O0 -std=c++17 -I /usr/local/Cellar/eigen/3.4.0_1/include/eigen3

all: main.o algo.o
	$(CXX) $(CXXFLAGS) -o out -c main.cpp

algo.o: algo.hpp algo.cpp
	$(CXX) $(CXXFLAGS) -c algo.cpp

clear:
	rm -f *.o *.out *.txt algo main out
	rm -rf algo.dSYM main.dSYM
	
run:
	./out

show:
	python3 graph.py

reset:
	make clear
	make
	make run
	make show