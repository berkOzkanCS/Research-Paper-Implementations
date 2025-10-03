all: out

out: main.o
	clang++ -o out main.o

main.o: main.cpp
	clang++ -std=c++17 -I /usr/local/Cellar/eigen/3.4.0_1/include/eigen3 -c main.cpp

clear:
	rm -f *.o out *.txt a.out main.o 

run:
	./out

show:
	python3 graph.py

reset:
	make clear
	make
	make run
	make show