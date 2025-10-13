all: out

out: main.o
	clang++ -o out main.o

main.o: main.cpp
	clang++ -std=c++17 -c main.cpp

clear:
	rm -f *.o out a.out main.o rcs_modified.pgm

run:
	./out
