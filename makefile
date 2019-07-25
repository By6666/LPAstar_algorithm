exe: main.o LPAstar_algorithm.o grid_input.o
	g++ -o exe.out main.o LPAstar_algorithm.o grid_input.o

main.o: main.cpp
	g++ -c main.cpp -std=c++11

LPAstar_algorithm.o: LPAstar_algorithm.cpp
	g++ -c LPAstar_algorithm.cpp -std=c++11

grid_input.o: grid_input.cpp
	g++ -c grid_input.cpp

clean:
	rm -f *.o
