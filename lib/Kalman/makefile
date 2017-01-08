test: main.o matrix.o
	g++ main.o matrix.o -o test

main.o: main.cpp
	g++ -c main.cpp -Wall

matrix.o: matrix.cpp
	g++ -c matrix.cpp -Wall

clean:
	rm *.o test
