tests: tests.o platform.o
	g++ -Wall tests.o platform.o -o tests

tests.o: tests.cpp platform.h
	g++ -Wall -c tests.cpp -o tests.o

platform.o: platform.cpp platform.h
	g++ -Wall -c platform.cpp -o platform.o
