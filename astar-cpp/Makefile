CC = g++
CFLAGS = -std=c++20 -g -o main

main: main.cpp astar.cpp
	$(CC) $(CFLAGS) main.cpp astar.cpp

optimized: main.cpp astar.cpp
	$(CC) -O3 $(CFLAGS) main.cpp astar.cpp

clean:
	rm -f main
