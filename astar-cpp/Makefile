CC = g++
CFLAGS = -std=c++20 -o main

main: main.cpp astar.cpp
	$(CC) $(CFLAGS) main.cpp astar.cpp

optimized: main.cpp astar.cpp
	$(CC) -O3 $(CFLAGS) main main.cpp astar.cpp

clean:
	rm -f main
