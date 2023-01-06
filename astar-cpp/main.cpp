#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include "astar.h"

AStar* import_from_file(std::string filename) {
    // To run Astar, we need:
    //  - a 3D occupancy grid
    //  - a start point
    //  - a goal point
    //  - some way to represent the points in the occupancy grid
    //     (essentially we need to know how to map the coordinate space to the occupancy grid)

    // To start, we get all the points from the file
    Point tl_corner;
    Point br_corner;

    Point start;
    Point goal;

    std::vector<Point> obstacles;

    // Now we need to read the file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Could not open file " << filename << std::endl;
        return nullptr;
    }

    std::string line;

    // The first line is the version of the file which we will store
    int version;
    getline(file, line);
    version = std::stoi(line.substr(line.find("=") + 1, line.length() - 1));

    // Now we get the tl_corner
    tl_corner = Point();
    getline(file, line);
    std::stringstream ss(line);

    std::string token;
    getline(ss, token, ' ');
    tl_corner.x = std::stod(token);

    getline(ss, token, ' ');
    tl_corner.y = std::stod(token);

    getline(ss, token, ' ');
    tl_corner.z = std::stod(token);

    // Now we get the br_corner
    br_corner = Point();
    getline(file, line);
    ss = std::stringstream(line);

    getline(ss, token, ' ');
    br_corner.x = std::stod(token);

    getline(ss, token, ' ');
    br_corner.y = std::stod(token);

    getline(ss, token, ' ');
    br_corner.z = std::stod(token);

    // Now we get the start point
    start = Point();
    getline(file, line);
    ss = std::stringstream(line);

    getline(ss, token, ' ');
    start.x = std::stod(token);

    getline(ss, token, ' ');
    start.y = std::stod(token);

    getline(ss, token, ' ');
    start.z = std::stod(token);

    // Now we get the goal point
    goal = Point();
    getline(file, line);

    ss = std::stringstream(line);

    getline(ss, token, ' ');
    goal.x = std::stod(token);

    getline(ss, token, ' ');
    goal.y = std::stod(token);

    getline(ss, token, ' ');
    goal.z = std::stod(token);

    // Now we get the obstacles
    getline(file, line);
    int num_obstacles = std::stoi(line);

    for (int i = 0; i < num_obstacles; i++) {
        getline(file, line);
        ss = std::stringstream(line);

        Point obstacle = Point();
        getline(ss, token, ' ');
        obstacle.x = std::stod(token);

        getline(ss, token, ' ');
        obstacle.y = std::stod(token);

        getline(ss, token, ' ');
        obstacle.z = std::stod(token);

        obstacles.push_back(obstacle);
    }

    // Now we have all the data we need to run Astar
    // Lets print everything out to make sure it is correct
    std::cout << "  == Imported Data from '" << filename << "' ==" << std::endl;
    std::cout << "Version: " << version << std::endl;
    std::cout << "Top Left Corner: " << tl_corner << std::endl;
    std::cout << "Bottom Right Corner: " << br_corner << std::endl;

    std::cout << "Start: " << start << std::endl;
    std::cout << "Goal: " << goal << std::endl;

    std::cout << "Obstacles: " << std::endl;
    for (auto obstacle : obstacles) {
        std::cout << obstacle << std::endl;
    }

    // Lets make the astar class
    int num_cells = 20;
    return new AStar(tl_corner, br_corner, num_cells, start, goal, obstacles);
}

int main() {
    AStar * pathfinder = import_from_file("point-cloud.cld");
    if (pathfinder == nullptr) {
        return 1;
    }

    // get the path
    std::vector<Point> path = pathfinder->run();

    // print the path
    if (path.empty()) {
        std::cout << "No path found" << std::endl;
        return 1;
    }

    std::cout << "Path: " << std::endl;
    for (auto point : path) {
        std::cout << point << std::endl;
    }

    delete pathfinder;
    return 0;
}