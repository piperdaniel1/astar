#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <ostream>

enum PointType {
    Obstacle,
    Start,
    Goal,
};

// Represents a point in space
class Point {
    public:
    PointType type;
    int x;
    int y;
    int z;

    // cout << operator for Point
    friend std::ostream& operator<<(std::ostream& os, const Point& point) {
        os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
        return os;
    }
};

// Represents a point in the AStar grid
// Accordingly, x y and z are the coordinates of the point
class AStarPoint {
    public:
    int x;
    int y;
    int z;
    int prev_cost;
    int est_cost;
    AStarPoint* parent;

    // cout << operator for AStarPoint
    friend std::ostream& operator<<(std::ostream& os, const AStarPoint& point) {
        os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
        return os;
    }
};

class AStar {
    public:
    AStar();
    ~AStar();
    
    private:
    // Occupancy grid for obstacles
    std::vector<std::vector<bool>> occ_grid;

    // The start and goal points
    AStarPoint* start;
    AStarPoint* goal;

    // The open and closed lists
    std::vector<AStarPoint*> open;
    std::vector<AStarPoint*> closed;

    // has side effect of moving the lowest open into
    // the closed list
    AStarPoint* pull_lowest_open();

    // Adds point to the open list while maintaining that
    //  - the open list is sorted by est_cost
    //  - there are no duplicates in the open list
    //  - the point is not in the closed list
    void push_open(AStarPoint* point);
};