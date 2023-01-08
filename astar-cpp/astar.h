#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <ostream>
#include <chrono>
#include <ctime>

enum PointType {
    Obstacle,
    Start,
    Goal,
};

// Represents a point in space
class Point {
    public:
    Point();
    Point(int, int, int);
    PointType type;
    double x;
    double y;
    double z;

    // cout << operator for Point
    friend std::ostream& operator<<(std::ostream& os, const Point& point) {
        os << point.x << " " << point.y << " " << point.z;
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
    double prev_cost;
    double est_cost;
    AStarPoint* parent;

    // cout << operator for AStarPoint
    friend std::ostream& operator<<(std::ostream& os, const AStarPoint& point) {
        os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
        return os;
    }
};

class AStar {
    public:
    AStar(Point, Point, int, Point, Point, std::vector<Point>);
    ~AStar();

    // Runs the AStar algorithm
    std::vector<Point*> run();

    private:
    // 3D Occupancy grid for obstacles
    std::vector<std::vector<std::vector<bool>>> occ_grid;
    Point* tl_point;
    Point* br_point;
    // number of cells in the grid
    int grid_size;
    double grid_cell_len_x; // Length of a cell in the x direction
    double grid_cell_len_y; // Length of a cell in the y direction
    double grid_cell_len_z; // Length of a cell in the z direction

    // The start and goal points
    AStarPoint* start;
    AStarPoint* goal;

    // The open and closed lists
    std::vector<AStarPoint*> open;
    std::vector<AStarPoint*> closed;

    // Cached path to goal
    std::vector<Point*> path;

    // has side effect of moving the lowest open into
    // the closed list
    AStarPoint* pull_lowest_open();

    // Adds point to the open list while maintaining that
    //  - the open list is sorted by est_cost
    //  - there are no duplicates in the open list
    //  - the point is not in the closed list
    void push_open(AStarPoint* point);

    AStarPoint* conv_point(Point* point, AStarPoint* parent);
    Point* conv_astar_point(AStarPoint* point);
    Point* conv_offset_astar_point(AStarPoint* point, int x_off, int y_off, int z_off);

    // Returns the neighbors of a point
    std::vector<AStarPoint*> get_neighbors(AStarPoint* point);

    // Returns the L2 distance between two points
    double dist(AStarPoint* a, AStarPoint* b);
    double dist(double x1, double y1, double z1, double x2, double y2, double z2);

    // Inserts a point into the open list using binary search
    // Returns the index of the point in the open list
    int insert_open(AStarPoint* point);
    int insert_close(AStarPoint* point);
    bool in_open(AStarPoint* point);
    bool in_closed(AStarPoint* point);
};