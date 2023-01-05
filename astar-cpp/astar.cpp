#include "astar.h"

using namespace std;

Point::Point() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->type = PointType::Obstacle;
}

Point::Point(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->type = PointType::Obstacle;
}

// In this constructor, we create a cube of size num_cells x num_cells x num_cells for the occupancy grid
// It may be more efficient to use a non uniform grid, but this will work for now
AStar::AStar(Point tl_corner, Point bl_corner, int num_cells, Point start, Point goal, std::vector<Point> obstacles) {
    this->start = new Point(start.x, start.y, start.z);
    this->goal = new Point(goal.x, goal.y, goal.z);

    // First we need to create the occupancy grid
    int x_size = abs(tl_corner.x - bl_corner.x);
    x_size = x_size / num_cells;

    int y_size = abs(tl_corner.y - bl_corner.y);
    y_size = y_size / num_cells;

    int z_size = abs(tl_corner.z - bl_corner.z);
    z_size = z_size / num_cells;

    // Reshape this->occ_grid to be a 3D vector of size num_cells x num_cells x num_cells
    this->occ_grid.resize(num_cells, vector<vector<bool>>(num_cells, vector<bool>(num_cells, false)));

    // print out the occupancy grid as a sanity check
    for (int i = 0; i < num_cells; i++) {
        for (int j = 0; j < num_cells; j++) {
            for (int k = 0; k < num_cells; k++) {
                cout << this->occ_grid[i][j][k] << " ";
            }
            cout << endl;
        }
        cout << endl;
    }

    // Alrighty, lets fill er in

}

AStar::~AStar() {
    // Delete all the points in the open and closed lists
    for (auto point : open) {
        if (point != nullptr) {
            delete point;
        }
    }
    for (auto point : closed) {
        if (point != nullptr) {
            delete point;
        }
    }

    // Delete the start and goal points
    if (start != nullptr) {
        delete start;
    }

    if (goal != nullptr) {
        delete goal;
    }
}

AStarPoint* AStar::pull_lowest_open() {
    // open is sorted by est_cost, so the last element is the lowest
    AStarPoint* lowest = open.back();
    open.pop_back();

    closed.push_back(lowest);
    return lowest;
}

void AStar::push_open(AStarPoint* point) {
    // Check if the point is already in the open list
    for (auto open_point : open) {
        if (open_point->x == point->x && open_point->y == point->y && open_point->z == point->z) {
            // If the point is already in the open list, check if the new point has a lower cost
            if (point->prev_cost < open_point->prev_cost) {
                // If the new point has a lower cost, replace the old point with the new point
                open_point->prev_cost = point->prev_cost;
                open_point->est_cost = point->est_cost;
                open_point->parent = point->parent;
            }

            // Sort the open list by est_cost
            sort(open.begin(), open.end(), [](AStarPoint* a, AStarPoint* b) {
                return a->est_cost > b->est_cost;
            });

            // Delete the new point
            delete point;
            return;
        }
    }

    // If the point is not in the open or closed list, add it to the open list
    open.push_back(point);

    // Sort the open list by est_cost in descending order
    sort(open.begin(), open.end(), [](AStarPoint* a, AStarPoint* b) {
        return a->est_cost > b->est_cost;
    });    
}
