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
    this->tl_point = new Point(tl_corner.x, tl_corner.y, tl_corner.z);
    this->br_point = new Point(bl_corner.x, bl_corner.y, bl_corner.z);
    this->grid_size = num_cells;

    // First we need to create the occupancy grid
    int x_size = abs(tl_corner.x - bl_corner.x);
    x_size = x_size / num_cells;

    int y_size = abs(tl_corner.y - bl_corner.y);
    y_size = y_size / num_cells;

    int z_size = abs(tl_corner.z - bl_corner.z);
    z_size = z_size / num_cells;

    // Reshape this->occ_grid to be a 3D vector of size num_cells x num_cells x num_cells
    this->occ_grid.resize(num_cells, vector<vector<bool>>(num_cells, vector<bool>(num_cells, false)));

    // Alrighty, lets fill er in
    for (auto obstacle : obstacles) {
        // First we need to find the index of the obstacle in the occupancy grid
        int x_index = (obstacle.x - tl_corner.x) / x_size;
        int y_index = (obstacle.y - tl_corner.y) / y_size;
        int z_index = (obstacle.z - tl_corner.z) / z_size;

        // Now we can set the occupancy grid at that index to true
        this->occ_grid[x_index][y_index][z_index] = true;
    }

    // Lets print out the occupancy grid again to make sure it looks right 
    // for (int i = 0; i < num_cells; i++) {
    //     for (int j = 0; j < num_cells; j++) {
    //         for (int k = 0; k < num_cells; k++) {
    //             cout << this->occ_grid[i][j][k] << " ";
    //         }
    //         cout << endl;
    //     }
    //     cout << endl;
    // }
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

    // Delete the top left and bottom right points
    if (tl_point != nullptr) {
        delete tl_point;
    }

    if (br_point != nullptr) {
        delete br_point;
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

std::vector<Point> AStar::run() {
    // Use the cache if we have already run the algorithm before
    if (this->path.size() > 0) {
        return this->path;
    }

    this->open.clear();
    this->closed.clear();

    // Add the start point to the open list
    this->push_open(this->start);
}

AStarPoint* AStar::conv_point(Point* point, AStarPoint* parent) {
    AStarPoint* astar_point = new AStarPoint();
    astar_point->x = point->x;
    astar_point->y = point->y;
    astar_point->z = point->z;
    astar_point->parent = parent;
    astar_point->prev_cost = parent->prev_cost;

    // Add this->gride size for straight moves and sqrt(2 * this->grid_size) for diagonal moves and sqrt(3 * this->grid_size) for 3D diagonal moves
    // Everytime I write something like this I think that I should return my CS degree
    if ((parent->x == point->x && parent->y == point->y) || (parent->x == point->x && parent->z == point->z) || (parent->y == point->y && parent->z == point->z)) {
        astar_point->prev_cost += this->grid_size;
    } else if ((parent->x == point->x && parent->y != point->y && parent->z != point->z)
            || (parent->x != point->x && parent->y == point->y && parent->z != point->z)
            || (parent->x != point->x && parent->y != point->y && parent->z == point->z)) {
        astar_point->prev_cost += sqrt(2 * this->grid_size);
    } else {
        astar_point->prev_cost += sqrt(3 * this->grid_size);
    }

    astar_point->est_cost = astar_point->prev_cost + dist(point, this->goal);

    return astar_point;
}

Point* AStar::conv_astar_point(AStarPoint* point) {
    Point* conv_point = new Point();
    conv_point->x = point->x;
    conv_point->y = point->y;
    conv_point->z = point->z;

    return conv_point;
}

double AStar::dist(Point* a, Point* b) {
    return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2) + pow(a->z - b->z, 2));
}

double AStar::dist(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

void AStar::push_open(Point* point, AStarPoint* parent) {
    push_open(conv_point(point, parent));
}

void AStar::push_open(Point* point) {
    push_open(conv_point(point, nullptr));
}